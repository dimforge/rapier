//! The vertex-vs-surface pass of a pair of soft surfaces (or of a mesh against itself), with the crossing tests it relies on.

use crate::alloc_prelude::*;

#[cfg(feature = "dim2")]
use crate::dynamics::soft_body_crossing_tests::segments_cross;
use crate::dynamics::{SoftBody, SoftCollisionMesh};
use crate::math::{DIM, Real, Vector};
use crate::geometry::PointQueryWithLocation;
use crate::utils::DotProduct;
use parry::bounding_volume::BoundingVolume;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_contacts_classify::{classify_inside, classify_inside_self, project_on_element, ring_depths};
use super::soft_contacts_volume::{VolumeSide, fill_depths, patch_vertices, volume_bins, volume_split};
use super::{SelfTangles, Side, SoftDetectionCtx, SoftVertexPass, SoftVertexCandidate, SoftVertexHits};

/// Whether two elements' boundaries cross transversally, whatever their arities (a
/// segment through a triangle covers a wire through a surface; two wires cross with
/// measure zero).
pub(crate) fn elements_cross(pa: &[Vector], pb: &[Vector]) -> bool {
    #[cfg(feature = "dim2")]
    {
        segments_cross([pa[0], pa[1]], [pb[0], pb[1]])
    }
    #[cfg(feature = "dim3")]
    match (pa.len(), pb.len()) {
        (3, 3) => {
            let (ta, tb) = ([pa[0], pa[1], pa[2]], [pb[0], pb[1], pb[2]]);
            crate::dynamics::soft_body_crossing_tests::tri_pair_crosses(
                &[0, 1, 2],
                &ta,
                &[3, 4, 5],
                &tb,
                false,
            )
        }
        (2, 3) => crate::dynamics::soft_body_crossing_tests::segment_crosses_triangle(
            pa[0], pa[1], pb[0], pb[1], pb[2],
        ),
        (3, 2) => crate::dynamics::soft_body_crossing_tests::segment_crosses_triangle(
            pb[0], pb[1], pa[0], pa[1], pa[2],
        ),
        _ => false,
    }
}

/// The positions of an element's vertices (an element has at most `DIM` vertices; the unused
/// entries stay zero).
#[inline]
pub(crate) fn element_points(mesh: &SoftCollisionMesh, element: &[u32]) -> [Vector; DIM] {
    let mut points = [Vector::ZERO; DIM];
    for (k, &v) in element.iter().enumerate().take(DIM) {
        points[k] = mesh.cached_vertex(v as usize);
    }
    points
}

/// An element's supporting plane: a point on it and its unit normal (`None` for a wire's
/// element or a degenerate one), from the vertex positions `vertex` reads.
pub(crate) fn element_plane(
    eb_mesh: &SoftCollisionMesh,
    element: usize,
    vertex: impl Fn(usize) -> Vector,
) -> Option<(Vector, Vector)> {
    let el = eb_mesh.element(element);
    let p0 = vertex(el[0] as usize);
    #[cfg(feature = "dim2")]
    let n = {
        let d = vertex(el[1] as usize) - p0;
        Vector::new(-d.y, d.x)
    };
    #[cfg(feature = "dim3")]
    let n = {
        if el.len() < 3 {
            return None;
        }
        (vertex(el[1] as usize) - p0).cross(vertex(el[2] as usize) - p0)
    };
    Some((p0, n.try_normalize()?))
}

/// The vertex-vs-surface candidates of the vertices of `vb` against the surface of `eb`
/// (`tangles`: the mesh's self-tangle signal for a self pass, `None` for a pair), with the
/// crossings between the two surfaces and the per-vertex crossing classification.
pub(crate) fn detect_vertex_pass(
    out: &mut SoftVertexPass,
    (eb, eb_mesh, surface_handle, eb_co): Side<'_>,
    (vb, vb_mesh, vertices_handle, vb_co): Side<'_>,
    tangles: Option<SelfTangles<'_>>,
    ctx: &SoftDetectionCtx,
) {
    let params = ctx.params;
    let step_dt = ctx.dt;
    let is_self = tangles.is_some();
    out.clear();
    out.surface = surface_handle;
    out.vertices_of = vertices_handle;
    let Some(eb_bvh) = eb_co.shape().as_composite_shape().map(|c| c.bvh()) else {
        return;
    };
    // Crossings between the two surfaces: where the boundaries already pass through each other,
    // the pair's keep-apart constraints are wrong-sided, so they stand down around the crossing,
    // like the self constraints do. 3D wires sit out (curve-curve crossings have measure zero).
    if !is_self && params.soft_bodies.recovery.cross_body_detection {
        if let Some(vb_bvh) = vb_co.shape().as_composite_shape().map(|c| c.bvh()) {
            let vb_inv_pose = vb_co.position().inverse();
            for e in 0..eb_mesh.indices().len() {
                let ee = eb_mesh.element(e);
                let pe = element_points(eb_mesh, ee);
                let mut aabb = crate::geometry::Aabb::new_invalid();
                for p in &pe[..ee.len()] {
                    aabb.take_point(*p);
                }
                let aabb = aabb.transform_by(&vb_inv_pose);
                for f in vb_bvh.intersect_aabb(&aabb) {
                    let ef = vb_mesh.element(f as usize);
                    let qf = element_points(vb_mesh, ef);
                    if !elements_cross(&pe[..ee.len()], &qf[..ef.len()]) {
                        continue;
                    }
                    if out.cross_tangled_elements.is_empty() {
                        out.cross_tangled_elements
                            .resize(eb_mesh.indices().len(), false);
                        out.cross_tangled_vertices
                            .resize(vb_mesh.vertex_count(), false);
                    }
                    out.cross_tangled_elements[e] = true;
                    for &v in ef {
                        out.cross_tangled_vertices[v as usize] = true;
                    }
                    {
                        if out.cross_tangled_vb_elements.is_empty() {
                            out.cross_tangled_vb_elements
                                .resize(vb_mesh.indices().len(), false);
                        }
                        out.cross_tangled_vb_elements[f as usize] = true;
                        let pair = (e as u32, f);
                        if out.cross_pairs.len() < 256 && !out.cross_pairs.contains(&pair) {
                            out.cross_pairs.push(pair);
                        }
                    }
                }
            }
        }
    }
    // The two contact skins (a soft surface's is its vertices' thickness; between two pieces of
    // one torn body, capped per pair by the gap at rest, see `rest_gap_skins`); the
    // speculative reach grows with both bodies' motion over the step (see the narrow phase).
    let skins = vb_co.contact_skin() + eb_co.contact_skin();
    let prediction = params.prediction_distance();
    let reach = prediction + skins + ctx.motion_margin(eb_co) + ctx.motion_margin(vb_co);
    // A closed surface's reversed sightings mark a foreign vertex inside it.
    let closed = eb_mesh.is_closed() && !is_self;

    // Crossing repulsion (see `repel_constraint`): the detected crossing pairs also get their own
    // constraints (the piercing element's vertices against the pierced element), so an edge-first
    // crossing with no vertex within reach is repelled too. `targets[f]`: elements crossed by `f`.
    let (tangled_vertices, tangled_elements, crossings): (&[bool], &[bool], &[(u32, u32)]) =
        match &tangles {
            Some(t) => (t.tangled_vertices, t.tangled_elements, t.crossings),
            None => (&[], &[], &[]),
        };
    let repel = params.soft_bodies.recovery.crossing_repulsion
        && !vb_mesh.is_wire()
        && !eb_mesh.is_wire()
        && if is_self {
            !tangled_vertices.is_empty() || !tangled_elements.is_empty()
        } else {
            !out.cross_tangled_vertices.is_empty() || !out.cross_tangled_elements.is_empty()
        };
    let (targets, vertex_elements): (Vec<Vec<u32>>, Vec<Vec<u32>>) = if repel {
        let mut targets = vec![Vec::new(); vb_mesh.indices().len()];
        if is_self {
            for &(i, j) in crossings {
                targets[i as usize].push(j);
                targets[j as usize].push(i);
            }
        } else {
            for &(e, f) in &out.cross_pairs {
                targets[f as usize].push(e);
            }
        }
        let mut vertex_elements = vec![Vec::new(); vb_mesh.vertex_count()];
        for f in 0..vb_mesh.indices().len() {
            for &v in vb_mesh.element(f) {
                vertex_elements[v as usize].push(f as u32);
            }
        }
        (targets, vertex_elements)
    } else {
        (Vec::new(), Vec::new())
    };
    // The guiding volume normals (see `crossing_repulsion_guide`): the intersection patches of the
    // two closed surfaces, binned like the volume contact, computed in every vertex pass and
    // without any volume constraint; the vertices inside the surface side's patch come with them.
    let patch_policy = params.soft_bodies.recovery.overlap_patch_constraints;
    if ((repel && params.soft_bodies.recovery.crossing_repulsion_guide)
        || patch_policy != crate::dynamics::SoftPatchConstraints::Keep)
        && !is_self
        && vb_mesh.is_closed()
        && eb_mesh.is_closed()
        && !out.cross_pairs.is_empty()
    {
        let mut inside_vb = Vec::new();
        let mut inside_eb = Vec::new();
        let mut depth_vb = Vec::new();
        let mut depth_eb = Vec::new();
        classify_inside(
            vb_mesh,
            vb,
            &out.cross_tangled_vb_elements,
            (eb_mesh, eb),
            &mut inside_vb,
        );
        classify_inside(
            eb_mesh,
            eb,
            &out.cross_tangled_elements,
            (vb_mesh, vb),
            &mut inside_eb,
        );
        let a1 = fill_depths(vb_mesh, vb, &inside_vb, eb_co, &mut depth_vb);
        let a2 = fill_depths(eb_mesh, eb, &inside_eb, vb_co, &mut depth_eb);
        if patch_policy != crate::dynamics::SoftPatchConstraints::Keep {
            out.patch_inside_vb = inside_vb;
        }
        if a1 || a2 {
            if a1 {
                ring_depths(vb_mesh, vb, eb_co, 0.0, &mut depth_vb);
            }
            if a2 {
                ring_depths(eb_mesh, eb, vb_co, 0.0, &mut depth_eb);
            }
            let own = VolumeSide {
                body: eb,
                mesh: eb_mesh,
                vertices: patch_vertices(eb_mesh, eb, &depth_eb, 0.0),
            };
            let other = VolumeSide {
                body: vb,
                mesh: vb_mesh,
                vertices: patch_vertices(vb_mesh, vb, &depth_vb, 0.0),
            };
            for bin in volume_bins(&own, Some(&other), volume_split(params)) {
                if bin.normal != Vector::ZERO {
                    out.repel_guides.push((bin.center, bin.normal));
                }
            }
        }
    }
    // The self guide (see `crossing_repulsion_self_guide`): the fold's vertices (in their body's
    // self-intersection region) form the own side, the nearest elements outside their neighborhood
    // the other side; each cell's normal, from the fold toward the facing surface, is the push.
    let self_guided = repel
        && params.soft_bodies.recovery.crossing_repulsion_self_guide
        && is_self
        && vb_mesh.is_closed()
        && !crossings.is_empty();
    if self_guided {
        let n_v = vb_mesh.vertex_count();
        let mut crossing = vec![false; vb_mesh.indices().len()];
        for &(i, j) in crossings {
            crossing[i as usize] = true;
            crossing[j as usize] = true;
        }
        let mut inside = Vec::new();
        if classify_inside_self(vb_mesh, vb, vb_co.contact_skin(), &crossing, &vertex_elements, &mut inside) {
            let mut depth_fold = vec![Real::NEG_INFINITY; n_v];
            let mut depth_face = vec![Real::NEG_INFINITY; n_v];
            for v in 0..n_v {
                if !inside[v] {
                    continue;
                }
                let p = vb_mesh.cached_vertex(v);
                let ring = &vb_mesh.ring
                    [vb_mesh.ring_offsets[v] as usize..vb_mesh.ring_offsets[v + 1] as usize];
                let mut best: Option<(Real, usize)> = None;
                for e in 0..vb_mesh.indices().len() {
                    let el = vb_mesh.element(e);
                    if el.iter().any(|&u| u as usize == v || ring.contains(&u)) {
                        continue;
                    }
                    let Some((q, _)) = project_on_element(vb_mesh, vb, e, p) else {
                        continue;
                    };
                    let d2 = (q - p).length_squared();
                    if best.is_none_or(|b| d2 < b.0) {
                        best = Some((d2, e));
                    }
                }
                let Some((d2, e)) = best else {
                    continue;
                };
                depth_fold[v] = d2.sqrt();
                for &u in vb_mesh.element(e) {
                    depth_face[u as usize] = 0.0;
                }
            }
            let own = VolumeSide {
                body: vb,
                mesh: vb_mesh,
                vertices: patch_vertices(vb_mesh, vb, &depth_fold, 0.0),
            };
            let other = VolumeSide {
                body: vb,
                mesh: vb_mesh,
                vertices: patch_vertices(vb_mesh, vb, &depth_face, 0.0),
            };
            for bin in volume_bins(&own, Some(&other), volume_split(params)) {
                if bin.normal != Vector::ZERO {
                    out.repel_guides.push((bin.center, bin.normal));
                }
            }
        }
        out.repel_inside = inside;
    }
    // The per-vertex enumeration, chunked across threads for a large vertex side (the chunks'
    // outputs are appended in order: the same candidates as the serial loop).
    let SoftVertexPass {
        cross_tangled_vertices,
        hits,
        candidates,
        ..
    } = out;
    let scan = VertexScan {
        eb,
        eb_mesh,
        eb_co,
        eb_bvh,
        eb_inv_pose: eb_co.position().inverse(),
        bounds: eb_co.compute_aabb().loosened(reach),
        vb,
        vb_mesh,
        vb_co,
        is_self,
        params,
        step_dt,
        reach,
        prediction,
        skins,
        closed,
        rest_gaps,
        tangled_vertices,
        tangled_elements,
        repel,
        targets: &targets,
        vertex_elements: &vertex_elements,
        cross_tangled_vertices,
    };
    let n_v = vb_mesh.vertex_count();
    #[cfg(feature = "parallel")]
    if n_v >= 2 * PARALLEL_CHUNK && rayon::current_num_threads() > 1 {
        use rayon::prelude::*;
        let chunks: Vec<(Vec<SoftVertexHits>, Vec<SoftVertexCandidate>)> = (0..n_v
            .div_ceil(PARALLEL_CHUNK))
            .into_par_iter()
            .map(|c| {
                let (mut hits, mut candidates) = (Vec::new(), Vec::new());
                let mut scratch = VertexScratch::default();
                for v in c * PARALLEL_CHUNK..((c + 1) * PARALLEL_CHUNK).min(n_v) {
                    scan.scan_vertex(v, &mut scratch, &mut hits, &mut candidates);
                }
                (hits, candidates)
            })
            .collect();
        for (chunk_hits, chunk_candidates) in chunks {
            let offset = candidates.len() as u32;
            hits.extend(chunk_hits.into_iter().map(|h| SoftVertexHits {
                candidates: h.candidates.start + offset..h.candidates.end + offset,
                ..h
            }));
            candidates.extend(chunk_candidates);
        }
        return;
    }
    let mut scratch = VertexScratch::default();
    for v in 0..n_v {
        scan.scan_vertex(v, &mut scratch, hits, candidates);
    }
}

/// The per-thread buffers of a vertex pass: the candidates of the vertex at hand, and the
/// self-exclusion stamp table (see `SoftCollisionMesh::mark_self_contact_exclusions`).
#[derive(Default)]
struct VertexScratch {
    candidates: Vec<SoftVertexCandidate>,
    stamps: Vec<u32>,
}

/// Vertices (or elements) per parallel chunk of the vertex and edge passes.
#[cfg(feature = "parallel")]
pub(super) const PARALLEL_CHUNK: usize = 512;

/// The read-only state of a vertex pass's per-vertex enumeration (see [`VertexScan::scan_vertex`]).
struct VertexScan<'a> {
    eb: &'a SoftBody,
    eb_mesh: &'a SoftCollisionMesh,
    eb_co: &'a crate::geometry::Collider,
    eb_bvh: &'a parry::partitioning::Bvh,
    eb_inv_pose: crate::math::Pose,
    /// The surface side's world bounds, loosened by the reach.
    bounds: crate::geometry::Aabb,
    vb: &'a SoftBody,
    vb_mesh: &'a SoftCollisionMesh,
    vb_co: &'a crate::geometry::Collider,
    is_self: bool,
    params: &'a crate::dynamics::IntegrationParameters,
    step_dt: Real,
    reach: Real,
    prediction: Real,
    skins: Real,
    closed: bool,
    rest_gaps: bool,
    tangled_vertices: &'a [bool],
    tangled_elements: &'a [bool],
    repel: bool,
    targets: &'a [Vec<u32>],
    vertex_elements: &'a [Vec<u32>],
    cross_tangled_vertices: &'a [bool],
}

impl VertexScan<'_> {
    /// The candidates of the vertex `v` (appended to `out_candidates`, with its hit record in
    /// `hits`); `scratch` is the per-vertex candidate buffer.
    fn scan_vertex(
        &self,
        v: usize,
        scratch: &mut VertexScratch,
        hits: &mut Vec<SoftVertexHits>,
        out_candidates: &mut Vec<SoftVertexCandidate>,
    ) {
        let VertexScratch {
            candidates: scratch,
            stamps,
        } = scratch;
        // Surface vertices only.
        if !self.vb_mesh.vertex_on_surface(v) {
            return;
        }
        // A tangled vertex (backed by inverted material, or part of a surface
        // self-crossing): its self contacts stand down (see `detect_self_tangles`).
        if self.is_self
            && self.params.soft_bodies.recovery.self_stand_down
            && !self.params.soft_bodies.recovery.crossing_repulsion
            && self.tangled_vertices.get(v).copied().unwrap_or(false)
        {
            return;
        }
        let vertex_pos = self.vb_mesh.cached_vertex(v);
        let mut vertex_vel: Option<Vector> = None;
        // A vertex beyond the surface's reach-loosened bounds has no candidate (its crossing
        // repulsion constraints, if flagged, do not go through the reach).
        let flagged_vertex = self.repel
            && if self.is_self {
                self.tangled_vertices.get(v).copied().unwrap_or(false)
            } else {
                self.cross_tangled_vertices.get(v).copied().unwrap_or(false)
            };
        if !self.is_self && !flagged_vertex && !self.bounds.contains_local_point(vertex_pos) {
            return;
        }
        if self.is_self {
            // The elements excluded around this vertex, stamped once.
            let n_e = self.eb_mesh.indices().len();
            if stamps.len() != n_e {
                stamps.clear();
                stamps.resize(n_e, u32::MAX);
            }
            self.vb_mesh.mark_self_contact_exclusions(v as u32, stamps);
        }

        // Candidate elements: the vertex's reach box against the surface's BVH. The
        // shape's vertices live in its cluster frame: the world box is localized first.
        scratch.clear();
        let aabb = crate::geometry::Aabb::from_half_extents(vertex_pos, Vector::splat(self.reach))
            .transform_by(&self.eb_inv_pose);
        for e in self.eb_bvh.intersect_aabb(&aabb) {
            let element = self.eb_mesh.element(e as usize);
            if self.is_self
                && (stamps[e as usize] == v as u32
                    || self.vb_mesh.self_contact_excluded_at_rest(
                        self.vb,
                        v as u32,
                        element,
                        self.vb_co.contact_skin(),
                    ))
            {
                continue;
            }
            // A tangled element (its cell is inverted, or it is part of a surface
            // self-crossing): its self contacts stand down (see `detect_self_tangles`).
            if self.is_self
                && self.params.soft_bodies.recovery.self_stand_down
                && !self.params.soft_bodies.recovery.crossing_repulsion
                && self.tangled_elements.get(e as usize).copied().unwrap_or(false)
            {
                continue;
            }
            let positions: [Vector; DIM] = core::array::from_fn(|k| {
                element
                    .get(k)
                    .map_or(Vector::ZERO, |v| self.eb_mesh.cached_vertex(*v as usize))
            });
            // A segment element (every mesh in 2D, a wire in 3D) or a triangle: the unused
            // weights stay at zero.
            let (proj, weights) = if element.len() < DIM {
                let (proj, loc) = parry::shape::Segment::new(positions[0], positions[1])
                    .project_local_point_and_get_location(vertex_pos, false);
                let bcoords = loc.barycentric_coordinates();
                let mut weights = [0.0; DIM];
                weights[0] = bcoords[0];
                weights[1] = bcoords[1];
                (proj, weights)
            } else {
                #[cfg(feature = "dim2")]
                {
                    let (proj, loc) = parry::shape::Segment::new(positions[0], positions[1])
                        .project_local_point_and_get_location(vertex_pos, false);
                    (proj, loc.barycentric_coordinates())
                }
                #[cfg(feature = "dim3")]
                {
                    let (proj, loc) =
                        parry::shape::Triangle::new(positions[0], positions[1], positions[2])
                            .project_local_point_and_get_location(vertex_pos, false);
                    let Some(weights) = loc.barycentric_coordinates() else {
                        continue;
                    };
                    (proj, weights)
                }
            };
            let sep = vertex_pos - proj.point;
            let len = sep.length();
            if len >= reach || len < 1.0e-6 {
                continue;
            }
            let dist = len - skins;
            // Beyond the prediction distance, only a contact closing fast enough to happen within
            // the whole step is kept (`self.params.dt` is the substep): the rest of the reach is
            // the bodies' motion margin, and a resting vertex must not keep constraints in it.
            if dist >= self.prediction {
                let vertex_vel =
                    vertex_vel.get_or_insert_with(|| self.vb_mesh.vertex_velocity(self.vb, v));
                let mut surface_vel = Vector::ZERO;
                for (k, v) in element.iter().enumerate() {
                    surface_vel += self.eb_mesh.vertex_velocity(self.eb, *v as usize) * weights[k];
                }
                let closing = (surface_vel - *vertex_vel).gdot(sep / len);
                if dist - closing * self.step_dt >= self.prediction {
                    continue;
                }
            }
            // A vertex/edge contact whose vertex projects inside a neighboring element is a ghost
            // of that internal feature (the neighbor reports the real contact); vertices thicker
            // than the element wrap several elements, so their feature contacts are real support.
            let size = (positions[1] - positions[0]).length();
            if self.vb_co.contact_skin() < size
                && self.eb_mesh.contact_is_ghost(self.eb, e as usize, &weights, vertex_pos)
            {
                continue;
            }
            let outward = if self.closed {
                self.eb_mesh
                    .element_outward_normal(self.eb, e as usize)
                    .and_then(|n| n.try_normalize())
            } else {
                None
            };
            scratch.push(SoftVertexCandidate {
                element: e,
                weights,
                dir: -sep / len,
                dist,
                outward,
                interior: weights.iter().all(|w| *w > 0.02),
                enabled: true,
                impulse: 0.0,
                tangent_impulse: Vector::ZERO,
            });
        }
        // Crossing repulsion: the crossing pairs' constraints come before the self.reach test
        // (an edge-first crossing vertex has no natural candidate at all).
        if self.repel {
            let flagged_vertex = if self.is_self {
                self.tangled_vertices.get(v).copied().unwrap_or(false)
            } else {
                self.cross_tangled_vertices.get(v).copied().unwrap_or(false)
            };
            if flagged_vertex {
                // Only the vertices near the pierced element's plane (the near endpoint
                // of a piercing edge): a constraint on a far vertex is a long-range hold that
                // blocks the resolution instead of driving it.
                let bound = 2.0 * self.skins + self.eb_co.contact_skin();
                for &f in &self.vertex_elements[v] {
                    for &e in &self.targets[f as usize] {
                        let near = element_plane(self.eb_mesh, e as usize, |v| self.eb_mesh.cached_vertex(v))
                            .is_some_and(|(p0, n)| (vertex_pos - p0).dot(n).abs() <= bound);
                        if near && scratch.iter().all(|c| c.element != e) {
                            scratch.push(SoftVertexCandidate {
                                element: e,
                                weights: [1.0 / DIM as Real; DIM],
                                dir: Vector::ZERO,
                                dist: 0.0,
                                outward: None,
                                interior: false,
                                enabled: true,
                                impulse: 0.0,
                                tangent_impulse: Vector::ZERO,
                            });
                        }
                    }
                }
            }
        }
        if scratch.is_empty() {
            return;
        }
        // Deterministic order, and one constraint per surface vertex touched: the elements sharing
        // that vertex all report it as their closest point (edge contacts shared by two
        // elements are kept twice, as the manifold path does).
        scratch.sort_by_key(|c| c.element);
        let mut kept = 0;
        for i in 0..scratch.len() {
            let c = scratch[i];
            let element = self.eb_mesh.element(c.element as usize);
            let vertex = (0..element.len()).find(|&k| c.weights[k] > 0.999);
            if let Some(k) = vertex {
                let key = element[k];
                if scratch[..kept].iter().any(|prev| {
                    let prev_element = self.eb_mesh.element(prev.element as usize);
                    (0..prev_element.len())
                        .any(|j| prev.weights[j] > 0.999 && prev_element[j] == key)
                }) {
                    continue;
                }
            }
            scratch[kept] = c;
            kept += 1;
        }
        scratch.truncate(kept);

        // A foreign vertex that crossed a closed surface (seen only from behind, inside by parity)
        // hands its pair to the recovery. A fast incoming vertex also sees the far side as reversed
        // through the speculative reach (near-side sightings often fail the interior gate).
        let mut seen_outside_any = false;
        let mut seen_reversed = false;
        for c in scratch.iter() {
            let Some(outward) = c.outward else {
                continue;
            };
            let side = c.dir.gdot(outward);
            if side.abs() <= 0.5 {
                continue;
            }
            if side < 0.0 {
                seen_outside_any = true;
            } else if c.interior {
                seen_reversed = true;
            }
        }
        let crossed = self.closed
            && seen_reversed
            && !seen_outside_any
            && self.eb_mesh.contains_point_parity(self.eb, vertex_pos);
        let start = out_candidates.len() as u32;
        out_candidates.extend_from_slice(&scratch);
        hits.push(SoftVertexHits {
            vertex: v as u32,
            candidates: start..out_candidates.len() as u32,
            crossed,
        });
    }
}
