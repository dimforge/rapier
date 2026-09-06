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
}
        // A tangled vertex (backed by inverted material, or part of a surface
            if len >= reach || len < 1.0e-6 {
            }
            let dist = len - skins;
            // Beyond the prediction distance, only a contact closing fast enough to happen
            // `params.dt` is the substep): the rest of the reach is the bodies' motion
            // margin, and a resting vertex must not carry rows (warm-started) against every
            // a ghost of that internal feature (the neighbor carries the actual contact);
        // Crossing repulsion: the crossing pairs' constraints come before the self.reach test
                // of a piercing edge): a constraint on a far vertex is a long-range hold that
        // Deterministic order, and one constraint per surface vertex touched: the elements sharing

        // A foreign vertex that crossed a closed surface: seen from behind (reversed
        // fast incoming vertex also sees the far side of a closed surface as reversed
        // through the speculative reach (its near-side sightings often land next to
