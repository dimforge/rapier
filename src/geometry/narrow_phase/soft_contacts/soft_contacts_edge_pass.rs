//! The edge-vs-edge pass of a pair of soft surfaces (or of a mesh against itself).

use crate::alloc_prelude::*;

use crate::dynamics::{SoftBody, SoftCollisionMesh};
use crate::math::{Real, Vector};
use crate::utils::DotProduct;
use parry::bounding_volume::BoundingVolume;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

#[cfg(feature = "parallel")]
use super::soft_contacts_vertex_pass::PARALLEL_CHUNK;
use super::soft_contacts_vertex_pass::element_points;
use super::{
    SelfTangles, Side, SoftDetectionCtx, SoftEdgeCandidate, SoftEdgePass, elements_cross,
    rest_gap_skins,
};

/// The edge-vs-edge candidates between the surfaces of `own` and `other` (itself for a self pass,
/// with `tangles` set); complements the vertex-vs-surface constraints, keeping only contacts
/// interior to both edges. Returns whether the pass runs at all.
pub(crate) fn detect_edges(
    out: &mut SoftEdgePass,
    (sb, mesh, surface_handle, surface_co): Side<'_>,
    (other, other_mesh, other_surface_handle, other_co): Side<'_>,
    tangles: Option<SelfTangles<'_>>,
    rest_gaps: bool,
    ctx: &SoftDetectionCtx,
) -> bool {
    let params = ctx.params;
    let step_dt = ctx.dt;
    let is_self = tangles.is_some();
    out.candidates.clear();
    out.crossed = false;
    out.slot_offset = u32::MAX;
    out.own = surface_handle;
    out.other = other_surface_handle;
    // In 2D two closed surfaces meet through their vertex constraints alone (point-vs-segment is
    // complete for first contact); in 3D two edge-leading closed bodies (slim bars crossing
    // corner-first) have no vertex near the other's surface and need the edge constraints.
    #[cfg(feature = "dim2")]
    if mesh.is_closed() && other_mesh.is_closed() {
        return false;
    }
    #[cfg(feature = "dim3")]
    if !params.soft_bodies.recovery.edge_speculation && mesh.is_closed() && other_mesh.is_closed() {
        return false;
    }
    let speculation = params.soft_bodies.recovery.edge_speculation;
    let Some(other_bvh) = other_co.shape().as_composite_shape().map(|c| c.bvh()) else {
        return false;
    };
    let skins = surface_co.contact_skin() + other_co.contact_skin();
    // The motion margin only matters when the pair can skip the skin band in
    // a step; a resting pair keeps the tight reach (candidate enumeration is the edge
    // pass's cost).
    let motion = ctx.motion_margin(surface_co) + ctx.motion_margin(other_co);
    let reach = params.prediction_distance()
        + skins
        + if speculation && motion > 0.25 * skins {
            motion
        } else {
            0.0
        };
    let tangled_elements: &[bool] = tangles.as_ref().map_or(&[], |t| t.tangled_elements);

    // Crossings between the two surfaces: where they already pass through each other, edge
    // constraints are wrong-sided and would freeze the crossing, so element pairs touching a
    // crossing stand down. Mixed arities cover a wire through a surface.
    let mut crossed_own: Vec<bool> = Vec::new();
    let mut crossed_other: Vec<bool> = Vec::new();
    let other_inv_pose = other_co.position().inverse();
    if !is_self
        && params.soft_bodies.recovery.cross_body_detection
        && params.soft_bodies.recovery.edge_stand_down
    {
        for i in 0..mesh.indices().len() {
            let element = mesh.element(i);
            let pa = element_points(mesh, element);
            let mut aabb = crate::geometry::Aabb::new_invalid();
            for p in &pa[..element.len()] {
                aabb.take_point(*p);
            }
            let aabb = aabb.transform_by(&other_inv_pose);
            for j in other_bvh.intersect_aabb(&aabb) {
                let ej = other_mesh.element(j as usize);
                let pb = element_points(other_mesh, ej);
                if !elements_cross(&pa[..element.len()], &pb[..ej.len()]) {
                    continue;
                }
                if crossed_own.is_empty() {
                    crossed_own.resize(mesh.indices().len(), false);
                    crossed_other.resize(other_mesh.indices().len(), false);
                }
                crossed_own[i] = true;
                crossed_other[j as usize] = true;
            }
        }
        // The pair is recovery-owned: the crossing guard leaves it alone.
        out.crossed = !crossed_own.is_empty();
    }

    // The candidate element pairs: every element of this surface against the other's BVH (in its
    // cluster frame, so the world box is localized first), AABB grown by the contact reach; then
    // their edge pairs. Chunked across threads for a large surface, candidates appended in order.
    let scan = EdgeScan {
        sb,
        mesh,
        other,
        other_mesh,
        other_bvh,
        other_inv_pose,
        reach,
        is_self,
        tangled_elements,
        crossed_own: &crossed_own,
        crossed_other: &crossed_other,
        speculation,
        rest_gaps,
        skins,
        params,
        step_dt,
    };
    let n_e = mesh.indices().len();
    #[cfg(feature = "parallel")]
    if n_e >= 2 * PARALLEL_CHUNK && rayon::current_num_threads() > 1 {
        use rayon::prelude::*;
        let chunks: Vec<Vec<SoftEdgeCandidate>> = (0..n_e.div_ceil(PARALLEL_CHUNK))
            .into_par_iter()
            .map(|c| {
                let mut candidates = Vec::new();
                for i in c * PARALLEL_CHUNK..((c + 1) * PARALLEL_CHUNK).min(n_e) {
                    scan.scan_element(i, &mut candidates);
                }
                candidates
            })
            .collect();
        for chunk in chunks {
            out.candidates.extend(chunk);
        }
        return true;
    }
    for i in 0..n_e {
        scan.scan_element(i, &mut out.candidates);
    }
    true
}

/// The read-only state of an edge pass's per-element enumeration (see [`EdgeScan::scan_element`]).
struct EdgeScan<'a> {
    sb: &'a SoftBody,
    mesh: &'a SoftCollisionMesh,
    other: &'a SoftBody,
    other_mesh: &'a SoftCollisionMesh,
    other_bvh: &'a parry::partitioning::Bvh,
    other_inv_pose: crate::math::Pose,
    reach: Real,
    is_self: bool,
    tangled_elements: &'a [bool],
    crossed_own: &'a [bool],
    crossed_other: &'a [bool],
    speculation: bool,
    rest_gaps: bool,
    skins: Real,
    params: &'a crate::dynamics::IntegrationParameters,
    step_dt: Real,
}

impl EdgeScan<'_> {
    /// The edge candidates of the element `i` against the other surface, appended to `out`.
    fn scan_element(&self, i: usize, out: &mut Vec<SoftEdgeCandidate>) {
        let (mesh, other_mesh) = (self.mesh, self.other_mesh);
        let element = mesh.element(i);
        let mut aabb = crate::geometry::Aabb::new_invalid();
        for &v in element {
            aabb.take_point(mesh.cached_vertex(v as usize));
        }
        let aabb = aabb.loosened(self.reach).transform_by(&self.other_inv_pose);
        for j in self.other_bvh.intersect_aabb(&aabb) {
            let i = i as u32;
            if self.is_self {
                if j <= i {
                    continue;
                }
                let (ei, ej) = (mesh.element(i as usize), other_mesh.element(j as usize));
                if ei.iter().any(|v| ej.contains(v)) {
                    continue;
                }
                // Tangled elements (inverted cells, or part of a surface self-crossing):
                // their self contacts stand down (see `detect_self_tangles`).
                if self
                    .tangled_elements
                    .get(i as usize)
                    .copied()
                    .unwrap_or(false)
                    || self
                        .tangled_elements
                        .get(j as usize)
                        .copied()
                        .unwrap_or(false)
                {
                    continue;
                }
            } else if self.crossed_own.get(i as usize).copied().unwrap_or(false)
                || self.crossed_other.get(j as usize).copied().unwrap_or(false)
            {
                continue;
            }
            // The edges owned by each element (a segment is its own edge, in 2D and for a
            // wire in 3D): a pair of edges is tested by exactly one element pair.
            for ea in self.mesh.element_edge_ids(i as usize) {
                if self.mesh.edge_owner(ea) != i {
                    continue;
                }
                let va = self.mesh.edge_vertices(ea);
                let pa = [
                    self.mesh.cached_vertex(va[0] as usize),
                    self.mesh.cached_vertex(va[1] as usize),
                ];
                // The edge's reach box: an edge pair whose boxes miss is farther apart than
                // the reach (the box distance bounds the segment distance from below).
                let box_a = crate::geometry::Aabb::new(pa[0].min(pa[1]), pa[0].max(pa[1]))
                    .loosened(self.reach);
                for eb in self.other_mesh.element_edge_ids(j as usize) {
                    if self.other_mesh.edge_owner(eb) != j {
                        continue;
                    }
                    let vb = self.other_mesh.edge_vertices(eb);
                    if self.is_self && va.iter().any(|v| vb.contains(v)) {
                        continue;
                    }
                    let pb = [
                        self.other_mesh.cached_vertex(vb[0] as usize),
                        self.other_mesh.cached_vertex(vb[1] as usize),
                    ];
                    if !box_a.intersects(&crate::geometry::Aabb::new(
                        pb[0].min(pb[1]),
                        pb[0].max(pb[1]),
                    )) {
                        continue;
                    }
                    // Near-parallel edges never cross: their proximity is a face contact,
                    // handled by the vertex constraints (and their closest points are ill-defined).
                    let (da, db) = (pa[1] - pa[0], pb[1] - pb[0]);
                    #[cfg(feature = "dim2")]
                    let sin_sq = {
                        let c = da.perp_dot(db);
                        c * c
                    };
                    #[cfg(feature = "dim3")]
                    let sin_sq = da.cross(db).length_squared();
                    if sin_sq < 0.01 * da.length_squared() * db.length_squared() {
                        continue;
                    }
                    let (loc_a, loc_b) =
                        parry::query::details::closest_points_segment_segment_with_locations_nD(
                            (&pa[0], &pa[1]),
                            (&pb[0], &pb[1]),
                        );
                    // Endpoint contacts are vertex contacts, handled by the vertex constraints.
                    let (
                        parry::shape::SegmentPointLocation::OnEdge(ba),
                        parry::shape::SegmentPointLocation::OnEdge(bb),
                    ) = (loc_a, loc_b)
                    else {
                        continue;
                    };
                    let point_a = pa[0] * ba[0] + pa[1] * ba[1];
                    let point_b = pb[0] * bb[0] + pb[1] * bb[1];
                    let sep = point_b - point_a;
                    let len = sep.length();
                    if len >= self.reach || len < 1.0e-6 {
                        continue;
                    }
                    // Two pieces of one torn body keep the gap the edges had at rest.
                    let pair_skins = if self.rest_gaps {
                        let ra = [
                            self.mesh.rest_vertex(self.sb, va[0] as usize),
                            self.mesh.rest_vertex(self.sb, va[1] as usize),
                        ];
                        let rb = [
                            self.other_mesh.rest_vertex(self.other, vb[0] as usize),
                            self.other_mesh.rest_vertex(self.other, vb[1] as usize),
                        ];
                        let (la, lb) =
                            parry::query::details::closest_points_segment_segment_with_locations_nD(
                                (&ra[0], &ra[1]),
                                (&rb[0], &rb[1]),
                            );
                        let (ca, cb) = (la.barycentric_coordinates(), lb.barycentric_coordinates());
                        let rest_gap =
                            (ra[0] * ca[0] + ra[1] * ca[1] - rb[0] * cb[0] - rb[1] * cb[1])
                                .length();
                        rest_gap_skins(self.skins, rest_gap)
                    } else {
                        self.skins
                    };
                    let dist = len - pair_skins;
                    // Beyond the prediction distance, only a contact closing fast enough
                    // to happen within the whole step is kept (the vertex pass's rule):
                    // the rest of the self.reach is the bodies' motion margin.
                    if dist >= self.params.prediction_distance() {
                        if !self.speculation {
                            continue;
                        }
                        let vel_a = self.mesh.vertex_velocity(self.sb, va[0] as usize) * ba[0]
                            + self.mesh.vertex_velocity(self.sb, va[1] as usize) * ba[1];
                        let (other, other_mesh) = (self.other, self.other_mesh);
                        let vel_b = other_mesh.vertex_velocity(other, vb[0] as usize) * bb[0]
                            + other_mesh.vertex_velocity(other, vb[1] as usize) * bb[1];
                        let closing = (vel_a - vel_b).gdot(sep / len);
                        if dist - closing * self.step_dt >= self.params.prediction_distance() {
                            continue;
                        }
                    }
                    // Force direction on this body's edge (away from the self.other edge).
                    let dir = -sep / len;
                    out.push(SoftEdgeCandidate {
                        edge: ea,
                        other_edge: eb,
                        bcoords: ba,
                        other_bcoords: bb,
                        point: point_a,
                        other_point: point_b,
                        dir,
                        dist,
                        enabled: true,
                        impulse: 0.0,
                        tangent_impulse: Vector::ZERO,
                    });
                }
            }
        }
    }
}
