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
use super::{SelfTangles, Side, SoftDetectionCtx, SoftEdgeCandidate, SoftEdgePass, elements_cross};

/// The edge-vs-edge candidates between the surfaces of `own` and `other` (itself for a self pass,
/// with `tangles` set); complements the vertex-vs-surface constraints, keeping only contacts
/// interior to both edges. Returns whether the pass runs at all.
pub(crate) fn detect_edges(
    out: &mut SoftEdgePass,
    (sb, mesh, surface_handle, surface_co): Side<'_>,
    (other, other_mesh, other_surface_handle, other_co): Side<'_>,
    tangles: Option<SelfTangles<'_>>,
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
    let Some(other_bvh) = other_co.shape().as_composite_shape().map(|c| c.bvh()) else {
        return false;
    };
    let skins = surface_co.contact_skin() + other_co.contact_skin();
    let motion = ctx.motion_margin(surface_co) + ctx.motion_margin(other_co);
    let scan = EdgeScan {
    };
    let n_e = mesh.indices().len();
    #[cfg(feature = "parallel")]
    if n_e >= 2 * PARALLEL_CHUNK && rayon::current_num_threads() > 1 {
        let chunks: Vec<Vec<SoftEdgeCandidate>> = (0..n_e.div_ceil(PARALLEL_CHUNK))
            .into_par_iter()
            .map(|c| {
                let mut candidates = Vec::new();
                for i in c * PARALLEL_CHUNK..((c + 1) * PARALLEL_CHUNK).min(n_e) {
                    scan.scan_element(i, &mut candidates);
                }
            })
            .collect();
        for chunk in chunks {
            out.candidates.extend(chunk);
        }
    }
    for i in 0..n_e {
        scan.scan_element(i, &mut out.candidates);
    }
}
struct EdgeScan<'a> {
    tangled_elements: &'a [bool],
    crossed_own: &'a [bool],
    crossed_other: &'a [bool],
}
impl EdgeScan<'_> {
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
                    if !box_a.intersects(&crate::geometry::Aabb::new(pb[0].min(pb[1]), pb[0].max(pb[1])))
                    {
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
                // Endpoint contacts are vertex contacts, carried by the vertex rows.
                    ) = (loc_a, loc_b)
                    else {
                        continue;
                    let point_a = pa[0] * ba[0] + pa[1] * ba[1];
                    let point_b = pb[0] * bb[0] + pb[1] * bb[1];
                    let sep = point_b - point_a;
                    let len = sep.length();
                    if len >= self.reach || len < 1.0e-6 {
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
