//! Solver-side clustering of contact manifolds with (nearly) parallel normals.
//!
//! Composite shapes produce one manifold per subshape; on flat patches most share a normal,
//! giving the solver redundant constraints for one contact plane. This merges them into
//! per-normal "cluster" manifolds rebuilt every frame — only clusters reach the solver; the
//! per-subshape manifolds stay untouched for user-facing queries and events. Warm-start impulses
//! live in the cluster points, carried by nearest-position matching (cluster identity is unstable).

use crate::alloc_prelude::*;
use crate::geometry::{ContactManifold, ContactManifoldData};
use crate::math::{Real, Vector};

/// Two manifolds are merged if their contact normals agree within ~5.1 degrees.
const COS_MERGE_ANGLE: Real = 0.996;
/// Hard cap on points per cluster (the solver stores point indices as `u8`). Reduction
/// selects at most 4 for the constraints; keeping all deduplicated points until then makes
/// selection and warm-start carry independent of the manifold iteration order.
const MAX_CLUSTER_POINTS: usize = 255;

fn manifold_normal1(manifold: &ContactManifold) -> Vector {
    manifold
        .subshape_pos1()
        .map(|p| p.rotation * manifold.local_n1)
        .unwrap_or(manifold.local_n1)
}

fn has_warmstart_data(data: &crate::geometry::ContactData) -> bool {
    data.impulse != 0.0 || data.warmstart_impulse != 0.0
}

/// Rebuilds `out` as the cluster manifolds for `manifolds`, carrying warm-start data from
/// `prev` (the clusters solved at the previous step). Buffers in `out` are reused.
pub(crate) fn cluster_manifolds_for_solver(
    manifolds: &[ContactManifold],
    prev: &[ContactManifold],
    out: &mut Vec<ContactManifold>,
    prediction_distance: Real,
) {
    let dedup_eps = prediction_distance * 0.25;
    let dedup_eps_sq = dedup_eps * dedup_eps;

    let mut num_out = 0;

    for manifold in manifolds {
        if manifold.points.is_empty() {
            continue;
        }

        let n1 = manifold_normal1(manifold);
        let cluster_id = out[..num_out]
            .iter()
            .position(|c| c.local_n1.dot(n1) >= COS_MERGE_ANGLE);

        let cluster_id = match cluster_id {
            Some(id) => id,
            None => {
                if num_out == out.len() {
                    out.push(ContactManifold::new());
                }

                let cluster = &mut out[num_out];
                cluster.points.clear();
                cluster.local_n1 = n1;
                cluster.local_n2 = manifold
                    .subshape_pos2()
                    .map(|p| p.rotation * manifold.local_n2)
                    .unwrap_or(manifold.local_n2);
                cluster.subshape1 = manifold.subshape1;
                cluster.subshape2 = manifold.subshape2;
                cluster.set_subshape_pos1(None);
                cluster.set_subshape_pos2(None);
                // Reset the solver data but keep the allocated solver_contacts buffer.
                let solver_contacts = core::mem::take(&mut cluster.data.solver_contacts);
                cluster.data = ContactManifoldData::default();
                cluster.data.solver_contacts = solver_contacts;
                cluster.data.solver_contacts.clear();

                num_out += 1;
                num_out - 1
            }
        };

        let cluster = &mut out[cluster_id];

        for pt in &manifold.points {
            let mut pt = *pt;
            if let Some(pos1) = manifold.subshape_pos1() {
                pt.local_p1 = *pos1 * pt.local_p1;
            }
            if let Some(pos2) = manifold.subshape_pos2() {
                pt.local_p2 = *pos2 * pt.local_p2;
            }
            // The warm-start data is carried from `prev` below, not from the
            // per-subshape manifolds (the solver never writes those back).
            pt.data = Default::default();

            // Deduplicate points that are nearly coincident (e.g. generated on both
            // sides of a shared triangle edge): keep the deepest one.
            if let Some(existing) = cluster
                .points
                .iter_mut()
                .find(|c| (c.local_p1 - pt.local_p1).length_squared() < dedup_eps_sq)
            {
                if pt.dist < existing.dist {
                    *existing = pt;
                }
            } else if cluster.points.len() < MAX_CLUSTER_POINTS {
                cluster.points.push(pt);
            } else if let Some(shallowest) = cluster.points.iter_mut().max_by(|a, b| {
                a.dist
                    .partial_cmp(&b.dist)
                    .unwrap_or(core::cmp::Ordering::Equal)
            }) {
                if pt.dist < shallowest.dist {
                    *shallowest = pt;
                }
            }
        }
    }

    out.truncate(num_out);

    carry_warmstart_data(prev, out, prediction_distance);
}

/// Copies warm-start data from `prev` manifold points into the best-matching points of
/// `targets` (nearest-position in the first shape's local space). Each previous point is
/// consumed at most once, so impulses are never duplicated.
pub(crate) fn carry_warmstart_data(
    prev: &[ContactManifold],
    targets: &mut [ContactManifold],
    prediction_distance: Real,
) {
    let match_eps_sq = prediction_distance * prediction_distance;

    for prev_manifold in prev {
        for prev_pt in &prev_manifold.points {
            if !has_warmstart_data(&prev_pt.data) {
                continue;
            }

            let mut best: Option<(usize, usize)> = None;
            let mut best_dist_sq = match_eps_sq;

            for (target_id, target) in targets.iter().enumerate() {
                if target.local_n1.dot(prev_manifold.local_n1) < COS_MERGE_ANGLE {
                    continue;
                }

                for (pt_id, pt) in target.points.iter().enumerate() {
                    if has_warmstart_data(&pt.data) {
                        // Already claimed by a previous point.
                        continue;
                    }

                    let p1 = target
                        .subshape_pos1()
                        .map(|pos| pos * pt.local_p1)
                        .unwrap_or(pt.local_p1);
                    let dist_sq = (p1 - prev_pt.local_p1).length_squared();

                    if dist_sq < best_dist_sq {
                        best = Some((target_id, pt_id));
                        best_dist_sq = dist_sq;
                    }
                }
            }

            if let Some((target_id, pt_id)) = best {
                targets[target_id].points[pt_id].data = prev_pt.data;
            }
        }
    }
}
