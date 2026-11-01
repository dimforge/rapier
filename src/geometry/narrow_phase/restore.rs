//! Post-restore replay of solver contacts.
//!
//! Snapshots store a replayable pair's solver contacts as one byte each (see
//! `contact_pair::pair_serde`). This pass recomputes them, with the frozen lever arms and the
//! manifold normals, by re-running the pair's last full update at its recorded collider poses.

use crate::alloc_prelude::*;
use crate::dynamics::RigidBodySet;
use crate::geometry::contact_pair::NEW_CONTACT_BIT;
use crate::geometry::{Collider, ColliderSet, ContactManifold, ContactPair};
use crate::math::Pose;
use parry::utils::PoseOpt;

impl super::NarrowPhase {
    /// Recomputes the solver contacts, manifold normals and frozen lever arms that snapshots
    /// store in compact form (a no-op when nothing is pending).
    ///
    /// World restores, delta applies and the first narrow-phase update run it; call it to read
    /// restored contacts before the first step. Pairs whose center of mass moved since their last
    /// update replay against the new one, differing from the original until the next step.
    pub fn rebuild_solver_contacts(&mut self, bodies: &RigidBodySet, colliders: &ColliderSet) {
        use crate::data::delta::StaleSlots;
        let edges = &mut self.contact_graph.graph.edges;
        match core::mem::take(&mut self.pending_solver_replay) {
            StaleSlots::None => {}
            StaleSlots::All => {
                for edge in edges.iter_mut() {
                    replay_pair(&mut edge.weight, bodies, colliders);
                }
            }
            StaleSlots::Some(patched) => {
                for i in patched {
                    if let Some(edge) = edges.get_mut(i as usize) {
                        replay_pair(&mut edge.weight, bodies, colliders);
                    }
                }
            }
        }
    }
}

fn replay_pair(pair: &mut ContactPair, bodies: &RigidBodySet, colliders: &ColliderSet) {
    let (handle1, handle2) = (pair.collider1, pair.collider2);
    let Some(rigid) = pair.rigid_mut() else {
        return;
    };
    let (Some(co1), Some(co2)) = (colliders.get(handle1), colliders.get(handle2)) else {
        return;
    };
    let Some(rs) = rigid.recycle_state else {
        return;
    };
    // Only compact-encoded pairs carry the marker (taken only once the replay can run, so a
    // re-save keeps the compact encoding); it lists the points whose tangent is recomputed.
    let Some(pending) = rigid.pending_replay.take() else {
        return;
    };
    let recompute_tangent = &pending.recompute_tangent;

    // The exact world poses of both colliders at the pair's last full update.
    let pos1_t = Pose::from_parts(rs.replay.translation1, rs.rot1);
    let pos2_t = Pose::from_parts(rs.replay.translation2, rs.rot2);

    // When clustering applied, the clusters carried the solver contacts and the plain
    // manifolds hold none; only the plain manifolds' normals need replaying then.
    let clustered = !rigid.solver_clusters.is_empty();
    let skins = (rs.replay.skin1, rs.replay.skin2);

    let num_manifolds = rigid.manifolds.len();
    for (mi, manifold) in rigid.manifolds.iter_mut().enumerate() {
        let world_pos1 = manifold.subshape_pos1().prepend_to(&pos1_t);
        manifold.data.normal = world_pos1.rotation * manifold.local_n1;
        if !clustered {
            let world_pos2 = manifold.subshape_pos2().prepend_to(&pos2_t);
            replay_manifold_contacts(
                manifold,
                co1,
                co2,
                bodies,
                &world_pos1,
                &world_pos2,
                &pos1_t,
                &pos2_t,
                skins,
            );
        }
        replay_points_tangent_world(manifold, recompute_tangent.get(mi));
    }

    // Cluster manifolds hold no subshape poses (their local normals and points are
    // already collider-local), so their world poses are the collider poses.
    for (ci, cluster) in rigid.solver_clusters.iter_mut().enumerate() {
        cluster.data.normal = pos1_t.rotation * cluster.local_n1;
        replay_manifold_contacts(
            cluster, co1, co2, bodies, &pos1_t, &pos2_t, &pos1_t, &pos2_t, skins,
        );
        replay_points_tangent_world(cluster, recompute_tangent.get(num_manifolds + ci));
    }
}

/// Recomputes the marked points' `warmstart_tangent_world` from the replayed manifold
/// normal (the encoder stored the vector verbatim for every other point).
fn replay_points_tangent_world(manifold: &mut ContactManifold, marked: Option<&Vec<u32>>) {
    #[cfg(feature = "dim2")]
    let _ = (manifold, marked);
    #[cfg(feature = "dim3")]
    {
        let Some(marked) = marked else { return };
        if marked.is_empty() {
            return;
        }
        let (t1, t2) = crate::geometry::contact_pair::replay_tangent_basis(&manifold.data.normal);
        for &i in marked {
            if let Some(pt) = manifold.points.get_mut(i as usize) {
                pt.data.warmstart_tangent_world =
                    crate::geometry::contact_pair::replay_tangent_world(&t1, &t2, &pt.data);
            }
        }
    }
}

/// Regenerates the world anchors of the contacts selected at the last full update, then
/// localizes them with the same function as `pair_update::process_pair`.
#[allow(clippy::too_many_arguments)]
fn replay_manifold_contacts(
    manifold: &mut ContactManifold,
    co1: &Collider,
    co2: &Collider,
    bodies: &RigidBodySet,
    world_pos1: &Pose,
    world_pos2: &Pose,
    pos1_t: &Pose,
    pos2_t: &Pose,
    (skin1, skin2): (crate::math::Real, crate::math::Real),
) {
    let rel_dom = manifold.data.relative_dominance;
    // Body poses at the last full update equal the collider poses (identity
    // `pos_wrt_parent`, guaranteed by `ContactReplayState::replayable`).
    let com_pose = |co: &Collider, pose_t: &Pose| {
        co.parent
            .as_ref()
            .and_then(|p| bodies.get(p.handle))
            .map(|rb| super::pair_update::com_frame(pose_t, rb.mprops.local_mprops.local_com))
    };
    let com_pose1 = (rel_dom <= 0).then(|| com_pose(co1, pos1_t)).flatten();
    let com_pose2 = (rel_dom >= 0).then(|| com_pose(co2, pos2_t)).flatten();

    let manifold_points = &manifold.points;
    for sc in &mut manifold.data.solver_contacts {
        let cid = (sc.contact_id[0] & !NEW_CONTACT_BIT) as usize;
        let Some(contact) = manifold_points.get(cid) else {
            continue;
        };
        sc.anchor1 = *world_pos1 * contact.local_p1;
        sc.anchor2 = *world_pos2 * contact.local_p2;
        sc.dist = contact.dist - skin1 - skin2;
        sc.tangent_velocity = Default::default();
    }
    super::pair_update::localize_solver_contacts(manifold, com_pose1.as_ref(), com_pose2.as_ref());
}
