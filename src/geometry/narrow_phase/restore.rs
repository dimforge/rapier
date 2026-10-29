//! Post-restore replay of solver contacts.
//!
//! Snapshots serialize a replayable pair's solver contacts as one byte each (see
//! `contact_pair::pair_serde`); deserialization leaves stubs holding only the contact
//! id. This pass recomputes the anchors, distances, frozen lever arms, and manifold
//! normals exactly as the pair's last full narrow-phase update did, by re-running the
//! same expressions at the collider poses stored in the pair's recycle state. Pairs
//! whose replay would not be bit-exact (hook-modified, offset colliders, no recycle
//! state) were serialized verbatim and are left untouched.

use crate::dynamics::RigidBodySet;
use crate::geometry::contact_pair::{NEW_CONTACT_BIT, PairEventStatus};
use crate::geometry::{Collider, ColliderSet, ContactManifold, ContactPair};
use crate::math::Pose;
use parry::utils::PoseOpt;

impl super::NarrowPhase {
    /// Recomputes the solver-facing contact data (solver contacts, manifold normals,
    /// frozen lever arms) that snapshots store in compact form.
    ///
    /// The first narrow-phase update after a deserialization runs this automatically;
    /// call it explicitly only to read solver contacts from a freshly restored world
    /// before its first step.
    pub fn rebuild_solver_contacts(&mut self, bodies: &RigidBodySet, colliders: &ColliderSet) {
        self.pending_solver_replay = false;
        for edge in self.contact_graph.graph.edges.iter_mut() {
            replay_pair(&mut edge.weight, bodies, colliders);
        }
    }
}

fn replay_pair(pair: &mut ContactPair, bodies: &RigidBodySet, colliders: &ColliderSet) {
    if pair.event_status.contains(PairEventStatus::SOLVER_HOOKED)
        || !pair
            .event_status
            .contains(PairEventStatus::SOLVER_REPLAY_ANCHORS)
    {
        return;
    }
    let (Some(co1), Some(co2)) = (colliders.get(pair.collider1), colliders.get(pair.collider2))
    else {
        return;
    };
    let Some(rigid) = pair.rigid_mut() else {
        return;
    };
    let Some(rs) = &rigid.recycle_state else {
        return;
    };

    // The exact world poses of both colliders at the pair's last full update.
    let pos1_t = Pose::from_parts(rs.translation1, rs.rot1);
    let pos2_t = Pose::from_parts(rs.translation2, rs.rot2);

    // When clustering applied, the clusters carried the solver contacts and the plain
    // manifolds hold none; only the plain manifolds' normals need replaying then.
    let clustered = !rigid.solver_clusters.is_empty();
    let skins = (rs.skin1, rs.skin2);

    for manifold in &mut rigid.manifolds {
        let world_pos1 = manifold.subshape_pos1().prepend_to(&pos1_t);
        manifold.data.normal = world_pos1.rotation * manifold.local_n1;
        if !clustered {
            let world_pos2 = manifold.subshape_pos2().prepend_to(&pos2_t);
            replay_manifold_contacts(
                manifold, co1, co2, bodies, &world_pos1, &world_pos2, &pos1_t, &pos2_t, skins,
            );
        }
    }

    // Cluster manifolds hold no subshape poses (their local normals and points are
    // already collider-local), so their world poses are the collider poses.
    for cluster in &mut rigid.solver_clusters {
        cluster.data.normal = pos1_t.rotation * cluster.local_n1;
        replay_manifold_contacts(
            cluster, co1, co2, bodies, &pos1_t, &pos2_t, &pos1_t, &pos2_t, skins,
        );
    }
}

/// Mirrors the solver-contact generation and localization of
/// `pair_update::process_pair` for the contacts selected at the last full update,
/// expression for expression so the results are bit-identical.
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
    let normal = manifold.data.normal;
    let rel_dom = manifold.data.relative_dominance;

    // Body poses at the last full update equal the collider poses (identity
    // `pos_wrt_parent`, guaranteed by `SOLVER_REPLAY_ANCHORS`).
    let com_pose = |co: &Collider, pose_t: &Pose| {
        co.parent
            .as_ref()
            .and_then(|p| bodies.get(p.handle))
            .map(|rb| pose_t.prepend_translation(rb.mprops.local_mprops.local_com))
    };
    let com_pose1 = if rel_dom <= 0 {
        com_pose(co1, pos1_t)
    } else {
        None
    };
    let com_pose2 = if rel_dom >= 0 {
        com_pose(co2, pos2_t)
    } else {
        None
    };

    let manifold_points = &mut manifold.points;
    for sc in &mut manifold.data.solver_contacts {
        let cid = (sc.contact_id[0] & !NEW_CONTACT_BIT) as usize;
        let Some(contact) = manifold_points.get(cid) else {
            continue;
        };
        let effective_contact_dist = contact.dist - skin1 - skin2;
        let world_pt1 = *world_pos1 * contact.local_p1;
        let world_pt2 = *world_pos2 * contact.local_p2;
        sc.anchor1 = world_pt1;
        sc.anchor2 = world_pt2;
        sc.dist = effective_contact_dist;
        sc.tangent_velocity = Default::default();

        let shift = (sc.anchor2 - sc.anchor1).dot(normal) - sc.dist;
        let p1 = sc.anchor1 + normal * shift;

        let point = (p1 + sc.anchor2) * 0.5;
        let pt_data = &mut manifold_points[cid].data;
        pt_data.solver_dp1 = match &com_pose1 {
            Some(pose) => point - pose.translation,
            None => point,
        };
        pt_data.solver_dp2 = match &com_pose2 {
            Some(pose) => point - pose.translation,
            None => point,
        };

        sc.anchor1 = match &com_pose1 {
            Some(pose) => pose.inverse_transform_point(p1),
            None => p1,
        };
        if let Some(pose) = &com_pose2 {
            sc.anchor2 = pose.inverse_transform_point(sc.anchor2);
        }
    }
}
