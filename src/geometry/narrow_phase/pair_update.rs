//! The per-pair contact-update kernel shared by `compute_contacts`' serial and
//! parallel dispatch paths: contact recycling, pair filtering, manifold
//! computation, solver-contact generation, and begin/end-touch bookkeeping.

use super::{
    PAIR_HINT_COUNT_MASK, PAIR_HINT_DYN_BIT, clear_filtered_pair, pair_qualified_manifold_count,
    single_manifold_bucket_drift,
};
#[cfg(not(feature = "parallel"))]
use crate::alloc_prelude::*;
use crate::dynamics::{
    CoefficientCombineRule, ImpulseJointSet, MultibodyJointSet, RigidBodyDominance, RigidBodySet,
    RigidBodyType,
};
use crate::geometry::{
    BoundingVolume, ColliderChanges, ColliderSet, ContactData, ContactManifoldData, ContactPair,
    SolverContact, SolverFlags,
};
use crate::math::{MAX_MANIFOLD_POINTS, Real};
use crate::pipeline::{ActiveHooks, ContactModificationContext, PairFilterContext, PhysicsHooks};
use parry::query::PersistentQueryDispatcher;
use parry::utils::PoseOpt;

/// Raw pointer to the per-pair solver hints, shared across parallel update workers.
/// Safety: only sound if each thread accesses a disjoint set of hint slots.
pub(super) struct HintsPtr(pub(super) *mut u16);
unsafe impl Sync for HintsPtr {}

/// A begin/end-touch transition detected by [`process_pair`]:
/// `(edge id, parent 1, parent 2, has_any_active_contact)`.
///
/// Everything a transition triggers (event emission, wake-ups, solver
/// coloring, island updates) mutates state shared across pairs, so both
/// dispatch paths defer it to a post-loop pass applied in sorted edge-id
/// order — the result is independent of the update schedule (and identical
/// between the serial and parallel builds).
pub(super) type PairTransition = (
    u32,
    Option<crate::dynamics::RigidBodyHandle>,
    Option<crate::dynamics::RigidBodyHandle>,
    bool,
);

// Outcome tags returned by `process_pair` (statistics only).
pub(super) const OUTCOME_SKIPPED: u8 = 0;
pub(super) const OUTCOME_RECYCLED: u8 = 1;
pub(super) const OUTCOME_FULL: u8 = 2;
// A recycled pair whose solver hint was repaired from count-cleared back to
// selectable (a sleeping side woke): it re-enters the selection without a
// full update, so the solver contact graph must reconcile it too.
pub(super) const OUTCOME_RECYCLED_REQUALIFIED: u8 = 3;
// A full update whose solver-graph bucket membership provably didn't
// change: counted as a full update, but not reconciled.
pub(super) const OUTCOME_FULL_CLEAN: u8 = 4;
// Full update of a composite pair (persistent workspace: heightfield/trimesh/
// voxels/compound). Manifold ordinals are NOT stable across such updates (BVH-order
// rebuild, dropped subshapes lose `graph_pos`), so force a full solver-graph rebuild.
pub(super) const OUTCOME_FULL_COMPOSITE: u8 = 5;
// Pair cleared by a filter early-out while it still had manifolds in the solver
// graph: `ContactPair::clear` destroys the `graph_pos` back-references the
// incremental reconcile needs, so the graph must be rebuilt from scratch (rare).
pub(super) const OUTCOME_CLEARED_IN_GRAPH: u8 = 6;

/// The per-pair contact update shared by `NarrowPhase::compute_contacts`'
/// single-threaded and parallel dispatch paths; returns an `OUTCOME_*` tag.
#[allow(clippy::too_many_arguments)]
pub(super) fn process_pair(
    edge: &mut crate::data::graph::Edge<ContactPair>,
    edge_id: u32,
    prediction_distance: Real,
    dt: Real,
    contact_clustering: bool,
    // Contact-recycling drift threshold; `0.0` disables recycling.
    contact_recycle_distance: Real,
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    impulse_joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    hooks: &dyn PhysicsHooks,
    query_dispatcher: &dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>,
    awake_body_mask: &[bool],
    hints_ptr: &HintsPtr,
    #[cfg(not(feature = "parallel"))] transitions: &mut Vec<PairTransition>,
    #[cfg(feature = "parallel")] snd: &std::sync::mpsc::Sender<PairTransition>,
) -> u8 {
    let pair = &mut edge.weight;
    let co1 = &colliders[pair.collider1];
    let co2 = &colliders[pair.collider2];

    let body_awake = |co: &crate::geometry::Collider| {
        co.parent.as_ref().is_some_and(|p| {
            awake_body_mask
                .get(p.handle.into_raw_parts().0 as usize)
                .copied()
                .unwrap_or(false)
        })
    };
    if !co1.changes.needs_narrow_phase_update()
        && !co2.changes.needs_narrow_phase_update()
        && !body_awake(co1)
        && !body_awake(co2)
    {
        // Neither collider was changed by the user nor possibly moved by the
        // simulation (its parent body is asleep or fixed).
        return OUTCOME_SKIPPED;
    }

    // Contact recycling: if the relative pose barely moved since the
    // last full update, skip contact determination entirely. Must run before the
    // `has_any_active_contact` walk below (whose result recycling cannot change).
    if contact_recycle_distance > 0.0 {
        if let Some(state) = &pair.recycle_state {
            // Anything beyond a position change (shape, groups, type,
            // enabled flag, ...) requires a full update, as do pairs
            // relying on per-step user hooks.
            let recycle_safe = ColliderChanges::IN_MODIFIED_SET
                | ColliderChanges::POSITION
                | ColliderChanges::LOCAL_MASS_PROPERTIES;
            let hooks_involved = !(co1.flags.active_hooks | co2.flags.active_hooks).is_empty();

            if ((co1.changes | co2.changes) & !recycle_safe).is_empty() && !hooks_involved {
                let pos12 = co1.pos.inv_mul(&co2.pos);
                // Conservative bound on how far any contact point moved in
                // the pair's local space since the last full update (chord
                // form, no `atan2`; see `relative_pose_drift`).
                let drift = crate::geometry::contact_pair::relative_pose_drift(
                    &state.pos12,
                    &pos12,
                    state.max_extent,
                );
                // The solver arms and normal are frozen in world space while
                // recycled, so each body's *absolute* rotation since the
                // freeze must stay small too (bound: cos Δθ > 0.98).
                let rot_cos =
                    crate::geometry::contact_pair::relative_rot_cos(&state.rot1, &co1.pos.rotation)
                        .min(crate::geometry::contact_pair::relative_rot_cos(
                            &state.rot2,
                            &co2.pos.rotation,
                        ));

                if drift <= state.max_drift && rot_cos > 0.98 {
                    // Recycling can't change the qualified-manifold count: only recompute
                    // a count-cleared hint (pair slept, or state was deserialized).
                    // SAFETY: each pair is processed at most once per update (disjoint slots).
                    let mut requalified = false;
                    let hint = unsafe { &mut *hints_ptr.0.add(edge_id as usize) };
                    if *hint & PAIR_HINT_COUNT_MASK == 0 {
                        let dyn_awake = |co: &crate::geometry::Collider| {
                            co.parent.is_some_and(|p| {
                                let rb = &bodies[p.handle];
                                rb.body_type.is_dynamic() && !rb.activation.sleeping
                            })
                        };
                        let is_dyn = dyn_awake(co1) || dyn_awake(co2);
                        *hint = pair_qualified_manifold_count(pair)
                            | ((is_dyn as u16) * PAIR_HINT_DYN_BIT);
                        // The pair re-entered the selection: the solver graph
                        // must reconcile it even though this is a recycle.
                        requalified =
                            *hint & PAIR_HINT_DYN_BIT != 0 && *hint & PAIR_HINT_COUNT_MASK != 0;
                    }

                    return if requalified {
                        OUTCOME_RECYCLED_REQUALIFIED
                    } else {
                        OUTCOME_RECYCLED
                    };
                }
            }
        }
    }

    let had_any_active_contact = pair.has_any_active_contact();
    let rb_handle1 = co1.parent.map(|p| p.handle);
    let rb_handle2 = co2.parent.map(|p| p.handle);
    let mut outcome = OUTCOME_SKIPPED;

    'emit_events: {
        if rb_handle1 == rb_handle2 && co1.parent.is_some() {
            // Same parents. Ignore collisions.
            if clear_filtered_pair(pair) {
                outcome = OUTCOME_CLEARED_IN_GRAPH;
            }
            break 'emit_events;
        }

        let rb1 = co1.parent.map(|co_parent1| &bodies[co_parent1.handle]);
        let rb2 = co2.parent.map(|co_parent2| &bodies[co_parent2.handle]);

        let rb_type1 = rb1.map(|rb| rb.body_type).unwrap_or(RigidBodyType::Fixed);
        let rb_type2 = rb2.map(|rb| rb.body_type).unwrap_or(RigidBodyType::Fixed);

        // Deal with contacts disabled between bodies attached by joints.
        if let (Some(co_parent1), Some(co_parent2)) = (&co1.parent, &co2.parent) {
            for (_, joint) in impulse_joints.joints_between(co_parent1.handle, co_parent2.handle) {
                if !joint.data.contacts_enabled {
                    if clear_filtered_pair(pair) {
                        outcome = OUTCOME_CLEARED_IN_GRAPH;
                    }
                    break 'emit_events;
                }
            }

            let link1 = multibody_joints.rigid_body_link(co_parent1.handle);
            let link2 = multibody_joints.rigid_body_link(co_parent2.handle);

            if let (Some(link1), Some(link2)) = (link1, link2) {
                // If both bodies belong to the same multibody, apply some additional built-in
                // contact filtering rules.
                if link1.multibody == link2.multibody {
                    // 1) check if self-contacts is enabled.
                    if let Some(mb) = multibody_joints.get_multibody(link1.multibody) {
                        if !mb.self_contacts_enabled() {
                            if clear_filtered_pair(pair) {
                                outcome = OUTCOME_CLEARED_IN_GRAPH;
                            }
                            break 'emit_events;
                        }
                    }

                    // 2) if they are attached by a joint, check if  contacts is disabled.
                    if let Some((_, _, mb_link)) =
                        multibody_joints.joint_between(co_parent1.handle, co_parent2.handle)
                    {
                        if !mb_link.joint.data.contacts_enabled {
                            if clear_filtered_pair(pair) {
                                outcome = OUTCOME_CLEARED_IN_GRAPH;
                            }
                            break 'emit_events;
                        }
                    }
                }
            }
        }

        // Filter based on the rigid-body types.
        if !co1.flags.active_collision_types.test(rb_type1, rb_type2)
            && !co2.flags.active_collision_types.test(rb_type1, rb_type2)
        {
            if clear_filtered_pair(pair) {
                outcome = OUTCOME_CLEARED_IN_GRAPH;
            }
            break 'emit_events;
        }

        // Filter based on collision groups.
        if !co1.flags.collision_groups.test(co2.flags.collision_groups) {
            if clear_filtered_pair(pair) {
                outcome = OUTCOME_CLEARED_IN_GRAPH;
            }
            break 'emit_events;
        }

        let active_hooks = co1.flags.active_hooks | co2.flags.active_hooks;

        let mut solver_flags = if active_hooks.contains(ActiveHooks::FILTER_CONTACT_PAIRS) {
            let context = PairFilterContext {
                bodies,
                colliders,
                rigid_body1: rb_handle1,
                rigid_body2: rb_handle2,
                collider1: pair.collider1,
                collider2: pair.collider2,
            };

            if let Some(solver_flags) = hooks.filter_contact_pair(&context) {
                solver_flags
            } else {
                // No contact allowed.
                if clear_filtered_pair(pair) {
                    outcome = OUTCOME_CLEARED_IN_GRAPH;
                }
                break 'emit_events;
            }
        } else {
            SolverFlags::default()
        };

        if !co1.flags.solver_groups.test(co2.flags.solver_groups) {
            solver_flags.remove(SolverFlags::COMPUTE_IMPULSES);
        }

        if co1.changes.contains(ColliderChanges::SHAPE)
            || co2.changes.contains(ColliderChanges::SHAPE)
        {
            // The shape changed so the workspace is no longer valid.
            pair.workspace = None;
        }

        let pos12 = co1.pos.inv_mul(&co2.pos);

        let contact_skin_sum = co1.contact_skin() + co2.contact_skin();
        let soft_ccd_prediction1 = rb1.map(|rb| rb.soft_ccd_prediction()).unwrap_or(0.0);
        let soft_ccd_prediction2 = rb2.map(|rb| rb.soft_ccd_prediction()).unwrap_or(0.0);
        let effective_prediction_distance = if soft_ccd_prediction1 > 0.0
            || soft_ccd_prediction2 > 0.0
        {
            let aabb1 = co1.compute_collision_aabb(0.0);
            let aabb2 = co2.compute_collision_aabb(0.0);
            let inv_dt = crate::utils::inv(dt);

            let linvel1 = rb1
                .map(|rb| rb.linvel().clamp_length_max(soft_ccd_prediction1 * inv_dt))
                .unwrap_or_default();
            let linvel2 = rb2
                .map(|rb| rb.linvel().clamp_length_max(soft_ccd_prediction2 * inv_dt))
                .unwrap_or_default();

            if !aabb1.intersects(&aabb2) && !aabb1.intersects_moving_aabb(&aabb2, linvel2 - linvel1)
            {
                if clear_filtered_pair(pair) {
                    outcome = OUTCOME_CLEARED_IN_GRAPH;
                }
                break 'emit_events;
            }

            prediction_distance.max(dt * (linvel1 - linvel2).length()) + contact_skin_sum
        } else {
            prediction_distance + contact_skin_sum
        };

        outcome = OUTCOME_FULL;
        let _ = query_dispatcher.contact_manifolds(
            &pos12,
            &*co1.shape,
            &*co2.shape,
            effective_prediction_distance,
            &mut pair.manifolds,
            &mut pair.workspace,
        );

        let friction = CoefficientCombineRule::combine(
            co1.material.friction,
            co2.material.friction,
            co1.material.friction_combine_rule,
            co2.material.friction_combine_rule,
        );
        let restitution = CoefficientCombineRule::combine(
            co1.material.restitution,
            co2.material.restitution,
            co1.material.restitution_combine_rule,
            co2.material.restitution_combine_rule,
        );

        let zero = RigidBodyDominance(0); // The value doesn't matter, it will be MAX because of the effective groups.
        let dominance1 = rb1.map(|rb| rb.dominance).unwrap_or(zero);
        let dominance2 = rb2.map(|rb| rb.dominance).unwrap_or(zero);

        #[cfg(feature = "dim3")]
        let use_clusters = contact_clustering && pair.manifolds.len() > 1;
        #[cfg(not(feature = "dim3"))]
        let use_clusters = {
            // Contact clustering isn’t implemented in 2D (manifolds hold at
            // most two points there, so there is little to merge).
            let _ = contact_clustering;
            false
        };

        #[cfg(feature = "dim3")]
        if use_clusters {
            // Rebuild the solver clusters, using the clusters solved at the
            // previous step as the warm-start source.
            core::mem::swap(&mut pair.solver_clusters, &mut pair.solver_clusters_prev);
            crate::geometry::contact_clustering::cluster_manifolds_for_solver(
                &pair.manifolds,
                &pair.solver_clusters_prev,
                &mut pair.solver_clusters,
                prediction_distance,
            );

            // The plain manifolds won't be seen by the solver, but keep their
            // user-facing data coherent.
            for manifold in &mut pair.manifolds {
                let world_pos1 = manifold.subshape_pos1().prepend_to(&co1.pos);
                manifold.data.solver_contacts.clear();
                manifold.data.rigid_body1 = rb_handle1;
                manifold.data.rigid_body2 = rb_handle2;
                manifold.data.solver_flags = solver_flags;
                manifold.data.friction = friction;
                manifold.data.restitution = restitution;
                manifold.data.relative_dominance =
                    dominance1.effective_group(&rb_type1) - dominance2.effective_group(&rb_type2);
                manifold.data.normal = world_pos1.rotation * manifold.local_n1;
            }
        } else if !pair.solver_clusters.is_empty() {
            // Clustering stopped applying to this pair: carry the warm-start
            // data back into the plain manifolds once, then drop the clusters.
            crate::geometry::contact_clustering::carry_warmstart_data(
                &pair.solver_clusters,
                &mut pair.manifolds,
                prediction_distance,
            );
            pair.solver_clusters.clear();
            pair.solver_clusters_prev.clear();
        }

        let solver_manifolds = if use_clusters {
            &mut pair.solver_clusters
        } else {
            &mut pair.manifolds
        };

        for manifold in solver_manifolds {
            let world_pos1 = manifold.subshape_pos1().prepend_to(&co1.pos);
            let world_pos2 = manifold.subshape_pos2().prepend_to(&co2.pos);
            manifold.data.solver_contacts.clear();
            manifold.data.rigid_body1 = rb_handle1;
            manifold.data.rigid_body2 = rb_handle2;
            manifold.data.solver_flags = solver_flags;
            manifold.data.friction = friction;
            manifold.data.restitution = restitution;
            manifold.data.relative_dominance =
                dominance1.effective_group(&rb_type1) - dominance2.effective_group(&rb_type2);
            manifold.data.normal = world_pos1.rotation * manifold.local_n1;

            // Generate solver contacts.
            #[allow(unused_mut)] // Mut not needed in 2D.
            let mut selected = [0, 1, 2, 3];
            #[allow(unused_mut)] // Mut not needed in 2D.
            let mut num_selected = MAX_MANIFOLD_POINTS.min(manifold.points.len());

            #[cfg(feature = "dim3")]
            crate::geometry::manifold_reduction::reduce_manifold_naive(
                manifold,
                &mut selected,
                &mut num_selected,
                prediction_distance,
            );

            // Sort points lexicographically on the contact plane (in `local_n1`'s orthonormal-
            // basis frame): picked empirically for pyramid stability over the alternatives.
            #[cfg(feature = "dim3")]
            if num_selected > 1 {
                use crate::utils::OrthonormalBasis;
                let basis = manifold.local_n1.orthonormal_basis();
                let mut keyed: [(Real, Real, usize); MAX_MANIFOLD_POINTS] =
                    [(0.0, 0.0, 0); MAX_MANIFOLD_POINTS];
                for (i, sel) in selected[..num_selected].iter().enumerate() {
                    let p = manifold.points[*sel].local_p1;
                    keyed[i] = (p.dot(basis[0]), p.dot(basis[1]), *sel);
                }
                // Manual insertion sort: much faster than `sort_unstable_by` for 4 elements.
                for i in 1..num_selected {
                    let k = keyed[i];
                    let mut j = i;
                    while j > 0
                        && (keyed[j - 1].0 > k.0 || (keyed[j - 1].0 == k.0 && keyed[j - 1].1 > k.1))
                    {
                        keyed[j] = keyed[j - 1];
                        j -= 1;
                    }
                    keyed[j] = k;
                }
                for (i, k) in keyed[..num_selected].iter().enumerate() {
                    selected[i] = k.2;
                }
            }

            for contact_id in &selected[..num_selected] {
                //     // manifold.points.iter().enumerate() {
                let contact = &manifold.points[*contact_id];
                let effective_contact_dist = contact.dist - co1.contact_skin() - co2.contact_skin();

                let keep_solver_contact = effective_contact_dist < prediction_distance || {
                    let world_pt1 = world_pos1 * contact.local_p1;
                    let world_pt2 = world_pos2 * contact.local_p2;
                    let vel1 = rb1
                        .map(|rb| rb.velocity_at_point(world_pt1))
                        .unwrap_or_default();
                    let vel2 = rb2
                        .map(|rb| rb.velocity_at_point(world_pt2))
                        .unwrap_or_default();
                    effective_contact_dist + (vel2 - vel1).dot(manifold.data.normal) * dt
                        < prediction_distance
                };

                if keep_solver_contact {
                    // The anchors hold world-space points until the localization pass
                    // below, so the contact-modification hook sees fresh world data.
                    let world_pt1 = world_pos1 * contact.local_p1;
                    let world_pt2 = world_pos2 * contact.local_p2;

                    let is_new =
                        (contact.data.impulse == 0.0) as crate::geometry::contact_pair::ContactId;
                    let solver_contact = SolverContact {
                        contact_id: [*contact_id as crate::geometry::contact_pair::ContactId
                            | (is_new * crate::geometry::contact_pair::NEW_CONTACT_BIT)],
                        anchor1: world_pt1,
                        anchor2: world_pt2,
                        dist: effective_contact_dist,
                        tangent_velocity: Default::default(),
                        #[cfg(feature = "dim3")]
                        padding: Default::default(),
                    };

                    manifold.data.solver_contacts.push(solver_contact);
                }
            }

            // Apply the user-defined contact modification.
            if active_hooks.contains(ActiveHooks::MODIFY_SOLVER_CONTACTS) {
                let mut modifiable_solver_contacts =
                    core::mem::take(&mut manifold.data.solver_contacts);
                let mut modifiable_user_data = manifold.data.user_data;
                let mut modifiable_normal = manifold.data.normal;
                let mut modifiable_friction = manifold.data.friction;
                let mut modifiable_restitution = manifold.data.restitution;

                let mut context = ContactModificationContext {
                    bodies,
                    colliders,
                    rigid_body1: rb_handle1,
                    rigid_body2: rb_handle2,
                    collider1: pair.collider1,
                    collider2: pair.collider2,
                    manifold,
                    solver_contacts: &mut modifiable_solver_contacts,
                    normal: &mut modifiable_normal,
                    friction: &mut modifiable_friction,
                    restitution: &mut modifiable_restitution,
                    user_data: &mut modifiable_user_data,
                };

                hooks.modify_solver_contacts(&mut context);

                manifold.data.solver_contacts = modifiable_solver_contacts;
                manifold.data.normal = modifiable_normal;
                manifold.data.friction = modifiable_friction;
                manifold.data.restitution = modifiable_restitution;
                manifold.data.user_data = modifiable_user_data;
            }

            // Localize solver contacts: bake skins (and hook-written `dist`) into the anchors, then
            // express each in its body's CoM frame (world-attached/dominance-superior sides keep world
            // anchors, matching the solver's identity pose). Riding rigidly lets recycled steps skip refresh.
            {
                let normal = manifold.data.normal;
                let rel_dom = manifold.data.relative_dominance;
                let com_pose = |rb: &&crate::dynamics::RigidBody| {
                    rb.pos
                        .position
                        .prepend_translation(rb.mprops.local_mprops.local_com)
                };
                let com_pose1 = rb1.as_ref().filter(|_| rel_dom <= 0).map(com_pose);
                let com_pose2 = rb2.as_ref().filter(|_| rel_dom >= 0).map(com_pose);
                // Split-borrow: the frozen solver arms are written to the
                // manifold points while iterating the solver contacts.
                let manifold_points = &mut manifold.points;
                for sc in &mut manifold.data.solver_contacts {
                    let shift = (sc.anchor2 - sc.anchor1).dot(normal) - sc.dist;
                    let p1 = sc.anchor1 + normal * shift;

                    // Freeze the solver's lever arms (world-space, CoM-relative; plain
                    // world for a world-attached side) at this full update. They stay
                    // verbatim while recycled: anchor freezing (see `ContactData::solver_dp1`).
                    let point = (p1 + sc.anchor2) * 0.5;
                    let cid = (sc.contact_id[0] & !crate::geometry::contact_pair::NEW_CONTACT_BIT)
                        as usize;
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
        }

        // Remember the relative configuration this full update ran at, so
        // subsequent steps can recycle the pair while it stays close to it.
        if contact_recycle_distance > 0.0 {
            // Computing the local AABBs goes through a dyn Shape call per
            // collider per full update; the extents only change when a
            // shape does, so reuse the previous full update's value.
            let shapes_changed =
                (co1.changes | co2.changes).contains(crate::geometry::ColliderChanges::SHAPE);
            let max_extent = match &pair.recycle_state {
                Some(state) if !shapes_changed => state.max_extent,
                _ => {
                    let origin_radius = |co: &crate::geometry::Collider| {
                        let aabb = co.shape.compute_local_aabb();
                        aabb.mins.length().max(aabb.maxs.length())
                    };
                    origin_radius(co1).max(origin_radius(co2))
                }
            };
            // A pair without contacts has an (unknown) separation larger than
            // the prediction distance. Cap its recycle window by the
            // prediction distance so an incoming contact can't be missed.
            let max_drift = if pair.has_any_active_contact() {
                contact_recycle_distance
            } else {
                contact_recycle_distance.min(prediction_distance)
            };
            pair.recycle_state = Some(crate::geometry::ContactRecycleState {
                pos12,
                rot1: co1.pos.rotation,
                rot2: co2.pos.rotation,
                max_extent,
                max_drift,
            });
        }
    }

    /*
     * Handle actions on contact start/stop: record the transition for the
     * deferred post-loop pass (see `PairTransition` — event emission, wake-ups,
     * coloring and island updates all mutate shared state, and deferring keeps
     * the result independent of the update schedule).
     */
    let has_any_active_contact = pair.has_any_active_contact();
    if has_any_active_contact != had_any_active_contact {
        let transition = (edge_id, rb_handle1, rb_handle2, has_any_active_contact);
        #[cfg(not(feature = "parallel"))]
        transitions.push(transition);
        #[cfg(feature = "parallel")]
        let _ = snd.send(transition);
    }

    // Refresh the pair's solver-qualification hint from its final state
    // (this point is reached by every path that may have changed the
    // manifolds: full updates and the various pair-clearing branches).
    let mut membership_changed = true;
    {
        let dyn_awake = |co: &crate::geometry::Collider| {
            co.parent.is_some_and(|p| {
                let rb = &bodies[p.handle];
                rb.body_type.is_dynamic() && !rb.activation.sleeping
            })
        };
        let is_dyn = dyn_awake(co1) || dyn_awake(co2);
        let new_hint = pair_qualified_manifold_count(pair) | ((is_dyn as u16) * PAIR_HINT_DYN_BIT);
        // SAFETY: each pair is processed at most once per update, so the
        //         hint slots accessed are disjoint.
        let old_hint = unsafe { *hints_ptr.0.add(edge_id as usize) };
        unsafe {
            *hints_ptr.0.add(edge_id as usize) = new_hint;
        }

        // Event-driven graph maintenance: reconcile only when bucket
        // membership changed — hint flip or (color, active-count) drift — so routine
        // updates cost nothing. Only exact for single-manifold pairs (stable ordinals).
        if outcome == OUTCOME_FULL && old_hint == new_hint {
            let selectable =
                new_hint & PAIR_HINT_DYN_BIT != 0 && new_hint & PAIR_HINT_COUNT_MASK != 0;
            membership_changed = match pair.solver_manifolds().len() {
                // No solver manifolds: nothing can be in the graph (a
                // manifold entering/leaving the graph flips the hint).
                0 => false,
                1 => single_manifold_bucket_drift(pair, selectable),
                // Multi-manifold/clustered lists are rebuilt by the
                // update (ordinals unstable): always reconcile while
                // selected.
                _ => selectable,
            };
        }
    }

    // Composite pairs have unstable manifold ordinals (see `OUTCOME_FULL_COMPOSITE`),
    // so signal a full rebuild. Must be checked before the `FULL_CLEAN` shortcut: a
    // surviving manifold can look "clean" while a dropped sibling leaked its graph slot.
    if outcome == OUTCOME_FULL && pair.workspace.is_some() {
        return OUTCOME_FULL_COMPOSITE;
    }
    if outcome == OUTCOME_FULL && !membership_changed {
        return OUTCOME_FULL_CLEAN;
    }
    outcome
}
