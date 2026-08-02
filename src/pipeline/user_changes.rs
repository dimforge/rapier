use crate::alloc_prelude::*;
use crate::dynamics::{
    ImpulseJointSet, IslandManager, JointEnabled, MultibodyJointSet, RigidBodyChanges,
    RigidBodyHandle, RigidBodySet,
};
use crate::geometry::{
    ColliderChanges, ColliderEnabled, ColliderHandle, ColliderPosition, ColliderSet,
    ModifiedColliders,
};

pub(crate) fn handle_user_changes_to_colliders(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    modified_colliders: &[ColliderHandle],
) {
    for handle in modified_colliders {
        // NOTE: we use `get` because the collider may no longer
        //       exist if it has been removed.
        if let Some(co) = colliders.get_mut_internal(*handle) {
            if co.changes.contains(ColliderChanges::PARENT) {
                if let Some(co_parent) = co.parent {
                    let parent_rb = &bodies[co_parent.handle];

                    co.pos = ColliderPosition(parent_rb.pos.position * co_parent.pos_wrt_parent);
                    co.changes |= ColliderChanges::POSITION;
                }
            }

            if co.changes.intersects(
                ColliderChanges::SHAPE
                    | ColliderChanges::LOCAL_MASS_PROPERTIES
                    | ColliderChanges::ENABLED_OR_DISABLED
                    | ColliderChanges::PARENT,
            ) {
                if let Some(rb) = co
                    .parent
                    .and_then(|p| bodies.get_mut_internal_with_modification_tracking(p.handle))
                {
                    rb.changes |= RigidBodyChanges::LOCAL_MASS_PROPERTIES;
                }
            }
        }
    }
}

pub(crate) fn handle_user_changes_to_rigid_bodies(
    mut islands: Option<&mut IslandManager>,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet, // FIXME: propagate disabled state to multibodies
    modified_bodies: &[RigidBodyHandle],
    modified_colliders: &mut ModifiedColliders,
) {
    enum FinalAction {
        RemoveFromIsland,
    }

    let mut any_jointed_body_modified = false;

    for handle in modified_bodies {
        let mut final_action = None;
        let type_changed;

        if !bodies.contains(*handle) {
            // The body no longer exists.
            continue;
        }

        // A modified body invalidates the solver's persistent joint assembly if
        // any impulse joint is attached to it (its type, pose, mass properties
        // or solver settings may be baked into the cached joint builders).
        any_jointed_body_modified =
            any_jointed_body_modified || impulse_joints.body_may_have_joints(*handle);

        {
            let rb = bodies.index_mut_internal(*handle);
            let changes = rb.changes;
            let activation = rb.activation;

            if rb.is_enabled() {
                // The body's status changed. We need to make sure
                // it is on the correct active set.
                if let Some(islands) = islands.as_deref_mut() {
                    islands.rigid_body_updated(*handle, bodies);
                }
            }

            let rb = bodies.index_mut_internal(*handle);

            // Update the colliders' positions.
            if changes.contains(RigidBodyChanges::POSITION)
                || changes.contains(RigidBodyChanges::COLLIDERS)
            {
                rb.colliders
                    .update_positions(colliders, modified_colliders, &rb.pos.position);

                // Refresh the world-space mass-properties. This is the only pre-solver
                // refresh for user-moved (or newly inserted) bodies: the regular
                // per-step refresh happens at the end of the step, right after pose
                // integration.
                rb.mprops
                    .update_world_mass_properties(rb.body_type, &rb.pos.position);
            }

            type_changed = changes.contains(RigidBodyChanges::TYPE);

            if changes.contains(RigidBodyChanges::DOMINANCE)
                || changes.contains(RigidBodyChanges::TYPE)
            {
                for handle in rb.colliders.0.iter() {
                    // NOTE: we can’t just use `colliders.get_mut_internal_with_modification_tracking`
                    // here because that would modify the `modified_colliders` inside of the `ColliderSet`
                    // instead of the one passed to this method.
                    let co = colliders.index_mut_internal(*handle);
                    modified_colliders.push_once(*handle, co);
                    co.changes |= ColliderChanges::PARENT_EFFECTIVE_DOMINANCE;
                }
            }

            if changes.contains(RigidBodyChanges::ENABLED_OR_DISABLED) {
                // Propagate the rigid-body’s enabled/disable status to its colliders.
                for handle in rb.colliders.0.iter() {
                    // NOTE: we can’t just use `colliders.get_mut_internal_with_modification_tracking`
                    // here because that would modify the `modified_colliders` inside of the `ColliderSet`
                    // instead of the one passed to this method.
                    let co = colliders.index_mut_internal(*handle);
                    modified_colliders.push_once(*handle, co);

                    if rb.enabled && co.flags.enabled == ColliderEnabled::DisabledByParent {
                        co.flags.enabled = ColliderEnabled::Enabled;
                    } else if !rb.enabled && co.flags.enabled == ColliderEnabled::Enabled {
                        co.flags.enabled = ColliderEnabled::DisabledByParent;
                    }

                    co.changes |= ColliderChanges::ENABLED_OR_DISABLED;
                }

                // Propagate the rigid-body’s enabled/disable status to its attached impulse joints.
                let mut joint_island_events = Vec::new();
                impulse_joints.map_attached_joints_mut(*handle, |rb1, rb2, joint_handle, joint| {
                    if rb.enabled && joint.data.enabled == JointEnabled::DisabledByAttachedBody {
                        joint.data.enabled = JointEnabled::Enabled;
                        joint_island_events.push(crate::dynamics::ImpulseJointIslandEvent::Link {
                            handle: joint_handle,
                            body1: rb1,
                            body2: rb2,
                        });
                    } else if !rb.enabled && joint.data.enabled == JointEnabled::Enabled {
                        joint.data.enabled = JointEnabled::DisabledByAttachedBody;
                        joint_island_events.push(
                            crate::dynamics::ImpulseJointIslandEvent::Unlink {
                                handle: joint_handle,
                            },
                        );
                    }
                });
                impulse_joints.island_events.extend(joint_island_events);

                // Persistent islands: a body toggling enabled/disabled changes
                // which bodies its multibody's connectivity chain spans.
                if let Some(link) = multibody_joints.rigid_body_link(*handle).copied() {
                    multibody_joints.island_chain_events.push(link.multibody);
                }

                // FIXME: Propagate the rigid-body’s enabled/disable status to its attached multibody joints.

                // Remove the rigid-body from the island manager.
                if !rb.enabled {
                    final_action = Some(FinalAction::RemoveFromIsland);
                }
            }

            // NOTE: recompute the mass-properties AFTER dealing with the rigid-body changes
            //       that imply a collider change (in particular, after propagation of the
            //       enabled/disabled status).
            if changes
                .intersects(RigidBodyChanges::LOCAL_MASS_PROPERTIES | RigidBodyChanges::COLLIDERS)
            {
                rb.mprops.recompute_mass_properties_from_colliders(
                    colliders,
                    &rb.colliders,
                    rb.body_type,
                    &rb.pos.position,
                );
            }

            rb.activation = activation;
        }

        // Adjust some ids, if needed.
        if let Some(islands) = islands.as_deref_mut() {
            if let Some(action) = final_action {
                match action {
                    FinalAction::RemoveFromIsland => {
                        let rb = bodies.index_mut_internal(*handle);
                        let ids = rb.ids;
                        islands.rigid_body_removed_or_disabled(*handle, &ids, bodies);
                    }
                };
            }

            if type_changed {
                // Persistent islands: a link recorded while an endpoint was
                // fixed doesn't connect (and vice versa), so a type change
                // must refresh every joint link of this body. (Contact links
                // are refreshed by the narrow-phase's modified-colliders pass;
                // the body's own island membership by `rigid_body_updated`.)
                let mut joint_island_events = Vec::new();
                impulse_joints.map_attached_joints_mut(*handle, |rb1, rb2, joint_handle, joint| {
                    joint_island_events.push(crate::dynamics::ImpulseJointIslandEvent::Unlink {
                        handle: joint_handle,
                    });
                    if joint.data.is_enabled() {
                        joint_island_events.push(crate::dynamics::ImpulseJointIslandEvent::Link {
                            handle: joint_handle,
                            body1: rb1,
                            body2: rb2,
                        });
                    }
                });
                impulse_joints.island_events.extend(joint_island_events);
                if let Some(link) = multibody_joints.rigid_body_link(*handle).copied() {
                    multibody_joints.island_chain_events.push(link.multibody);
                }
            }

            // A moved *fixed* body must wake its joint partners: fixed bodies
            // are not island members, so their own wake is a no-op, and only
            // *contact* partners get woken through the modified-colliders
            // path. (A moved dynamic/kinematic body wakes its whole island,
            // joint partners included.)
            let rb = &bodies[*handle];
            if rb.is_fixed() && rb.changes.contains(RigidBodyChanges::POSITION) {
                let mut to_wake = Vec::new();
                impulse_joints.map_attached_joints_mut(*handle, |rb1, rb2, _, _| {
                    to_wake.push(if rb1 == *handle { rb2 } else { rb1 });
                });
                for other in multibody_joints.bodies_attached_with_enabled_joint(*handle) {
                    to_wake.push(other);
                }
                for partner in to_wake {
                    islands.wake_up(bodies, partner, true);
                }
            }
        }
    }

    if any_jointed_body_modified {
        impulse_joints.bump_assembly_epoch();
    }
}
