use crate::dynamics::{
    ImpulseJointSet, IslandManager, JointEnabled, MultibodyJointSet, RigidBodyChanges,
    RigidBodyHandle, RigidBodySet, RigidBodyType,
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
    _multibody_joints: &mut MultibodyJointSet, // FIXME: propagate disabled state to multibodies
    modified_bodies: &[RigidBodyHandle],
    modified_colliders: &mut ModifiedColliders,
) {
    enum FinalAction {
        UpdateActiveKinematicSetId(usize),
        UpdateActiveDynamicSetId(usize),
        RemoveFromIsland,
    }

    for handle in modified_bodies {
        let mut final_action = None;

        if !bodies.contains(*handle) {
            // The body no longer exists.
            continue;
        }

        let rb = bodies.index_mut_internal(*handle);
        let mut ids = rb.ids;
        let changes = rb.changes;
        let activation = rb.activation;

        {
            if rb.is_enabled() {
                // The body's status changed. We need to make sure
                // it is on the correct active set.
                if let Some(islands) = islands.as_deref_mut() {
                    if changes.contains(RigidBodyChanges::TYPE) {
                        match rb.body_type {
                            RigidBodyType::Dynamic => {
                                // Remove from the active kinematic set if it was there.
                                if islands.active_kinematic_set.get(ids.active_set_id)
                                    == Some(handle)
                                {
                                    islands.active_kinematic_set.swap_remove(ids.active_set_id);
                                    final_action = Some(FinalAction::UpdateActiveKinematicSetId(
                                        ids.active_set_id,
                                    ));
                                }
                            }
                            RigidBodyType::KinematicVelocityBased
                            | RigidBodyType::KinematicPositionBased => {
                                // Remove from the active dynamic set if it was there.
                                if islands.active_dynamic_set.get(ids.active_set_id) == Some(handle)
                                {
                                    islands.active_dynamic_set.swap_remove(ids.active_set_id);
                                    final_action = Some(FinalAction::UpdateActiveDynamicSetId(
                                        ids.active_set_id,
                                    ));
                                }

                                // Add to the active kinematic set.
                                if islands.active_kinematic_set.get(ids.active_set_id)
                                    != Some(handle)
                                {
                                    ids.active_set_id = islands.active_kinematic_set.len();
                                    islands.active_kinematic_set.push(*handle);
                                }
                            }
                            RigidBodyType::Fixed => {}
                        }
                    }

                    // Update the active kinematic set.
                    if (changes.contains(RigidBodyChanges::POSITION)
                        || changes.contains(RigidBodyChanges::COLLIDERS))
                        && rb.is_kinematic()
                        && islands.active_kinematic_set.get(ids.active_set_id) != Some(handle)
                    {
                        ids.active_set_id = islands.active_kinematic_set.len();
                        islands.active_kinematic_set.push(*handle);
                    }

                    // Push the body to the active set if it is not inside the active set yet, and
                    // is either not longer sleeping or became dynamic.
                    if (changes.contains(RigidBodyChanges::SLEEP) || changes.contains(RigidBodyChanges::TYPE))
                        && rb.is_enabled()
                        && !rb.activation.sleeping // May happen if the body was put to sleep manually.
                        && rb.is_dynamic() // Only dynamic bodies are in the active dynamic set.
                        && islands.active_dynamic_set.get(ids.active_set_id) != Some(handle)
                    {
                        ids.active_set_id = islands.active_dynamic_set.len(); // This will handle the case where the activation_channel contains duplicates.
                        islands.active_dynamic_set.push(*handle);
                    }
                }
            }

            // Update the colliders' positions.
            if changes.contains(RigidBodyChanges::POSITION)
                || changes.contains(RigidBodyChanges::COLLIDERS)
            {
                rb.colliders
                    .update_positions(colliders, modified_colliders, &rb.pos.position);
            }

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
                impulse_joints.map_attached_joints_mut(*handle, |_, _, _, joint| {
                    if rb.enabled && joint.data.enabled == JointEnabled::DisabledByAttachedBody {
                        joint.data.enabled = JointEnabled::Enabled;
                    } else if !rb.enabled && joint.data.enabled == JointEnabled::Enabled {
                        joint.data.enabled = JointEnabled::DisabledByAttachedBody;
                    }
                });

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
                    &rb.pos.position,
                );
            }

            rb.ids = ids;
            rb.activation = activation;
        }

        // Adjust some ids, if needed.
        if let Some(islands) = islands.as_deref_mut() {
            if let Some(action) = final_action {
                match action {
                    FinalAction::RemoveFromIsland => {
                        let ids = rb.ids;
                        islands.rigid_body_removed(*handle, &ids, bodies);
                    }
                    FinalAction::UpdateActiveKinematicSetId(id) => {
                        let active_set = &mut islands.active_kinematic_set;
                        if id < active_set.len() {
                            bodies.index_mut_internal(active_set[id]).ids.active_set_id = id;
                        }
                    }
                    FinalAction::UpdateActiveDynamicSetId(id) => {
                        let active_set = &mut islands.active_dynamic_set;
                        if id < active_set.len() {
                            bodies.index_mut_internal(active_set[id]).ids.active_set_id = id;
                        }
                    }
                };
            }
        }
    }
}
