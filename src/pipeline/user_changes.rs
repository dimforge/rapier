use crate::dynamics::{
    IslandManager, RigidBodyChanges, RigidBodyHandle, RigidBodySet, RigidBodyType,
};
use crate::geometry::{ColliderChanges, ColliderHandle, ColliderPosition, ColliderSet};
use parry::utils::hashmap::HashMap;

pub(crate) fn handle_user_changes_to_colliders(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    modified_colliders: &[ColliderHandle],
) {
    // TODO: avoid this hashmap? We could perhaps add a new flag to RigidBodyChanges to
    //       indicated that the mass properties need to be recomputed?
    let mut mprops_to_update = HashMap::default();

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

            if co.changes.contains(ColliderChanges::SHAPE) {
                if let Some(co_parent) = co.parent {
                    mprops_to_update.insert(co_parent.handle, ());
                }
            }
        }
    }

    for (to_update, _) in mprops_to_update {
        let rb = bodies.index_mut_internal(to_update);
        rb.mprops.recompute_mass_properties_from_colliders(
            colliders,
            &rb.colliders,
            &rb.pos.position,
        );
    }
}

pub(crate) fn handle_user_changes_to_rigid_bodies(
    mut islands: Option<&mut IslandManager>,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    modified_bodies: &[RigidBodyHandle],
    modified_colliders: &mut Vec<ColliderHandle>,
) {
    enum FinalAction {
        UpdateActiveKinematicSetId,
        UpdateActiveDynamicSetId,
    }

    for handle in modified_bodies {
        let mut final_action = None;

        if !bodies.contains(*handle) {
            // The body no longer exists.
            continue;
        }

        let rb = bodies.index_mut_internal(*handle);
        let mut changes = rb.changes;
        let mut ids = rb.ids;
        let mut activation = rb.activation;

        {
            // The body's status changed. We need to make sure
            // it is on the correct active set.
            if let Some(islands) = islands.as_deref_mut() {
                if changes.contains(RigidBodyChanges::TYPE) {
                    match rb.body_type {
                        RigidBodyType::Dynamic => {
                            // Remove from the active kinematic set if it was there.
                            if islands.active_kinematic_set.get(ids.active_set_id) == Some(handle) {
                                islands.active_kinematic_set.swap_remove(ids.active_set_id);
                                final_action = Some((
                                    FinalAction::UpdateActiveKinematicSetId,
                                    ids.active_set_id,
                                ));
                            }

                            // Add to the active dynamic set.
                            activation.wake_up(true);
                            // Make sure the sleep change flag is set (even if for some
                            // reasons the rigid-body was already awake) to make
                            // sure the code handling sleeping change adds the body to
                            // the active_dynamic_set.
                            changes.set(RigidBodyChanges::SLEEP, true);
                        }
                        RigidBodyType::KinematicVelocityBased
                        | RigidBodyType::KinematicPositionBased => {
                            // Remove from the active dynamic set if it was there.
                            if islands.active_dynamic_set.get(ids.active_set_id) == Some(handle) {
                                islands.active_dynamic_set.swap_remove(ids.active_set_id);
                                final_action = Some((
                                    FinalAction::UpdateActiveDynamicSetId,
                                    ids.active_set_id,
                                ));
                            }

                            // Add to the active kinematic set.
                            if islands.active_kinematic_set.get(ids.active_set_id) != Some(handle) {
                                ids.active_set_id = islands.active_kinematic_set.len();
                                islands.active_kinematic_set.push(*handle);
                            }
                        }
                        RigidBodyType::Fixed => {}
                    }
                }

                // Update the positions of the colliders.
                if changes.contains(RigidBodyChanges::POSITION)
                    || changes.contains(RigidBodyChanges::COLLIDERS)
                {
                    rb.colliders
                        .update_positions(colliders, modified_colliders, &rb.pos.position);

                    if rb.is_kinematic()
                        && islands.active_kinematic_set.get(ids.active_set_id) != Some(handle)
                    {
                        ids.active_set_id = islands.active_kinematic_set.len();
                        islands.active_kinematic_set.push(*handle);
                    }
                }

                // Push the body to the active set if it is not
                // sleeping and if it is not already inside of the active set.
                if changes.contains(RigidBodyChanges::SLEEP)
                    && !activation.sleeping // May happen if the body was put to sleep manually.
                    && rb.is_dynamic() // Only dynamic bodies are in the active dynamic set.
                    && islands.active_dynamic_set.get(ids.active_set_id) != Some(handle)
                {
                    ids.active_set_id = islands.active_dynamic_set.len(); // This will handle the case where the activation_channel contains duplicates.
                    islands.active_dynamic_set.push(*handle);
                }
            } else {
                // We don't use islands. So just update the colliders' positions.
                if changes.contains(RigidBodyChanges::POSITION)
                    || changes.contains(RigidBodyChanges::COLLIDERS)
                {
                    rb.colliders
                        .update_positions(colliders, modified_colliders, &rb.pos.position);
                }
            }

            if changes.contains(RigidBodyChanges::DOMINANCE)
                || changes.contains(RigidBodyChanges::TYPE)
            {
                for handle in rb.colliders.0.iter() {
                    let co = colliders.index_mut_internal(*handle);
                    if !co.changes.contains(ColliderChanges::MODIFIED) {
                        modified_colliders.push(*handle);
                    }

                    co.changes |=
                        ColliderChanges::MODIFIED | ColliderChanges::PARENT_EFFECTIVE_DOMINANCE;
                }
            }

            rb.changes = RigidBodyChanges::empty();
            rb.ids = ids;
            rb.activation = activation;
        }

        // Adjust some ids, if needed.
        if let Some(islands) = islands.as_deref_mut() {
            if let Some((action, id)) = final_action {
                let active_set = match action {
                    FinalAction::UpdateActiveKinematicSetId => &mut islands.active_kinematic_set,
                    FinalAction::UpdateActiveDynamicSetId => &mut islands.active_dynamic_set,
                };

                if id < active_set.len() {
                    bodies.index_mut_internal(active_set[id]).ids.active_set_id = id;
                }
            }
        }
    }
}
