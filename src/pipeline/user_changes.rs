use crate::data::{BundleSet, ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{
    IslandManager, RigidBodyActivation, RigidBodyChanges, RigidBodyColliders, RigidBodyHandle,
    RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType,
};
use crate::geometry::{
    ColliderChanges, ColliderHandle, ColliderMassProps, ColliderParent, ColliderPosition,
    ColliderShape,
};
use parry::utils::hashmap::HashMap;

pub(crate) fn handle_user_changes_to_colliders<Bodies, Colliders>(
    bodies: &mut Bodies,
    colliders: &mut Colliders,
    modified_colliders: &[ColliderHandle],
) where
    Bodies: ComponentSet<RigidBodyPosition>
        + ComponentSet<RigidBodyColliders>
        + ComponentSetMut<RigidBodyMassProps>,
    Colliders: ComponentSetMut<ColliderChanges>
        + ComponentSetMut<ColliderPosition>
        + ComponentSetOption<ColliderParent>
        + ComponentSet<ColliderShape>
        + ComponentSet<ColliderMassProps>,
{
    // TODO: avoid this hashmap? We could perhaps add a new flag to RigidBodyChanges to
    //       indicated that the mass properties need to be recomputed?
    let mut mprops_to_update = HashMap::default();

    for handle in modified_colliders {
        // NOTE: we use `get` because the collider may no longer
        //       exist if it has been removed.
        let co_changes: Option<ColliderChanges> = colliders.get(handle.0).copied();

        if let Some(co_changes) = co_changes {
            if co_changes.contains(ColliderChanges::PARENT) {
                let co_parent: Option<&ColliderParent> = colliders.get(handle.0);

                if let Some(co_parent) = co_parent {
                    let parent_pos: &RigidBodyPosition = bodies.index(co_parent.handle.0);

                    let new_pos = parent_pos.position * co_parent.pos_wrt_parent;
                    let new_changes = co_changes | ColliderChanges::POSITION;
                    colliders.set_internal(handle.0, ColliderPosition(new_pos));
                    colliders.set_internal(handle.0, new_changes);
                }
            }

            if co_changes.contains(ColliderChanges::SHAPE) {
                let co_parent: Option<&ColliderParent> = colliders.get(handle.0);
                if let Some(co_parent) = co_parent {
                    mprops_to_update.insert(co_parent.handle, ());
                }
            }
        }
    }

    for (to_update, _) in mprops_to_update {
        let (rb_pos, rb_colliders): (&RigidBodyPosition, &RigidBodyColliders) =
            bodies.index_bundle(to_update.0);
        let position = rb_pos.position;
        // FIXME: remove the clone once we remove the ComponentSets.
        let attached_colliders = rb_colliders.clone();

        bodies.map_mut_internal(to_update.0, |rb_mprops| {
            rb_mprops.recompute_mass_properties_from_colliders(
                colliders,
                &attached_colliders,
                &position,
            )
        });
    }
}

pub(crate) fn handle_user_changes_to_rigid_bodies<Bodies, Colliders>(
    mut islands: Option<&mut IslandManager>,
    bodies: &mut Bodies,
    colliders: &mut Colliders,
    modified_bodies: &[RigidBodyHandle],
    modified_colliders: &mut Vec<ColliderHandle>,
) where
    Bodies: ComponentSetMut<RigidBodyChanges>
        + ComponentSet<RigidBodyType>
        + ComponentSetMut<RigidBodyIds>
        + ComponentSetMut<RigidBodyActivation>
        + ComponentSet<RigidBodyColliders>
        + ComponentSet<RigidBodyPosition>,
    Colliders: ComponentSetMut<ColliderPosition>
        + ComponentSetMut<ColliderChanges>
        + ComponentSetOption<ColliderParent>,
{
    enum FinalAction {
        UpdateActiveKinematicSetId,
        UpdateActiveDynamicSetId,
    }

    for handle in modified_bodies {
        let mut final_action = None;
        let changes: Option<&RigidBodyChanges> = bodies.get(handle.0);

        if changes.is_none() {
            // The body no longer exists.
            continue;
        }

        let mut changes = *changes.unwrap();
        let mut ids: RigidBodyIds = *bodies.index(handle.0);
        let mut activation: RigidBodyActivation = *bodies.index(handle.0);
        let (status, rb_colliders, poss): (
            &RigidBodyType,
            &RigidBodyColliders,
            &RigidBodyPosition,
        ) = bodies.index_bundle(handle.0);

        {
            // The body's status changed. We need to make sure
            // it is on the correct active set.
            if let Some(islands) = islands.as_deref_mut() {
                if changes.contains(RigidBodyChanges::TYPE) {
                    match status {
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
                    rb_colliders.update_positions(colliders, modified_colliders, &poss.position);

                    if status.is_kinematic()
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
                    && status.is_dynamic() // Only dynamic bodies are in the active dynamic set.
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
                    rb_colliders.update_positions(colliders, modified_colliders, &poss.position);
                }
            }

            if changes.contains(RigidBodyChanges::DOMINANCE)
                || changes.contains(RigidBodyChanges::TYPE)
            {
                for handle in rb_colliders.0.iter() {
                    colliders.map_mut_internal(handle.0, |co_changes: &mut ColliderChanges| {
                        if !co_changes.contains(ColliderChanges::MODIFIED) {
                            modified_colliders.push(*handle);
                        }

                        *co_changes |=
                            ColliderChanges::MODIFIED | ColliderChanges::PARENT_EFFECTIVE_DOMINANCE;
                    });
                }
            }

            bodies.set_internal(handle.0, RigidBodyChanges::empty());
            bodies.set_internal(handle.0, ids);
            bodies.set_internal(handle.0, activation);
        }

        // Adjust some ids, if needed.
        if let Some(islands) = islands.as_deref_mut() {
            if let Some((action, id)) = final_action {
                let active_set = match action {
                    FinalAction::UpdateActiveKinematicSetId => &mut islands.active_kinematic_set,
                    FinalAction::UpdateActiveDynamicSetId => &mut islands.active_dynamic_set,
                };

                if id < active_set.len() {
                    bodies.map_mut_internal(active_set[id].0, |ids2: &mut RigidBodyIds| {
                        ids2.active_set_id = id;
                    });
                }
            }
        }
    }
}
