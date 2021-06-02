use crate::data::{BundleSet, ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{
    IslandManager, RigidBodyActivation, RigidBodyChanges, RigidBodyColliders, RigidBodyHandle,
    RigidBodyIds, RigidBodyPosition, RigidBodyType,
};
use crate::geometry::{ColliderChanges, ColliderHandle, ColliderParent, ColliderPosition};

pub(crate) fn handle_user_changes_to_colliders<Colliders>(
    bodies: &mut impl ComponentSet<RigidBodyPosition>,
    colliders: &mut Colliders,
    modified_colliders: &[ColliderHandle],
) where
    Colliders: ComponentSetMut<ColliderChanges>
        + ComponentSetMut<ColliderPosition>
        + ComponentSetOption<ColliderParent>,
{
    for handle in modified_colliders {
        // NOTE: we use `get` because the collider may no longer
        //       exist if it has been removed.
        let co_changes: Option<&ColliderChanges> = colliders.get(handle.0);

        if let Some(co_changes) = co_changes {
            if co_changes.contains(ColliderChanges::PARENT) {
                let co_parent: Option<&ColliderParent> = colliders.get(handle.0);

                if let Some(co_parent) = co_parent {
                    let parent_pos = bodies.index(co_parent.handle.0);

                    let new_pos = parent_pos.position * co_parent.pos_wrt_parent;
                    let new_changes = *co_changes | ColliderChanges::POSITION;
                    colliders.set_internal(handle.0, ColliderPosition(new_pos));
                    colliders.set_internal(handle.0, new_changes);
                }
            }
        }
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
                            if islands.active_dynamic_set.get(ids.active_set_id) == Some(&handle) {
                                islands.active_dynamic_set.swap_remove(ids.active_set_id);
                                final_action = Some((
                                    FinalAction::UpdateActiveDynamicSetId,
                                    ids.active_set_id,
                                ));
                            }

                            // Add to the active kinematic set.
                            if islands.active_kinematic_set.get(ids.active_set_id) != Some(&handle)
                            {
                                ids.active_set_id = islands.active_kinematic_set.len();
                                islands.active_kinematic_set.push(*handle);
                            }
                        }
                        RigidBodyType::Static => {}
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
