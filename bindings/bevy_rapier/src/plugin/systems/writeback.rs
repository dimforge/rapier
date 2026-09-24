use crate::dynamics::MassProperties;
use crate::dynamics::ReadMassProperties;
use crate::geometry::{
    Collider, ColliderMassProperties, RapierColliderHandle, ReadColliderMassProperties,
};
use crate::plugin::context::{RapierContextColliders, RapierContextEntityLink, RapierRigidBodySet};
use crate::plugin::RapierConfiguration;
use crate::prelude::MassModifiedEvent;
use bevy::prelude::*;

/// System responsible for writing updated mass properties back into the [`ReadMassProperties`] component.
pub fn writeback_mass_properties(
    link: Query<&RapierContextEntityLink>,
    rigidbody_set: Query<&RapierRigidBodySet>,
    config: Query<&RapierConfiguration>,

    mut mass_props: Query<&mut ReadMassProperties>,
    mut mass_modified: MessageReader<MassModifiedEvent>,
) {
    for entity in mass_modified.read() {
        let link = link
            .get(entity.0)
            .expect("Could not find `RapierContextEntityLink`");
        let config = config
            .get(link.0)
            .expect("Could not find `RapierConfiguration`");
        if config.physics_pipeline_active {
            let Ok(rigidbody_set) = rigidbody_set.get(link.0) else {
                continue;
            };

            if let Some(handle) = rigidbody_set.entity2body.get(entity).copied() {
                if let Some(rb) = rigidbody_set.bodies.get(handle) {
                    if let Ok(mut mass_props) = mass_props.get_mut(**entity) {
                        let new_mass_props =
                            MassProperties::from_rapier(rb.mass_properties().local_mprops);

                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        if mass_props.get() != &new_mass_props {
                            mass_props.set(new_mass_props);
                        }
                    }
                }
            }
        }
    }
}

/// System responsible for writing the mass properties of colliders back into the
/// [`ReadColliderMassProperties`] component.
///
/// This runs for colliders that were just created, or with a modified shape or
/// [`ColliderMassProperties`].
pub fn writeback_collider_mass_properties(
    context_colliders: Query<&RapierContextColliders>,
    mut colliders: Query<
        (
            &RapierColliderHandle,
            &RapierContextEntityLink,
            &mut ReadColliderMassProperties,
        ),
        Or<(
            Added<RapierColliderHandle>,
            Added<ReadColliderMassProperties>,
            Changed<Collider>,
            Changed<ColliderMassProperties>,
        )>,
    >,
) {
    for (handle, link, mut mass_props) in colliders.iter_mut() {
        let Some(co) = context_colliders
            .get(link.0)
            .ok()
            .and_then(|ctxt| ctxt.colliders.get(handle.0))
        else {
            continue;
        };

        // NOTE: only write actual changes to avoid triggering bevy's change detection.
        mass_props.set_if_neq(ReadColliderMassProperties::from_rapier(co));
    }
}
