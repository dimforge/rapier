use crate::dynamics::{DeformableCollider, DeformableColliderError, ReadMassProperties};
use crate::geometry::Collider;
use crate::plugin::context::systemparams::{RapierEntity, RAPIER_CONTEXT_EXPECT_ERROR};
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::{
    context::{RapierContextColliders, RapierRigidBodySet},
    RapierConfiguration,
};
use crate::prelude::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, ColliderDisabled, ColliderMassProperties,
    ColliderScale, CollidingEntities, CollisionEvent, CollisionGroups, ContactForceEventThreshold,
    ContactSkin, Friction, MassModifiedEvent, MassProperties, RapierColliderHandle,
    RapierRigidBodyHandle, Restitution, Sensor, SolverGroups,
};
use crate::utils;
use bevy::prelude::*;

use super::RapierContextLinkResolver;
use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::ColliderBuilder;
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::prelude::{AsyncCollider, AsyncSceneCollider},
    bevy::world_serialization::WorldInstance,
};

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// Components related to colliders.
pub type ColliderComponents<'a> = (
    (Entity, Option<&'a RapierContextEntityLink>),
    &'a Collider,
    Option<&'a Sensor>,
    Option<&'a ColliderMassProperties>,
    Option<&'a ActiveEvents>,
    Option<&'a ActiveHooks>,
    Option<&'a ActiveCollisionTypes>,
    Option<&'a Friction>,
    Option<&'a Restitution>,
    Option<&'a ContactSkin>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ColliderDisabled>,
);

/// System responsible for applying [`GlobalTransform`] scale and/or [`ColliderScale`] to
/// colliders.
pub fn apply_scale(
    config: Query<&RapierConfiguration>,
    mut changed_collider_scales: Query<
        (
            &mut Collider,
            &RapierContextEntityLink,
            &GlobalTransform,
            Option<&ColliderScale>,
        ),
        Or<(
            Changed<Collider>,
            Changed<GlobalTransform>,
            Changed<ColliderScale>,
        )>,
    >,
) {
    for (mut shape, link, transform, custom_scale) in changed_collider_scales.iter_mut() {
        let config = config.get(link.0).unwrap();
        #[cfg(feature = "dim2")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => {
                *scale * transform.compute_transform().scale.xy()
            }
            None => transform.compute_transform().scale.xy(),
        };
        #[cfg(feature = "dim3")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => *scale * transform.compute_transform().scale,
            None => transform.compute_transform().scale,
        };

        if shape.scale != crate::geometry::get_snapped_scale(effective_scale) {
            shape.set_scale(effective_scale, config.scaled_shape_subdivision);
        }
    }
}

/// System responsible for applying changes the user made to a collider-related component.
pub fn apply_collider_user_changes(
    mut context: Query<(&RapierRigidBodySet, &mut RapierContextColliders)>,
    config: Query<&RapierConfiguration>,
    (changed_collider_transforms, child_of_query, transform_query): (
        Query<
            (RapierEntity, &RapierColliderHandle, &GlobalTransform),
            (Without<RapierRigidBodyHandle>, Changed<GlobalTransform>),
        >,
        Query<&ChildOf>,
        Query<&Transform>,
    ),

    changed_shapes: Query<(RapierEntity, &RapierColliderHandle, &Collider), Changed<Collider>>,
    changed_active_events: Query<
        (RapierEntity, &RapierColliderHandle, &ActiveEvents),
        Changed<ActiveEvents>,
    >,
    changed_active_hooks: Query<
        (RapierEntity, &RapierColliderHandle, &ActiveHooks),
        Changed<ActiveHooks>,
    >,
    changed_active_collision_types: Query<
        (RapierEntity, &RapierColliderHandle, &ActiveCollisionTypes),
        Changed<ActiveCollisionTypes>,
    >,
    (changed_friction, changed_restitution, changed_contact_skin): (
        Query<(RapierEntity, &RapierColliderHandle, &Friction), Changed<Friction>>,
        Query<(RapierEntity, &RapierColliderHandle, &Restitution), Changed<Restitution>>,
        Query<(RapierEntity, &RapierColliderHandle, &ContactSkin), Changed<ContactSkin>>,
    ),
    changed_collision_groups: Query<
        (RapierEntity, &RapierColliderHandle, &CollisionGroups),
        Changed<CollisionGroups>,
    >,
    changed_solver_groups: Query<
        (RapierEntity, &RapierColliderHandle, &SolverGroups),
        Changed<SolverGroups>,
    >,
    changed_sensors: Query<(RapierEntity, &RapierColliderHandle, &Sensor), Changed<Sensor>>,
    changed_disabled: Query<
        (RapierEntity, &RapierColliderHandle, &ColliderDisabled),
        Changed<ColliderDisabled>,
    >,
    changed_contact_force_threshold: Query<
        (
            RapierEntity,
            &RapierColliderHandle,
            &ContactForceEventThreshold,
        ),
        Changed<ContactForceEventThreshold>,
    >,
    changed_collider_mass_props: Query<
        (RapierEntity, &RapierColliderHandle, &ColliderMassProperties),
        Changed<ColliderMassProperties>,
    >,

    mut mass_modified: MessageWriter<MassModifiedEvent>,
) {
    for (rapier_entity, handle, transform) in changed_collider_transforms.iter() {
        let (rigidbody_set, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if is_deformable(&context_colliders, handle) {
            // A deformable collider follows the particles of its soft body.
            continue;
        }
        if context_colliders
            .collider_parent(rigidbody_set, rapier_entity.entity)
            .is_some()
        {
            let (_, collider_position) = collider_offset(
                rapier_entity.entity,
                rigidbody_set,
                &child_of_query,
                &transform_query,
            );

            if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
                let new_pos = utils::transform_to_iso(&collider_position);

                if co
                    .position_wrt_parent()
                    .map(|pos| *pos != new_pos)
                    .unwrap_or(true)
                {
                    co.set_position_wrt_parent(new_pos);
                }
            }
        } else if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            let new_pos = utils::transform_to_iso(&transform.compute_transform());

            if *co.position() != new_pos {
                co.set_position(utils::transform_to_iso(&transform.compute_transform()));
            }
        }
    }

    for (rapier_entity, handle, shape) in changed_shapes.iter() {
        let (rigidbody_set, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        let config = config.get(rapier_entity.rapier_context_link.0).unwrap();
        if is_deformable(&context_colliders, handle) {
            // The shape of a deformable collider is managed by its soft body.
            continue;
        }
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            let mut scaled_shape = shape.clone();
            scaled_shape.set_scale(shape.scale, config.scaled_shape_subdivision);
            co.set_shape(scaled_shape.raw.clone());

            if let Some(body) = co.parent() {
                if let Some(body_entity) = rigidbody_set.rigid_body_entity(body) {
                    mass_modified.write(body_entity.into());
                }
            }
        }
    }

    for (rapier_entity, handle, active_events) in changed_active_events.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_active_events((*active_events).into())
        }
    }

    for (rapier_entity, handle, active_hooks) in changed_active_hooks.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_active_hooks((*active_hooks).into())
        }
    }

    for (rapier_entity, handle, active_collision_types) in changed_active_collision_types.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_active_collision_types((*active_collision_types).into())
        }
    }

    for (rapier_entity, handle, friction) in changed_friction.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_friction(friction.coefficient);
            co.set_friction_combine_rule(friction.combine_rule.into());
        }
    }

    for (rapier_entity, handle, restitution) in changed_restitution.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_restitution(restitution.coefficient);
            co.set_restitution_combine_rule(restitution.combine_rule.into());
        }
    }

    for (rapier_entity, handle, contact_skin) in changed_contact_skin.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_contact_skin(contact_skin.0);
        }
    }

    for (rapier_entity, handle, collision_groups) in changed_collision_groups.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_collision_groups((*collision_groups).into());
        }
    }

    for (rapier_entity, handle, solver_groups) in changed_solver_groups.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_solver_groups((*solver_groups).into());
        }
    }

    for (rapier_entity, handle, _) in changed_sensors.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_sensor(true);
        }
    }

    for (rapier_entity, handle, _) in changed_disabled.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_enabled(false);
        }
    }

    for (rapier_entity, handle, threshold) in changed_contact_force_threshold.iter() {
        let (_, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            co.set_contact_force_event_threshold(threshold.0);
        }
    }

    for (rapier_entity, handle, mprops) in changed_collider_mass_props.iter() {
        let (rigidbody_set, mut context_colliders) = context
            .get_mut(rapier_entity.rapier_context_link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(co) = context_colliders.colliders.get_mut(handle.0) {
            match mprops {
                ColliderMassProperties::Density(density) => co.set_density(*density),
                ColliderMassProperties::Mass(mass) => co.set_mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    co.set_mass_properties(mprops.into_rapier())
                }
            }

            if let Some(body) = co.parent() {
                if let Some(body_entity) = rigidbody_set.rigid_body_entity(body) {
                    mass_modified.write(body_entity.into());
                }
            }
        }
    }
}

/// Whether the collider of `handle` is bound to a soft body.
fn is_deformable(
    context_colliders: &RapierContextColliders,
    handle: &RapierColliderHandle,
) -> bool {
    context_colliders
        .colliders
        .get(handle.0)
        .is_some_and(|co| co.deformable_mesh_ref().is_some())
}

pub(crate) fn collider_offset(
    entity: Entity,
    rigidbody_set: &RapierRigidBodySet,
    child_of_query: &Query<&ChildOf>,
    transform_query: &Query<&Transform>,
) -> (Option<RigidBodyHandle>, Transform) {
    let mut body_entity = entity;
    let mut body_handle = rigidbody_set.entity2body.get(&body_entity).copied();
    let mut child_transform = Transform::default();
    while body_handle.is_none() {
        if let Ok(child_of) = child_of_query.get(body_entity) {
            if let Ok(transform) = transform_query.get(body_entity) {
                child_transform = *transform * child_transform;
            }
            body_entity = child_of.parent();
        } else {
            break;
        }

        body_handle = rigidbody_set.entity2body.get(&body_entity).copied();
    }

    if body_handle.is_some() {
        if let Ok(transform) = transform_query.get(body_entity) {
            let scale_transform = Transform {
                scale: transform.scale,
                ..default()
            };

            child_transform = scale_transform * child_transform;
        }
    }

    (body_handle, child_transform)
}

/// The proxy rigid-body a [`DeformableCollider`] targeting `target` is bound to: the whole-body
/// cluster's proxy of a soft body entity, or the proxy of a soft-body cluster entity.
fn deformable_collider_parent(
    rigidbody_set: &RapierRigidBodySet,
    target: Entity,
) -> Option<RigidBodyHandle> {
    if let Some(sb) = rigidbody_set.soft_body(target) {
        return Some(sb.root_body());
    }
    let handle = *rigidbody_set.entity2body.get(&target)?;
    rigidbody_set
        .bodies
        .get(handle)?
        .is_soft_frame()
        .then_some(handle)
}

/// System responsible for creating new Rapier colliders from the related `bevy_rapier` components.
pub fn init_colliders(
    mut commands: Commands,
    config: Query<&RapierConfiguration>,
    mut context_access: Query<(&mut RapierRigidBodySet, &mut RapierContextColliders)>,
    context_links: RapierContextLinkResolver,
    colliders: Query<
        (ColliderComponents, Option<&GlobalTransform>),
        (
            Without<RapierColliderHandle>,
            Without<DeformableColliderError>,
        ),
    >,
    mut rigid_body_mprops: Query<&mut ReadMassProperties>,
    child_of_query: Query<&ChildOf>,
    transform_query: Query<&Transform>,
    deformables: Query<&DeformableCollider>,
) {
    for (
        (
            (entity, entity_context_link),
            shape,
            sensor,
            mprops,
            active_events,
            active_hooks,
            active_collision_types,
            friction,
            restitution,
            contact_skin,
            collision_groups,
            solver_groups,
            contact_force_event_threshold,
            disabled,
        ),
        global_transform,
    ) in colliders.iter()
    {
        // Use the RapierContextEntityLink, or insert the context of an ancestor or the default one.
        let context_entity = context_links.resolve(entity, entity_context_link, &mut commands);
        let Some(context_entity) = context_entity else {
            continue;
        };

        let config = config.get(context_entity).unwrap_or_else(|_| {
            panic!("Failed to retrieve `RapierConfiguration` on entity {context_entity}.")
        });

        let Some(mut rigidbody_set_collider_set) = context_access.get_mut(context_entity).ok()
        else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let context_colliders = &mut *rigidbody_set_collider_set.1;
        let rigidbody_set = &mut *rigidbody_set_collider_set.0;
        let mut scaled_shape = shape.clone();
        scaled_shape.set_scale(shape.scale, config.scaled_shape_subdivision);
        let mut builder = ColliderBuilder::new(scaled_shape.raw.clone());

        builder = builder.sensor(sensor.is_some());
        builder = builder.enabled(disabled.is_none());

        if let Some(mprops) = mprops {
            builder = match mprops {
                ColliderMassProperties::Density(density) => builder.density(*density),
                ColliderMassProperties::Mass(mass) => builder.mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    builder.mass_properties(mprops.into_rapier())
                }
            };
        }

        if let Some(active_events) = active_events {
            builder = builder.active_events((*active_events).into());
        }

        if let Some(active_hooks) = active_hooks {
            builder = builder.active_hooks((*active_hooks).into());
        }

        if let Some(active_collision_types) = active_collision_types {
            builder = builder.active_collision_types((*active_collision_types).into());
        }

        if let Some(friction) = friction {
            builder = builder
                .friction(friction.coefficient)
                .friction_combine_rule(friction.combine_rule.into());
        }

        if let Some(restitution) = restitution {
            builder = builder
                .restitution(restitution.coefficient)
                .restitution_combine_rule(restitution.combine_rule.into());
        }

        if let Some(contact_skin) = contact_skin {
            builder = builder.contact_skin(contact_skin.0);
        }

        if let Some(collision_groups) = collision_groups {
            builder = builder.collision_groups((*collision_groups).into());
        }

        if let Some(solver_groups) = solver_groups {
            builder = builder.solver_groups((*solver_groups).into());
        }

        if let Some(threshold) = contact_force_event_threshold {
            builder = builder.contact_force_event_threshold(threshold.0);
        }
        builder = builder.user_data(entity.to_bits() as u128);

        if let Ok(deformable) = deformables.get(entity) {
            // The soft body (or cluster) may not be created yet: try again next frame.
            let Some(proxy) = deformable_collider_parent(rigidbody_set, deformable.target) else {
                continue;
            };
            let proxy_pose = rigidbody_set
                .bodies
                .get(proxy)
                .map(|rb| *rb.position())
                .unwrap_or_default();
            let global_transform = global_transform.cloned().unwrap_or_default();
            let pose = utils::transform_to_iso(&global_transform.compute_transform());
            // The vertices are placed by the entity's global transform.
            builder = builder.position(proxy_pose.inverse() * pose);
            match context_colliders.colliders.insert_deformable(
                builder,
                deformable.binding.clone(),
                proxy,
                &mut rigidbody_set.bodies,
                &mut rigidbody_set.soft_bodies,
            ) {
                Ok(handle) => {
                    commands.entity(entity).insert(RapierColliderHandle(handle));
                    context_colliders.entity2collider.insert(entity, handle);
                }
                Err(err) => {
                    log::error!("Failed to create the deformable collider of {entity}: {err}");
                    commands.entity(entity).insert(DeformableColliderError(err));
                }
            }
            continue;
        }

        let body_entity = entity;
        let (body_handle, child_transform) =
            collider_offset(entity, rigidbody_set, &child_of_query, &transform_query);

        let handle = if let Some(body_handle) = body_handle {
            builder = builder.position(utils::transform_to_iso(&child_transform));
            let handle = context_colliders.colliders.insert_with_parent(
                builder,
                body_handle,
                &mut rigidbody_set.bodies,
            );
            if let Ok(mut mprops) = rigid_body_mprops.get_mut(body_entity) {
                // Inserting the collider changed the rigid-body’s mass properties.
                // Read them back from the engine.
                if let Some(parent_body) = rigidbody_set.bodies.get(body_handle) {
                    mprops.set(MassProperties::from_rapier(
                        parent_body.mass_properties().local_mprops,
                    ));
                }
            }
            handle
        } else {
            let global_transform = global_transform.cloned().unwrap_or_default();
            builder = builder.position(utils::transform_to_iso(
                &global_transform.compute_transform(),
            ));
            context_colliders.colliders.insert(builder)
        };

        commands.entity(entity).insert(RapierColliderHandle(handle));
        context_colliders.entity2collider.insert(entity, handle);
    }
}

/// System responsible for creating `Collider` components from `AsyncCollider` components if the
/// corresponding mesh has become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    async_colliders: Query<(Entity, &Mesh3d, &AsyncCollider)>,
) {
    for (entity, mesh_handle, async_collider) in async_colliders.iter() {
        if let Some(mesh) = meshes.get(mesh_handle) {
            match Collider::from_bevy_mesh(mesh, &async_collider.0) {
                Some(collider) => {
                    commands
                        .entity(entity)
                        .insert(collider)
                        .remove::<AsyncCollider>();
                }
                None => log::error!("Unable to generate collider from mesh {mesh:?}"),
            }
        }
    }
}

/// System responsible for creating `Collider` components from `AsyncSceneCollider` components if the
/// corresponding scene has become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scene_spawner: If<Res<WorldInstanceSpawner>>,
    async_colliders: Query<(Entity, &WorldInstance, &AsyncSceneCollider)>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Mesh3d)>,
) {
    for (scene_entity, scene_instance, async_collider) in async_colliders.iter() {
        if scene_spawner.instance_is_ready(**scene_instance) {
            for child_entity in children.iter_descendants(scene_entity) {
                if let Ok((name, handle)) = mesh_handles.get(child_entity) {
                    let shape = async_collider
                        .named_shapes
                        .get(name.as_str())
                        .unwrap_or(&async_collider.shape);
                    if let Some(shape) = shape {
                        let mesh = meshes.get(handle).unwrap(); // NOTE: Mesh is already loaded
                        match Collider::from_bevy_mesh(mesh, shape) {
                            Some(collider) => {
                                commands.entity(child_entity).insert(collider);
                            }
                            None => log::error!(
                                "Unable to generate collider from mesh {mesh:?} with name {name}"
                            ),
                        }
                    }
                }
            }

            commands.entity(scene_entity).remove::<AsyncSceneCollider>();
        }
    }
}

/// Adds entity to [`CollidingEntities`] on starting collision and removes from it when the
/// collision ends.
pub fn update_colliding_entities(
    mut collision_events: MessageReader<CollisionEvent>,
    mut colliding_entities: Query<&mut CollidingEntities>,
) {
    for event in collision_events.read() {
        match event.to_owned() {
            CollisionEvent::Started(entity1, entity2, _) => {
                if let Ok(mut entities) = colliding_entities.get_mut(entity1) {
                    entities.0.insert(entity2);
                }
                if let Ok(mut entities) = colliding_entities.get_mut(entity2) {
                    entities.0.insert(entity1);
                }
            }
            CollisionEvent::Stopped(entity1, entity2, _) => {
                if let Ok(mut entities) = colliding_entities.get_mut(entity1) {
                    entities.0.remove(&entity2);
                }
                if let Ok(mut entities) = colliding_entities.get_mut(entity2) {
                    entities.0.remove(&entity1);
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(missing_docs)]
pub mod test {
    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_collider_initializes() {
        use super::*;
        use bevy::{mesh::MeshPlugin, world_serialization::WorldSerializationPlugin};

        let mut app = App::new();
        app.add_plugins((AssetPlugin::default(), MeshPlugin, WorldSerializationPlugin));
        app.add_systems(Update, init_async_colliders);

        app.finish();

        let mut meshes = app.world_mut().resource_mut::<Assets<Mesh>>();
        let cube = meshes.add(Cuboid::default());

        let entity = app
            .world_mut()
            .spawn((Mesh3d(cube), AsyncCollider::default()))
            .id();

        app.update();

        let entity = app.world().entity(entity);
        assert!(
            entity.get::<Collider>().is_some(),
            "Collider component should be added"
        );
        assert!(
            entity.get::<AsyncCollider>().is_none(),
            "AsyncCollider component should be removed after Collider component creation"
        );
    }

    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_collider_voxels_and_converted_initialize() {
        use super::*;
        use crate::geometry::{ComputedColliderShape, FillMode, MeshConverter};
        use bevy::{mesh::MeshPlugin, world_serialization::WorldSerializationPlugin};

        let mut app = App::new();
        app.add_plugins((AssetPlugin::default(), MeshPlugin, WorldSerializationPlugin));
        app.add_systems(Update, init_async_colliders);

        app.finish();

        let mut meshes = app.world_mut().resource_mut::<Assets<Mesh>>();
        let cube = meshes.add(Cuboid::default());

        let voxels = app
            .world_mut()
            .spawn((
                Mesh3d(cube.clone()),
                AsyncCollider(ComputedColliderShape::Voxels {
                    voxel_size: 0.25,
                    fill_mode: FillMode::default(),
                }),
            ))
            .id();
        let aabb = app
            .world_mut()
            .spawn((
                Mesh3d(cube),
                AsyncCollider(ComputedColliderShape::Converted(MeshConverter::Aabb)),
            ))
            .id();

        app.update();

        let voxels = app.world().entity(voxels).get::<Collider>().unwrap();
        assert!(voxels.as_voxels().is_some());
        let aabb = app.world().entity(aabb).get::<Collider>().unwrap();
        assert!(aabb.as_cuboid().is_some());
    }

    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_scene_collider_initializes() {
        use super::*;
        use bevy::{mesh::MeshPlugin, world_serialization::WorldSerializationPlugin};

        let mut app = App::new();
        app.add_plugins((AssetPlugin::default(), MeshPlugin, WorldSerializationPlugin));
        app.add_systems(PostUpdate, init_async_scene_colliders);

        let mut meshes = app.world_mut().resource_mut::<Assets<Mesh>>();
        let cube_handle = meshes.add(Cuboid::default());
        let capsule_handle = meshes.add(Capsule3d::default());
        let cube = app
            .world_mut()
            .spawn((Name::new("Cube"), Mesh3d(cube_handle)))
            .id();
        let capsule = app
            .world_mut()
            .spawn((Name::new("Capsule"), Mesh3d(capsule_handle)))
            .id();

        let mut scenes = app.world_mut().resource_mut::<Assets<WorldAsset>>();
        let scene = scenes.add(WorldAsset::new(World::new()));

        let mut named_shapes = bevy::platform::collections::HashMap::default();
        named_shapes.insert("Capsule".to_string(), None);
        let parent = app
            .world_mut()
            .spawn((
                WorldAssetRoot(scene),
                AsyncSceneCollider {
                    named_shapes,
                    ..Default::default()
                },
            ))
            .add_children(&[cube, capsule])
            .id();

        app.update();

        assert!(
            app.world().entity(cube).get::<Collider>().is_some(),
            "Collider component should be added for cube"
        );
        assert!(
            app.world().entity(capsule).get::<Collider>().is_none(),
            "Collider component shouldn't be added for capsule"
        );
        assert!(
            app.world().entity(parent).get::<AsyncCollider>().is_none(),
            "AsyncSceneCollider component should be removed after Collider components creation"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{CoefficientCombineRule, GravityScale, RigidBody};
    use crate::geometry::{Group, InteractionTestMode, ReadColliderMassProperties};
    use crate::math::Vect;
    use crate::plugin::context::systemparams::ReadRapierContext;
    use crate::plugin::context::{RapierContextSimulation, RapierRigidBodySet};
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use crate::prelude::QueryFilter;
    use bevy::ecs::system::RunSystemOnce;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use rapier::geometry::ColliderBuilder;

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
        app.finish();
        app
    }

    fn with_collider<R>(
        app: &mut App,
        entity: Entity,
        f: impl FnOnce(&RapierContextColliders, &rapier::geometry::Collider) -> R,
    ) -> R {
        let world = app.world_mut();
        let set = world
            .query::<&RapierContextColliders>()
            .single(world)
            .unwrap();
        let co = set.colliders.get(set.entity2collider()[&entity]).unwrap();
        f(set, co)
    }

    #[test]
    fn collider_components_are_reset_on_removal() {
        let mut app = test_app();
        app.update();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                Collider::ball(0.5),
                Friction {
                    coefficient: 1.0,
                    combine_rule: CoefficientCombineRule::GeometricMean,
                },
                Restitution {
                    coefficient: 0.8,
                    combine_rule: CoefficientCombineRule::ClampedSum,
                },
                ContactSkin(0.1),
                CollisionGroups::new(Group::GROUP_2, Group::GROUP_3)
                    .with_test_mode(InteractionTestMode::Or),
                SolverGroups::new(Group::GROUP_4, Group::GROUP_5),
                (
                    ActiveEvents::all(),
                    ActiveHooks::all(),
                    ActiveCollisionTypes::all(),
                    ContactForceEventThreshold(10.0),
                    ColliderMassProperties::Density(3.0),
                ),
            ))
            .id();
        app.update();

        with_collider(&mut app, entity, |_, co| {
            assert_eq!(co.friction(), 1.0);
            assert_eq!(
                co.friction_combine_rule(),
                rapier::dynamics::CoefficientCombineRule::GeometricMean
            );
            assert_eq!(co.density(), 3.0);
            assert_eq!(
                co.collision_groups().test_mode,
                rapier::geometry::InteractionTestMode::Or
            );
        });

        app.world_mut().entity_mut(entity).remove::<(
            Friction,
            Restitution,
            ContactSkin,
            CollisionGroups,
            SolverGroups,
            ActiveEvents,
            ActiveHooks,
            ActiveCollisionTypes,
            ContactForceEventThreshold,
            ColliderMassProperties,
        )>();
        app.update();

        let defaults = ColliderBuilder::default();
        with_collider(&mut app, entity, |_, co| {
            assert_eq!(co.friction(), defaults.friction);
            assert_eq!(co.friction_combine_rule(), defaults.friction_combine_rule);
            assert_eq!(co.restitution(), defaults.restitution);
            assert_eq!(
                co.restitution_combine_rule(),
                defaults.restitution_combine_rule
            );
            assert_eq!(co.contact_skin(), defaults.contact_skin);
            assert_eq!(co.collision_groups(), defaults.collision_groups);
            assert_eq!(co.solver_groups(), defaults.solver_groups);
            assert_eq!(co.active_events(), defaults.active_events);
            assert_eq!(co.active_hooks(), defaults.active_hooks);
            assert_eq!(co.active_collision_types(), defaults.active_collision_types);
            assert_eq!(
                co.contact_force_event_threshold(),
                defaults.contact_force_event_threshold
            );
            assert_eq!(co.density(), ColliderBuilder::default_density());
        });
    }

    #[test]
    fn collider_mass_properties_are_read_back() {
        let mut app = test_app();
        app.update();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                GravityScale(0.0),
                Collider::ball(0.5),
                ColliderMassProperties::Density(2.0),
                ReadColliderMassProperties::default(),
            ))
            .id();
        app.update();

        #[cfg(feature = "dim2")]
        let volume = |r: f32| std::f32::consts::PI * r * r;
        #[cfg(feature = "dim3")]
        let volume = |r: f32| 4.0 / 3.0 * std::f32::consts::PI * r * r * r;

        let props = *app
            .world()
            .get::<ReadColliderMassProperties>(entity)
            .unwrap();
        approx::assert_relative_eq!(props.volume, volume(0.5), epsilon = 1.0e-5);
        assert_eq!(props.density, 2.0);
        approx::assert_relative_eq!(props.mass, 2.0 * volume(0.5), epsilon = 1.0e-5);
        assert_eq!(props.local_mass_properties.mass, props.mass);

        with_collider(&mut app, entity, |set, _| {
            assert_eq!(set.volume(entity), Some(props.volume));
            assert_eq!(set.mass(entity), Some(props.mass));
            assert_eq!(set.density(entity), Some(2.0));
            assert_eq!(set.volume(Entity::PLACEHOLDER), None);

            let aabb = set.compute_aabb(entity).unwrap();
            approx::assert_relative_eq!(Vect::from(aabb.min), Vect::splat(-0.5));
            approx::assert_relative_eq!(Vect::from(aabb.max), Vect::splat(0.5));

            let swept = set
                .compute_swept_aabb(entity, Transform::from_xyz(2.0, 0.0, 0.0))
                .unwrap();
            approx::assert_relative_eq!(Vect::from(swept.min), Vect::splat(-0.5));
            approx::assert_relative_eq!(swept.max.x, 2.5);
        });

        // Shape changes are read back.
        app.world_mut()
            .entity_mut(entity)
            .insert(Collider::ball(1.0));
        app.update();
        let props = *app
            .world()
            .get::<ReadColliderMassProperties>(entity)
            .unwrap();
        approx::assert_relative_eq!(props.volume, volume(1.0), epsilon = 1.0e-5);
        approx::assert_relative_eq!(props.mass, 2.0 * volume(1.0), epsilon = 1.0e-5);

        // Mass-properties changes are read back.
        app.world_mut()
            .entity_mut(entity)
            .insert(ColliderMassProperties::Mass(5.0));
        app.update();
        let props = *app
            .world()
            .get::<ReadColliderMassProperties>(entity)
            .unwrap();
        assert_eq!(props.mass, 5.0);
        approx::assert_relative_eq!(props.density, 5.0 / volume(1.0), epsilon = 1.0e-5);

        // Removing the custom mass-properties falls back to the default density.
        app.world_mut()
            .entity_mut(entity)
            .remove::<ColliderMassProperties>();
        app.update();
        let props = *app
            .world()
            .get::<ReadColliderMassProperties>(entity)
            .unwrap();
        assert_eq!(props.density, 1.0);
        approx::assert_relative_eq!(props.mass, volume(1.0), epsilon = 1.0e-5);

        // The rigid-body mass accounts for the reset collider mass.
        let world = app.world_mut();
        let set = world.query::<&RapierRigidBodySet>().single(world).unwrap();
        approx::assert_relative_eq!(set.mass(entity).unwrap(), volume(1.0), epsilon = 1.0e-5);
    }

    #[test]
    fn collision_groups_or_mode_enables_contacts() {
        let mut app = test_app();
        app.update();

        // The membership of `b` matches the filter of `a`, but not the other way around.
        let groups_a = CollisionGroups::new(Group::GROUP_1, Group::GROUP_2);
        let groups_b = CollisionGroups::new(Group::GROUP_2, Group::GROUP_3);

        let mut spawn_pair = |x: f32, mode: InteractionTestMode| {
            let a = app
                .world_mut()
                .spawn((
                    Transform::from_xyz(x, 0.0, 0.0),
                    RigidBody::Dynamic,
                    GravityScale(0.0),
                    Collider::ball(0.5),
                    groups_a.with_test_mode(mode),
                ))
                .id();
            let b = app
                .world_mut()
                .spawn((
                    Transform::from_xyz(x + 0.5, 0.0, 0.0),
                    Collider::ball(0.5),
                    groups_b.with_test_mode(mode),
                ))
                .id();
            (a, b)
        };
        let and_pair = spawn_pair(0.0, InteractionTestMode::And);
        let or_pair = spawn_pair(10.0, InteractionTestMode::Or);

        app.update();
        app.update();

        let world = app.world_mut();
        let (simulation, colliders, bodies) = world
            .query::<(
                &RapierContextSimulation,
                &RapierContextColliders,
                &RapierRigidBodySet,
            )>()
            .single(world)
            .unwrap();
        let in_contact = |(a, b): (Entity, Entity)| {
            simulation
                .contact_pair(colliders, bodies, a, b)
                .is_some_and(|pair| pair.has_any_active_contact())
        };
        assert!(!in_contact(and_pair));
        assert!(in_contact(or_pair));
    }

    #[test]
    fn query_filter_groups_honor_test_mode() {
        let mut app = test_app();
        app.update();

        let groups = CollisionGroups::new(Group::GROUP_1, Group::GROUP_2)
            .with_test_mode(InteractionTestMode::Or);
        let entity = app
            .world_mut()
            .spawn((Transform::default(), Collider::ball(0.5), groups))
            .id();
        app.update();
        app.update();

        let query_groups = CollisionGroups::new(Group::GROUP_2, Group::GROUP_3);
        let hit = |mode: InteractionTestMode| {
            move |context: ReadRapierContext| {
                let context = context.single().unwrap();
                let filter = QueryFilter::new().groups(query_groups.with_test_mode(mode));
                let mut hits = vec![];
                context.intersect_point(Vect::ZERO, filter, |e, _| {
                    hits.push(e);
                    true
                });
                hits
            }
        };

        let and_hits = app
            .world_mut()
            .run_system_once(hit(InteractionTestMode::And))
            .unwrap();
        assert!(and_hits.is_empty());
        let or_hits = app
            .world_mut()
            .run_system_once(hit(InteractionTestMode::Or))
            .unwrap();
        assert_eq!(or_hits, vec![entity]);
    }
}
