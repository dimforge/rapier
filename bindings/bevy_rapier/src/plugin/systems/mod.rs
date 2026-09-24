//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

mod character_controller;
mod collider;
mod joint;
mod multiple_rapier_contexts;
mod pid_controller;
mod remove;
mod rigid_body;
mod soft_body;
#[cfg(feature = "dim3")]
mod vehicle_controller;
mod writeback;

pub use character_controller::*;
pub use collider::*;
pub use joint::*;
pub use multiple_rapier_contexts::*;
pub use pid_controller::*;
pub use remove::*;
pub use rigid_body::*;
pub use soft_body::*;
#[cfg(feature = "dim3")]
pub use vehicle_controller::*;
pub use writeback::*;

use crate::dynamics::{
    RapierRigidBodyHandle, RigidBodyDisabled, SoftBodyDisabled, TransformInterpolation,
};
use crate::geometry::ColliderDisabled;
use crate::pipeline::{CollisionEvent, ContactForceEvent, PhysicsQuarantineEvent};
use crate::plugin::context::SimulationToRenderTime;
use crate::plugin::{RapierConfiguration, TimestepMode};
use crate::prelude::{BevyPhysicsHooks, BevyPhysicsHooksAdapter};
use bevy::ecs::system::{StaticSystemParam, SystemParamItem};
use bevy::prelude::*;

use super::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
pub fn step_simulation<Hooks>(
    mut commands: Commands,
    mut context: Query<(
        Entity,
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
        &RapierConfiguration,
        &mut SimulationToRenderTime,
    )>,
    timestep_mode: Res<TimestepMode>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time>,
    mut collision_events: MessageWriter<CollisionEvent>,
    mut contact_force_events: MessageWriter<ContactForceEvent>,
    mut quarantine_events: MessageWriter<PhysicsQuarantineEvent>,
    mut interpolation_query: Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());

    for (
        context_entity,
        mut context,
        mut context_colliders,
        mut joints,
        mut rigidbody_set,
        config,
        mut sim_to_render_time,
    ) in context.iter_mut()
    {
        let context = &mut *context;
        let context_colliders = &mut *context_colliders;

        if config.physics_pipeline_active {
            context.set_num_threads(config.num_threads);
            context.step_simulation(
                context_colliders,
                &mut joints,
                &mut rigidbody_set,
                config.gravity,
                config.simulation_mode,
                *timestep_mode,
                Some((&collision_events, &contact_force_events)),
                &hooks_adapter,
                &time,
                &mut sim_to_render_time,
                Some(&mut interpolation_query),
            );
        } else {
            rigidbody_set.propagate_modified_body_positions_to_colliders(context_colliders);
        }
        context.send_bevy_events(&mut collision_events, &mut contact_force_events);
        // Mirror the quarantine on the entities so that removing the component re-enables them.
        for entity in &context.quarantined_bodies {
            commands.entity(*entity).try_insert(RigidBodyDisabled);
        }
        for entity in &context.quarantined_colliders {
            commands.entity(*entity).try_insert(ColliderDisabled);
        }
        for entity in &context.quarantined_soft_bodies {
            commands.entity(*entity).try_insert(SoftBodyDisabled);
        }
        context.send_quarantine_events(context_entity, &mut quarantine_events);
    }
}

#[cfg(test)]
#[allow(missing_docs)]
pub mod tests {
    use bevy::time::TimePlugin;
    use rapier::geometry::CollisionEventFlags;
    use std::f32::consts::PI;

    use super::*;
    use crate::{
        plugin::{NoUserData, RapierPhysicsPlugin},
        prelude::{Collider, CollidingEntities, RigidBody},
        utils,
    };

    #[test]
    fn colliding_entities_updates() {
        let mut app = App::new();
        app.add_message::<CollisionEvent>()
            .add_systems(Update, update_colliding_entities);

        let entity1 = app.world_mut().spawn(CollidingEntities::default()).id();
        let entity2 = app.world_mut().spawn(CollidingEntities::default()).id();

        let mut collision_events = app
            .world_mut()
            .get_resource_mut::<Messages<CollisionEvent>>()
            .unwrap();
        collision_events.write(CollisionEvent::Started(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world()
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities1.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities1.iter().next().unwrap(),
            entity2,
            "Colliding entity should be equal to the second entity"
        );

        let colliding_entities2 = app
            .world()
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities2.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities2.iter().next().unwrap(),
            entity1,
            "Colliding entity should be equal to the first entity"
        );

        let mut collision_events = app
            .world_mut()
            .get_resource_mut::<Messages<CollisionEvent>>()
            .unwrap();
        collision_events.write(CollisionEvent::Stopped(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world()
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities1.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );

        let colliding_entities2 = app
            .world()
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities2.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );
    }

    #[test]
    fn transform_propagation() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));
        app.finish();

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world_mut()
                .spawn((child_transform, RigidBody::Fixed, Collider::ball(1.0)))
                .id();

            app.world_mut()
                .spawn(parent_transform)
                .add_children(&[child]);

            app.update();

            let world = app.world_mut();
            let rigidbody_set = world
                .query::<&RapierRigidBodySet>()
                .iter(world)
                .next()
                .unwrap();
            let child_transform = world.entity(child).get::<GlobalTransform>().unwrap();
            let child_handle = rigidbody_set.entity2body[&child];
            let child_body = rigidbody_set.bodies.get(child_handle).unwrap();
            let body_transform = utils::iso_to_transform(child_body.position());

            fn transforms_approx_equal(
                a: &GlobalTransform,
                b: &GlobalTransform,
                epsilon: f32,
            ) -> bool {
                a.translation().abs_diff_eq(b.translation(), epsilon)
                    && a.scale().abs_diff_eq(b.scale(), epsilon)
                    && a.rotation().abs_diff_eq(b.rotation(), epsilon)
            }
            assert!(
                transforms_approx_equal(
                    &GlobalTransform::from(body_transform),
                    child_transform,
                    1.0e-5,
                ),
                "Collider transforms should have have equal global rotation and translation"
            );
        }
    }

    #[test]
    fn transform_propagation2() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));
        app.finish();

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                // NOTE: in 2D the test will fail if the rotation is wrt. an axis
                //       other than Z because 2D physics objects can’t rotate wrt.
                //       other axes.
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world_mut()
                .spawn((child_transform, Collider::ball(1.0)))
                .id();

            let parent = app
                .world_mut()
                .spawn((parent_transform, RigidBody::Fixed))
                .add_children(&[child])
                .id();

            app.update();

            let child_transform = app
                .world()
                .entity(child)
                .get::<GlobalTransform>()
                .unwrap()
                .compute_transform();
            let world = app.world_mut();
            let (rigidbody_set, context_colliders) = world
                .query::<(&RapierRigidBodySet, &RapierContextColliders)>()
                .iter(world)
                .next()
                .unwrap();
            let parent_handle = rigidbody_set.entity2body[&parent];
            let parent_body = rigidbody_set.bodies.get(parent_handle).unwrap();
            let child_collider_handle = parent_body.colliders()[0];
            let child_collider = context_colliders
                .colliders
                .get(child_collider_handle)
                .unwrap();
            let body_transform = utils::iso_to_transform(child_collider.position());
            approx::assert_relative_eq!(
                body_transform.translation,
                child_transform.translation,
                epsilon = 1.0e-5
            );

            // Adjust signs to account for the quaternion’s double covering.
            let comparison_child_rotation =
                if body_transform.rotation.w * child_transform.rotation.w < 0.0 {
                    -child_transform.rotation
                } else {
                    child_transform.rotation
                };

            approx::assert_relative_eq!(
                body_transform.rotation,
                comparison_child_rotation,
                epsilon = 1.0e-5
            );
            approx::assert_relative_eq!(body_transform.scale, child_transform.scale,);
        }
    }
}
