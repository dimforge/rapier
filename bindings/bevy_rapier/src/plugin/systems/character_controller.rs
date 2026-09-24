use crate::control::controller_filter::ControllerExclusions;
use crate::control::CharacterCollision;
use crate::control::ControllerIgnored;
use crate::dynamics::RapierRigidBodyHandle;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::systemparams::RAPIER_CONTEXT_EXPECT_ERROR;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::RapierConfiguration;
use crate::prelude::context::RapierContextColliders;
use crate::prelude::context::RapierContextSimulation;
use crate::prelude::context::RapierRigidBodySet;
use crate::prelude::KinematicCharacterController;
use crate::prelude::KinematicCharacterControllerOutput;
use crate::utils;
use bevy::ecs::entity::EntityHashSet;
use bevy::prelude::*;
use rapier::geometry::{Collider as RapierCollider, ColliderHandle};
use rapier::math::Pose;
use rapier::pipeline::QueryFilter;

/// System responsible for applying the character controller translation to the underlying
/// collider.
pub fn update_character_controls(
    mut commands: Commands,
    config: Query<&RapierConfiguration>,
    mut context_access: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierRigidBodySet,
    )>,
    mut character_controllers: Query<(
        Entity,
        &RapierContextEntityLink,
        &mut KinematicCharacterController,
        Option<&mut KinematicCharacterControllerOutput>,
        Option<&RapierColliderHandle>,
        Option<&RapierRigidBodyHandle>,
        Option<&GlobalTransform>,
    )>,
    mut transforms: Query<&mut Transform>,
    ignored: Query<Entity, With<ControllerIgnored>>,
) {
    let ignored: EntityHashSet = ignored.iter().collect();
    for (
        entity,
        rapier_context_link,
        mut controller,
        output,
        collider_handle,
        body_handle,
        glob_transform,
    ) in character_controllers.iter_mut()
    {
        if let (Some(raw_controller), Some(translation)) =
            (controller.to_raw(), controller.translation)
        {
            let config = config
                .get(rapier_context_link.0)
                .expect("Could not get [`RapierConfiguration`]");
            let (mut context, mut context_colliders, mut rigidbody_set) = context_access
                .get_mut(rapier_context_link.0)
                .expect(RAPIER_CONTEXT_EXPECT_ERROR);

            let context = &mut *context;
            let rigidbody_set = &mut *rigidbody_set;
            let scaled_custom_shape =
                controller
                    .custom_shape
                    .as_ref()
                    .map(|(custom_shape, tra, rot)| {
                        // TODO: avoid the systematic scale somehow?
                        let mut scaled_shape = custom_shape.clone();
                        scaled_shape.set_scale(custom_shape.scale, config.scaled_shape_subdivision);

                        (scaled_shape, *tra, *rot)
                    });

            let parent_rigid_body = body_handle.map(|h| h.0).or_else(|| {
                collider_handle
                    .and_then(|h| context_colliders.colliders.get(h.0))
                    .and_then(|c| c.parent())
            });
            let entity_to_move = parent_rigid_body
                .and_then(|rb| rigidbody_set.rigid_body_entity(rb))
                .unwrap_or(entity);

            let (character_shape, character_pos) = if let Some((scaled_shape, tra, rot)) =
                &scaled_custom_shape
            {
                let mut shape_pos: Pose = utils::pose_from(*tra, *rot);

                if let Some(body) = body_handle.and_then(|h| rigidbody_set.bodies.get(h.0)) {
                    shape_pos = body.position() * shape_pos
                } else if let Some(gtransform) = glob_transform {
                    shape_pos = utils::transform_to_iso(&gtransform.compute_transform()) * shape_pos
                }

                (&*scaled_shape.raw, shape_pos)
            } else if let Some(collider) =
                collider_handle.and_then(|h| context_colliders.colliders.get(h.0))
            {
                (collider.shape(), *collider.position())
            } else {
                continue;
            };
            let character_shape = character_shape.clone_dyn();

            let exclude_collider = collider_handle.map(|h| h.0);

            let character_mass = controller
                .custom_mass
                .or_else(|| {
                    parent_rigid_body
                        .and_then(|h| rigidbody_set.bodies.get(h))
                        .map(|rb| rb.mass())
                })
                .unwrap_or(0.0);

            let exclusions = ControllerExclusions::new(
                &controller.exclude_colliders,
                &controller.exclude_rigid_bodies,
                &ignored,
                controller.filter_predicate.as_ref(),
                rigidbody_set,
            );
            let predicate =
                |_: ColliderHandle, collider: &RapierCollider| exclusions.test(collider);
            let mut filter = QueryFilter {
                flags: controller.filter_flags,
                groups: controller.filter_groups.map(|g| g.into()),
                exclude_collider: None,
                exclude_rigid_body: None,
                predicate: (!exclusions.is_empty())
                    .then_some(&predicate as &dyn Fn(ColliderHandle, &RapierCollider) -> bool),
            };

            if let Some(parent) = parent_rigid_body {
                filter = filter.exclude_rigid_body(parent);
            } else if let Some(excl_co) = exclude_collider {
                filter = filter.exclude_collider(excl_co)
            };

            let collisions = &mut context.character_collisions_collector;
            collisions.clear();

            let mut query_pipeline = context.broad_phase.as_query_pipeline_mut(
                context.narrow_phase.query_dispatcher(),
                &mut rigidbody_set.bodies,
                &mut context_colliders.colliders,
                filter,
            );

            let movement = raw_controller.move_shape(
                context.integration_parameters.dt,
                &query_pipeline.as_ref(),
                &*character_shape,
                &character_pos,
                translation,
                |c| collisions.push(c),
            );

            if controller.apply_impulse_to_dynamic_bodies {
                raw_controller.solve_character_collision_impulses(
                    context.integration_parameters.dt,
                    &mut query_pipeline,
                    &*character_shape,
                    character_mass,
                    collisions.iter(),
                )
            }

            if let Ok(mut transform) = transforms.get_mut(entity_to_move) {
                // TODO: take the parent’s GlobalTransform rotation into account?
                transform.translation.x += movement.translation.x;
                transform.translation.y += movement.translation.y;
                #[cfg(feature = "dim3")]
                {
                    transform.translation.z += movement.translation.z;
                }
            }

            let converted_collisions = context
                .character_collisions_collector
                .iter()
                .filter_map(|c| CharacterCollision::from_raw(&context_colliders, c));

            if let Some(mut output) = output {
                output.desired_translation = controller.translation.unwrap();
                output.effective_translation = movement.translation;
                output.grounded = movement.grounded;
                output.collisions.clear();
                output.collisions.extend(converted_collisions);
                output.is_sliding_down_slope = movement.is_sliding_down_slope;
            } else {
                commands
                    .entity(entity)
                    .insert(KinematicCharacterControllerOutput {
                        desired_translation: controller.translation.unwrap(),
                        effective_translation: movement.translation,
                        grounded: movement.grounded,
                        collisions: converted_collisions.collect(),
                        is_sliding_down_slope: movement.is_sliding_down_slope,
                    });
            }

            controller.translation = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::control::{
        ControllerFilterPredicate, ControllerIgnored, KinematicCharacterController,
    };
    use crate::geometry::Collider;
    use crate::math::Vect;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use crate::prelude::RigidBody;
    use bevy::ecs::entity::EntityHashSet;
    use bevy::prelude::*;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};

    #[cfg(feature = "dim2")]
    fn wall() -> Collider {
        Collider::cuboid(0.25, 2.0)
    }

    #[cfg(feature = "dim3")]
    fn wall() -> Collider {
        Collider::cuboid(0.25, 2.0, 2.0)
    }

    #[test]
    fn character_controller_exclusions() {
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

        // One row per scenario, each with its own wall in the character's path.
        let mut spawn_row = |row: usize, wall_is_body: bool| {
            let y = row as f32 * 10.0;
            let mut wall_entity = app
                .world_mut()
                .spawn((Transform::from_xyz(3.0, y, 0.0), wall()));
            if wall_is_body {
                wall_entity.insert(RigidBody::Fixed);
            }
            let wall_entity = wall_entity.id();
            let character = app
                .world_mut()
                .spawn((
                    Transform::from_xyz(0.0, y, 0.0),
                    Collider::ball(0.5),
                    KinematicCharacterController {
                        snap_to_ground: None,
                        ..Default::default()
                    },
                ))
                .id();
            (wall_entity, character)
        };
        let (_, blocked) = spawn_row(0, false);
        let (wall1, excluded_collider) = spawn_row(1, false);
        let (wall2, excluded_body) = spawn_row(2, true);
        let (wall3, predicate_filtered) = spawn_row(3, false);
        let (wall4, ignored_collider) = spawn_row(4, false);
        let (wall5, ignored_body) = spawn_row(5, true);

        // Let the colliders be initialized and inserted into the broad-phase.
        app.update();
        app.update();

        let world = app.world_mut();
        world
            .get_mut::<KinematicCharacterController>(excluded_collider)
            .unwrap()
            .exclude_colliders = EntityHashSet::from_iter([wall1]);
        world
            .get_mut::<KinematicCharacterController>(excluded_body)
            .unwrap()
            .exclude_rigid_bodies = EntityHashSet::from_iter([wall2]);
        world
            .get_mut::<KinematicCharacterController>(predicate_filtered)
            .unwrap()
            .filter_predicate = Some(ControllerFilterPredicate::new(move |e, _| e != wall3));
        world.entity_mut(wall4).insert(ControllerIgnored);
        world.entity_mut(wall5).insert(ControllerIgnored);
        for character in [
            blocked,
            excluded_collider,
            excluded_body,
            predicate_filtered,
            ignored_collider,
            ignored_body,
        ] {
            world
                .get_mut::<KinematicCharacterController>(character)
                .unwrap()
                .translation = Some(Vect::X * 6.0);
        }

        app.update();

        let x = |entity: Entity| app.world().get::<Transform>(entity).unwrap().translation.x;
        assert!(x(blocked) < 2.5, "The wall should block the character");
        for character in [
            excluded_collider,
            excluded_body,
            predicate_filtered,
            ignored_collider,
            ignored_body,
        ] {
            assert!(
                (x(character) - 6.0).abs() < 1.0e-3,
                "The character should pass through the excluded wall: {}",
                x(character)
            );
        }
    }
}
