#![allow(dead_code)]

mod boxes3;
mod debug_toggle3;
mod despawn3;
mod events3;
mod joints3;
mod joints_despawn3;
mod locked_rotations3;
mod multiple_colliders3;
mod picking3;
mod ray_casting3;
mod soft_bodies3;
mod static_trimesh3;
mod voxels3;

use bevy::{
    camera::visibility::RenderLayers,
    ecs::{resource::IsResource, world::error::EntityDespawnError},
    prelude::*,
};
use bevy_rapier3d::prelude::*;

#[derive(Debug, Reflect, Clone, Copy, Eq, PartialEq, Default, Hash, States)]
pub enum Examples {
    #[default]
    None,
    Boxes3,
    Voxels3,
    DebugToggle3,
    Despawn3,
    Events3,
    Joints3,
    JointsDespawn3,
    LockedRotations3,
    MultipleColliders3,
    Picking3,
    Raycasting3,
    StaticTrimesh3,
    SoftBodies3,
}

#[derive(Resource, Default, Reflect)]
struct ExamplesRes {
    entities_before: Vec<Entity>,
}

#[derive(Resource, Debug, Default, Reflect)]
struct ExampleSelected(pub usize);

#[derive(Debug, Reflect)]
struct ExampleDefinition {
    pub state: Examples,
    pub name: &'static str,
}

impl From<(Examples, &'static str)> for ExampleDefinition {
    fn from((state, name): (Examples, &'static str)) -> Self {
        Self { state, name }
    }
}

#[derive(Resource, Debug, Reflect)]
struct ExampleSet(pub Vec<ExampleDefinition>);

fn main() {
    let mut app = App::new();
    app.init_resource::<ExamplesRes>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_plugins(RapierPickingPlugin)
        .register_type::<Examples>()
        .register_type::<ExamplesRes>()
        .register_type::<ExampleSelected>()
        .init_state::<Examples>()
        .insert_resource(ExampleSet(vec![
            (Examples::Boxes3, "Boxes3").into(),
            (Examples::Voxels3, "Voxels3").into(),
            (Examples::DebugToggle3, "DebugToggle3").into(),
            (Examples::Despawn3, "Despawn3").into(),
            (Examples::Events3, "Events3").into(),
            (Examples::Joints3, "Joints3").into(),
            (Examples::JointsDespawn3, "JointsDespawn3").into(),
            (Examples::LockedRotations3, "LockedRotations3").into(),
            (Examples::MultipleColliders3, "MultipleColliders3").into(),
            (Examples::Picking3, "Picking3").into(),
            (Examples::Raycasting3, "Raycasting3").into(),
            (Examples::StaticTrimesh3, "StaticTrimesh3").into(),
            (Examples::SoftBodies3, "SoftBodies3").into(),
        ]))
        .init_resource::<ExampleSelected>()
        //
        // boxes3
        .add_systems(
            OnEnter(Examples::Boxes3),
            (boxes3::setup_graphics, boxes3::setup_physics),
        )
        .add_systems(OnExit(Examples::Boxes3), cleanup)
        //
        // voxels3
        .add_systems(
            OnEnter(Examples::Voxels3),
            (voxels3::setup_graphics, voxels3::setup_physics),
        )
        .add_systems(OnExit(Examples::Voxels3), cleanup)
        //
        // Debug toggle
        .add_systems(
            OnEnter(Examples::DebugToggle3),
            (debug_toggle3::setup_graphics, debug_toggle3::setup_physics),
        )
        .add_systems(
            Update,
            (
                debug_toggle3::toggle_debug,
                (|mut debug_render_context: ResMut<DebugRenderContext>| {
                    debug_render_context.enabled = !debug_render_context.enabled;
                })
                .run_if(debug_toggle3::input_just_pressed(KeyCode::KeyV)),
            )
                .run_if(in_state(Examples::DebugToggle3)),
        )
        .add_systems(OnExit(Examples::DebugToggle3), cleanup)
        //
        // despawn
        .init_resource::<despawn3::DespawnResource>()
        .add_systems(PreStartup, |mut commands: Commands| {
            commands.spawn((
                Camera2d,
                IsDefaultUiCamera,
                Camera {
                    order: 999,
                    ..Default::default()
                },
                RenderLayers::none(),
            ));
            commands.spawn((
                ExampleLabel,
                Text::default(),
                Node {
                    position_type: PositionType::Absolute,
                    top: px(8),
                    left: px(8),
                    ..default()
                },
            ));
        })
        .add_systems(
            OnEnter(Examples::Despawn3),
            (despawn3::setup_graphics, despawn3::setup_physics),
        )
        .add_systems(
            Update,
            despawn3::despawn.run_if(in_state(Examples::Despawn3)),
        )
        .add_systems(OnExit(Examples::Despawn3), cleanup)
        //
        // events
        .add_systems(
            OnEnter(Examples::Events3),
            (events3::setup_graphics, events3::setup_physics),
        )
        .add_systems(
            Update,
            events3::display_events.run_if(in_state(Examples::Events3)),
        )
        .add_systems(OnExit(Examples::Events3), cleanup)
        //
        // joints
        .add_systems(
            OnEnter(Examples::Joints3),
            (joints3::setup_graphics, joints3::setup_physics),
        )
        .add_systems(OnExit(Examples::Joints3), cleanup)
        //
        // joints despawn
        .init_resource::<joints_despawn3::DespawnResource>()
        .add_systems(
            OnEnter(Examples::JointsDespawn3),
            (
                joints_despawn3::setup_graphics,
                joints_despawn3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            joints_despawn3::despawn.run_if(in_state(Examples::JointsDespawn3)),
        )
        .add_systems(OnExit(Examples::JointsDespawn3), cleanup)
        //
        // locked rotations
        .add_systems(
            OnEnter(Examples::LockedRotations3),
            (
                locked_rotations3::setup_graphics,
                locked_rotations3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::LockedRotations3), cleanup)
        //
        // multiple colliders
        .add_systems(
            OnEnter(Examples::MultipleColliders3),
            (
                multiple_colliders3::setup_graphics,
                multiple_colliders3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::MultipleColliders3), cleanup)
        //
        // picking
        .add_systems(
            OnEnter(Examples::Picking3),
            (picking3::setup_graphics, picking3::setup_physics),
        )
        .add_systems(OnExit(Examples::Picking3), cleanup)
        //
        // raycasting
        .add_systems(
            OnEnter(Examples::Raycasting3),
            (ray_casting3::setup_graphics, ray_casting3::setup_physics),
        )
        .add_systems(
            Update,
            ray_casting3::cast_ray.run_if(in_state(Examples::Raycasting3)),
        )
        .add_systems(OnExit(Examples::Raycasting3), cleanup)
        //
        // static trimesh
        .init_resource::<static_trimesh3::BallState>()
        .add_systems(
            OnEnter(Examples::StaticTrimesh3),
            (
                static_trimesh3::setup_graphics,
                static_trimesh3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            static_trimesh3::ball_spawner.run_if(in_state(Examples::StaticTrimesh3)),
        )
        .add_systems(OnExit(Examples::StaticTrimesh3), cleanup)
        //
        // soft bodies
        .add_systems(
            OnEnter(Examples::SoftBodies3),
            (soft_bodies3::setup_graphics, soft_bodies3::setup_physics),
        )
        .add_systems(
            Update,
            (soft_bodies3::cut_cloth, soft_bodies3::log_tears)
                .run_if(in_state(Examples::SoftBodies3)),
        )
        .add_systems(OnExit(Examples::SoftBodies3), cleanup)
        //
        //testbed
        .add_systems(
            OnEnter(Examples::None),
            |mut next_state: ResMut<NextState<Examples>>| {
                next_state.set(Examples::Boxes3);
            },
        )
        .add_systems(OnExit(Examples::None), init)
        .add_systems(
            Update,
            (
                select_example,
                (change_example, update_example_label).run_if(resource_changed::<ExampleSelected>),
            )
                .chain(),
        );

    app.run();
}

fn init(world: &mut World) {
    // save all entities that are in the world before setting up any example
    // to be able to always return to this state when switching from one example to the other.
    // Resource-backed entities (carrying `IsResource`) are excluded: in Bevy 0.19 resources are
    // stored as entities, and despawning them in `cleanup` panics.
    world.resource_mut::<ExamplesRes>().entities_before = world
        .query_filtered::<Entity, Without<IsResource>>()
        .iter(world)
        .collect::<Vec<_>>();
}

fn cleanup(world: &mut World) {
    let keep_alive = world.resource::<ExamplesRes>().entities_before.clone();

    let remove = world
        .query_filtered::<Entity, Without<IsResource>>()
        .iter(world)
        .filter(|e| !keep_alive.contains(e))
        .collect::<Vec<_>>();
    for r in remove {
        // The entity may already have been despawned as part of a parent's despawn cascade;
        // skip it in that case to avoid a flood of spurious "invalid entity" warnings.
        if !world.entities().contains(r) {
            continue;
        }
        if let Err(error @ EntityDespawnError(_)) = world.try_despawn(r) {
            warn!("Cleanup error: {error:?}");
        }
    }
}

fn change_example(
    example_selected: Res<ExampleSelected>,
    examples_available: Res<ExampleSet>,
    mut next_state: ResMut<NextState<Examples>>,
) {
    next_state.set(examples_available.0[example_selected.0].state);
}

/// Marks the text node displaying the current example's name.
#[derive(Component)]
struct ExampleLabel;

fn select_example(
    keys: Res<ButtonInput<KeyCode>>,
    mut current_example: ResMut<ExampleSelected>,
    examples_available: Res<ExampleSet>,
) {
    let len = examples_available.0.len();
    if keys.just_pressed(KeyCode::PageDown) {
        current_example.0 = (current_example.0 + 1) % len;
    } else if keys.just_pressed(KeyCode::PageUp) {
        current_example.0 = (current_example.0 + len - 1) % len;
    }
}

fn update_example_label(
    current_example: Res<ExampleSelected>,
    examples_available: Res<ExampleSet>,
    mut label: Query<&mut Text, With<ExampleLabel>>,
) {
    for mut text in &mut label {
        text.0 = format!(
            "Example: {} (PageUp/PageDown to switch)",
            examples_available.0[current_example.0].name
        );
    }
}
