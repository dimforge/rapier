//! A kinematic character controlled with `KinematicCharacterController`, walking on slopes,
//! stairs, a heightfield, and a moving platform, and pushing dynamic boxes.
//!
//! It also showcases the controller's filters: the purple wall is ignored through
//! `exclude_colliders` (while still blocking the boxes), and holding Down lets the character drop
//! through the orange stairs thanks to a `filter_predicate`.
//!
//! Controls: Left/Right to move, Space to fly up, Down to move down, Shift to slow down.

use bevy::{ecs::entity::EntityHashSet, prelude::*};
use bevy_rapier2d::prelude::*;
use std::f32::consts::PI;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

const CHARACTER_SPEED: f32 = 6.0;
const GROUNDED_COLOR: Hsla = Hsla::hsl(120.0, 1.0, 0.35);
const AIRBORNE_COLOR: Hsla = Hsla::hsl(0.0, 1.0, 0.45);

/// Shared with the character controller's filter predicate: while set, the stairs are ignored.
#[derive(Resource, Clone, Default)]
struct DropThrough(Arc<AtomicBool>);

#[derive(Component)]
struct MovingPlatform;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .init_resource::<DropThrough>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (move_platform, move_character, update_character_color),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 50.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(1.0, 2.0, 0.0),
    ));
}

fn setup_physics(mut commands: Commands, drop_through: Res<DropThrough>) {
    /*
     * Ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = rad;

    for j in 0usize..4 {
        for i in 0..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;
            commands.spawn((
                Transform::from_xyz(x, y, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(rad, rad),
            ));
        }
    }

    /*
     * A wall ignored by the character (but not by the cubes).
     */
    let ghost_wall = commands
        .spawn((
            Transform::from_xyz(-1.5, 0.5, 0.0),
            Collider::cuboid(0.05, 0.5),
            ColliderDebugColor(Hsla::hsl(280.0, 0.8, 0.6)),
        ))
        .id();

    /*
     * Create some stairs, which the character can drop through.
     */
    let stair_width = 1.0;
    let stair_height = 0.1;
    let mut stairs = EntityHashSet::default();
    for i in 0..10 {
        let x = i as f32 * stair_width / 2.0;
        let y = i as f32 * stair_height * 1.5 + 3.0;

        let stair = commands
            .spawn((
                Transform::from_xyz(x, y, 0.0),
                Collider::cuboid(stair_width / 2.0, stair_height / 2.0),
                ColliderDebugColor(Hsla::hsl(30.0, 1.0, 0.5)),
            ))
            .id();
        stairs.insert(stair);
    }

    /*
     * Create a slope we can climb.
     */
    let slope_angle = 0.2;
    let slope_size = 2.0;
    commands.spawn((
        Transform::from_xyz(ground_size + slope_size, -ground_height + 0.4, 0.0)
            .with_rotation(Quat::from_rotation_z(slope_angle)),
        Collider::cuboid(slope_size, ground_height),
    ));

    /*
     * Create a slope we can't climb.
     */
    let impossible_slope_angle = 0.9;
    let impossible_slope_size = 2.0;
    commands.spawn((
        Transform::from_xyz(
            ground_size + slope_size * 2.0 + impossible_slope_size - 0.9,
            -ground_height + 2.3,
            0.0,
        )
        .with_rotation(Quat::from_rotation_z(impossible_slope_angle)),
        Collider::cuboid(slope_size, ground_height),
    ));

    /*
     * Create a wall we can't climb.
     */
    let wall_size = 2.0;
    let wall_pos = Vec3::new(
        ground_size + slope_size * 2.0 + impossible_slope_size + 0.35,
        -ground_height + 2.5 * 2.3,
        0.0,
    );
    commands.spawn((
        Transform::from_translation(wall_pos).with_rotation(Quat::from_rotation_z(PI / 2.0)),
        Collider::cuboid(wall_size, ground_height),
    ));
    commands.spawn((
        Transform::from_translation(wall_pos),
        Collider::cuboid(wall_size, ground_height),
    ));

    /*
     * Create a moving platform.
     */
    commands.spawn((
        Transform::from_xyz(-8.0, 0.0, 0.0),
        RigidBody::KinematicVelocityBased,
        Collider::cuboid(2.0, ground_height),
        Velocity::zero(),
        MovingPlatform,
    ));

    /*
     * More complex ground.
     */
    let heightfield_size = Vec2::new(10.0, 1.0);
    let nsubdivs = 20;
    let heights = (0..=nsubdivs)
        .map(|i| (i as f32 * heightfield_size.x / nsubdivs as f32 / 2.0).cos() * 1.5)
        .collect();
    commands.spawn((
        Transform::from_xyz(-8.0, 5.0, 0.0),
        Collider::heightfield(heights, heightfield_size),
    ));

    /*
     * A tilting dynamic body with a limited joint.
     */
    let anchor = commands
        .spawn((Transform::from_xyz(0.0, 5.0, 0.0), RigidBody::Fixed))
        .id();
    commands.spawn((
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 0.1),
        ImpulseJoint::new(anchor, RevoluteJointBuilder::new().limits([-0.3, 0.3])),
    ));

    /*
     * Character we will control manually.
     */
    let drop_through = drop_through.0.clone();
    commands.spawn((
        Transform::from_xyz(-3.0, 5.0, 0.0),
        RigidBody::KinematicPositionBased,
        Collider::capsule_y(0.3, 0.15),
        KinematicCharacterController {
            max_slope_climb_angle: impossible_slope_angle - 0.02,
            min_slope_slide_angle: impossible_slope_angle - 0.02,
            slide: true,
            exclude_colliders: EntityHashSet::from_iter([ghost_wall]),
            filter_predicate: Some(ControllerFilterPredicate::new(move |entity, _| {
                !(drop_through.load(Ordering::Relaxed) && stairs.contains(&entity))
            })),
            ..default()
        },
        ColliderDebugColor(AIRBORNE_COLOR),
    ));
}

fn move_platform(time: Res<Time>, mut platforms: Query<&mut Velocity, With<MovingPlatform>>) {
    let t = time.elapsed_secs();
    for mut velocity in &mut platforms {
        velocity.linear = Vec2::new((t * 2.0).sin() * 2.0, (t * 5.0).sin() * 1.5);
    }
}

fn move_character(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    drop_through: Res<DropThrough>,
    mut controllers: Query<&mut KinematicCharacterController>,
) {
    let mut speed = CHARACTER_SPEED;
    let mut desired_movement = Vec2::ZERO;

    if keyboard.pressed(KeyCode::ArrowRight) {
        desired_movement += Vec2::X;
    }
    if keyboard.pressed(KeyCode::ArrowLeft) {
        desired_movement -= Vec2::X;
    }
    if keyboard.pressed(KeyCode::Space) {
        desired_movement += Vec2::Y * 2.0;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        desired_movement -= Vec2::Y;
    }
    if keyboard.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
        speed /= 10.0;
    }

    // Artificial gravity.
    desired_movement -= Vec2::Y;

    drop_through
        .0
        .store(keyboard.pressed(KeyCode::ArrowDown), Ordering::Relaxed);

    for mut controller in &mut controllers {
        controller.translation = Some(desired_movement * speed * time.delta_secs());
    }
}

fn update_character_color(
    mut characters: Query<(&KinematicCharacterControllerOutput, &mut ColliderDebugColor)>,
) {
    for (output, mut color) in &mut characters {
        color.0 = if output.grounded {
            GROUNDED_COLOR
        } else {
            AIRBORNE_COLOR
        };
    }
}
