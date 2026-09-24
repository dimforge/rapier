//! Breakable joints: a rope bridge made of planks linked by revolute impulse joints. Balls keep
//! falling on the bridge, and each joint is removed once the impulse it applies (read from the
//! `ImpulseJointImpulses` component) exceeds a threshold.
//!
//! Press R to rebuild the bridge.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// The number of planks of the bridge.
const NUM_PLANKS: usize = 20;
/// Half the width of a plank.
const PLANK_HALF_WIDTH: f32 = 0.3;
/// Half the thickness of a plank.
const PLANK_HALF_HEIGHT: f32 = 0.08;
/// A joint breaks when the norm of the linear impulse it applied during the last step exceeds this
/// value. The bridge alone needs about 0.3, so it breaks under heavy impacts or a pile of balls.
const BREAK_IMPULSE: f32 = 1.5;
/// The time between two ball drops, in seconds.
const DROP_PERIOD: f32 = 0.4;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(BallDropper {
            rng: oorandom::Rand32::new(42),
            timer: Timer::from_seconds(DROP_PERIOD, TimerMode::Repeating),
        })
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (
                drop_balls,
                break_joints,
                despawn_fallen_bodies,
                rebuild_bridge,
            ),
        )
        .run();
}

/// Marker for the joints that break under a large impulse.
#[derive(Component)]
struct Breakable;

/// Marker for the planks of the bridge, despawned when it is rebuilt.
#[derive(Component)]
struct Plank;

/// The state of the ball spawner.
#[derive(Resource)]
struct BallDropper {
    rng: oorandom::Rand32,
    timer: Timer,
}

/// The fixed pillars at both ends of the bridge.
#[derive(Resource)]
struct Pillars {
    left: Entity,
    right: Entity,
}

pub fn setup_graphics(mut commands: Commands) {
    // The scene is expressed in meters: zoom in so that one meter spans 50 pixels.
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 0.02,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 2.0, 0.0),
    ));
    commands.spawn((
        Text::new("R: rebuild the bridge"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        TextColor(Color::BLACK),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Two pillars holding both ends of the bridge.
     */
    let half_span = NUM_PLANKS as f32 * PLANK_HALF_WIDTH;
    let pillar_half_extents = Vec2::new(0.5, 3.0);
    let left = commands
        .spawn((
            Transform::from_xyz(
                -half_span - pillar_half_extents.x,
                -pillar_half_extents.y,
                0.0,
            ),
            RigidBody::Fixed,
            Collider::cuboid(pillar_half_extents.x, pillar_half_extents.y),
        ))
        .id();
    let right = commands
        .spawn((
            Transform::from_xyz(
                half_span + pillar_half_extents.x,
                -pillar_half_extents.y,
                0.0,
            ),
            RigidBody::Fixed,
            Collider::cuboid(pillar_half_extents.x, pillar_half_extents.y),
        ))
        .id();

    let pillars = Pillars { left, right };
    spawn_bridge(&mut commands, &pillars);
    commands.insert_resource(pillars);
}

/// Spawns the planks of the bridge, linked to each other and to the pillars with breakable joints.
fn spawn_bridge(commands: &mut Commands, pillars: &Pillars) {
    let half_span = NUM_PLANKS as f32 * PLANK_HALF_WIDTH;
    // The joint anchors on the top inner corners of the pillars.
    let pillar_corner = Vec2::new(0.5, 3.0);

    let mut prev = pillars.left;
    let mut prev_anchor = pillar_corner;

    for i in 0..NUM_PLANKS {
        let x = -half_span + (2.0 * i as f32 + 1.0) * PLANK_HALF_WIDTH;
        let plank = commands
            .spawn((
                Transform::from_xyz(x, 0.0, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(PLANK_HALF_WIDTH, PLANK_HALF_HEIGHT),
                Plank,
            ))
            .id();

        // Each joint lives on its own child entity, so that despawning it breaks only that link.
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(prev_anchor)
            .local_anchor2(Vec2::new(-PLANK_HALF_WIDTH, 0.0));
        commands
            .entity(plank)
            .with_child((ImpulseJoint::new(prev, joint), Breakable));

        if i == NUM_PLANKS - 1 {
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(Vec2::new(-pillar_corner.x, pillar_corner.y))
                .local_anchor2(Vec2::new(PLANK_HALF_WIDTH, 0.0));
            commands
                .entity(plank)
                .with_child((ImpulseJoint::new(pillars.right, joint), Breakable));
        }

        prev = plank;
        prev_anchor = Vec2::new(PLANK_HALF_WIDTH, 0.0);
    }
}

/// Drops a ball of random size and position above the bridge at regular intervals.
fn drop_balls(mut commands: Commands, time: Res<Time>, mut dropper: ResMut<BallDropper>) {
    if !dropper.timer.tick(time.delta()).just_finished() {
        return;
    }

    let half_span = NUM_PLANKS as f32 * PLANK_HALF_WIDTH;
    let x = (dropper.rng.rand_float() * 2.0 - 1.0) * half_span;
    let radius = 0.15 + 0.35 * dropper.rng.rand_float();
    commands.spawn((
        Transform::from_xyz(x, 8.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(radius),
        ColliderMassProperties::Density(3.0),
    ));
}

/// Removes the joints applying an impulse larger than the breaking threshold.
fn break_joints(
    mut commands: Commands,
    joints: Query<(Entity, &ImpulseJointImpulses), With<Breakable>>,
) {
    for (entity, impulses) in &joints {
        if impulses.linear.length() > BREAK_IMPULSE {
            commands.entity(entity).despawn();
        }
    }
}

/// Despawns the balls and planks that fell far below the bridge.
fn despawn_fallen_bodies(
    mut commands: Commands,
    bodies: Query<(Entity, &Transform), With<RigidBody>>,
) {
    for (entity, transform) in &bodies {
        if transform.translation.y < -20.0 {
            commands.entity(entity).despawn();
        }
    }
}

/// Replaces the bridge with a new one when R is pressed.
fn rebuild_bridge(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    pillars: Res<Pillars>,
    planks: Query<Entity, With<Plank>>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for plank in &planks {
            commands.entity(plank).despawn();
        }
        spawn_bridge(&mut commands, &pillars);
    }
}
