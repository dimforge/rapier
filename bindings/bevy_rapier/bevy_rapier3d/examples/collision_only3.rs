//! Collision detection without dynamics using `SimulationMode::CollisionOnly`.
//!
//! Two kinematic bodies, a ball and a cube, are driven through their `Transform`. They report
//! collision events when they touch the fixed obstacles, which turn red while touched. Nothing
//! is pushed or stopped.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// A kinematic body following a path parameterized by the elapsed time.
#[derive(Component)]
struct Mover(fn(f32) -> Vec3);

/// The number of colliders currently touching a fixed obstacle.
#[derive(Component, Default)]
struct Touching(usize);

const IDLE_COLOR: Hsla = Hsla::hsl(220.0, 1.0, 0.3);
const TOUCHED_COLOR: Hsla = Hsla::hsl(0.0, 1.0, 0.5);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, move_bodies)
        .add_systems(PostUpdate, handle_events.after(PhysicsSet::Writeback))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 12.0, 12.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(
    mut commands: Commands,
    mut config: Query<&mut RapierConfiguration>,
) -> Result<()> {
    config.single_mut()?.simulation_mode = SimulationMode::CollisionOnly;

    /*
     * Fixed obstacles: three on the path of the ball, one on the path of the cube.
     */
    let obstacles = [
        (Vec3::new(-5.0, 0.0, 0.0), Collider::cuboid(1.0, 1.0, 1.0)),
        (Vec3::new(5.0, 0.0, 0.0), Collider::ball(1.2)),
        (Vec3::new(0.0, 0.0, -5.0), Collider::cuboid(1.5, 1.0, 0.1)),
        (Vec3::new(0.0, 0.0, 1.5), Collider::cuboid(1.0, 1.0, 0.5)),
    ];

    for (pos, collider) in obstacles {
        commands.spawn((
            Transform::from_translation(pos),
            collider,
            ActiveEvents::COLLISION_EVENTS,
            ColliderDebugColor(IDLE_COLOR),
            Touching::default(),
        ));
    }

    /*
     * The movers. Kinematic-fixed pairs are ignored by default, so they are enabled with
     * `ActiveCollisionTypes`.
     */
    let ball_path = Mover(|t| Vec3::new(5.0 * (t * 0.8).cos(), 0.0, 5.0 * (t * 0.8).sin()));
    let cube_path = Mover(|t| Vec3::new(0.0, 0.0, 3.0 * (t * 1.2).sin()));
    let movers = [
        (ball_path, Collider::ball(0.5)),
        (cube_path, Collider::cuboid(0.5, 0.5, 0.5)),
    ];

    for (path, collider) in movers {
        commands.spawn((
            Transform::from_translation(path.0(0.0)),
            RigidBody::KinematicPositionBased,
            collider,
            ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_STATIC,
            path,
        ));
    }

    Ok(())
}

fn move_bodies(time: Res<Time>, mut movers: Query<(&Mover, &mut Transform)>) {
    for (mover, mut transform) in &mut movers {
        transform.translation = mover.0(time.elapsed_secs());
    }
}

fn handle_events(
    mut collision_events: MessageReader<CollisionEvent>,
    mut obstacles: Query<(&mut Touching, &mut ColliderDebugColor)>,
) {
    for event in collision_events.read() {
        info!("Received collision event: {event:?}");

        let (e1, e2, delta) = match *event {
            CollisionEvent::Started(e1, e2, _) => (e1, e2, 1),
            CollisionEvent::Stopped(e1, e2, _) => (e1, e2, -1),
        };

        for entity in [e1, e2] {
            if let Ok((mut touching, mut color)) = obstacles.get_mut(entity) {
                touching.0 = touching.0.saturating_add_signed(delta);
                color.0 = if touching.0 > 0 {
                    TOUCHED_COLOR
                } else {
                    IDLE_COLOR
                };
            }
        }
    }
}
