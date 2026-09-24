//! Gyroscopic forces toggled with the `GyroscopicForces` component, illustrated by the
//! Dzhanibekov effect (<https://en.wikipedia.org/wiki/Tennis_racket_theorem>).
//!
//! Both bodies spin around their unstable intermediate axis. The left one (with gyroscopic
//! forces) periodically flips, while the right one (without them) keeps spinning steadily.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

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
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 12.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    let bodies = [
        (
            -3.0,
            GyroscopicForces::enabled(),
            Hsla::hsl(220.0, 1.0, 0.3),
        ),
        (3.0, GyroscopicForces::disabled(), Hsla::hsl(20.0, 1.0, 0.5)),
    ];

    for (x, gyroscopic_forces, color) in bodies {
        let shapes = vec![
            (Vec3::ZERO, Quat::IDENTITY, Collider::cuboid(2.0, 0.2, 0.2)),
            (
                Vec3::new(0.0, 0.8, 0.0),
                Quat::IDENTITY,
                Collider::cuboid(0.2, 0.4, 0.2),
            ),
        ];

        commands.spawn((
            Transform::from_xyz(x, 0.0, 0.0),
            RigidBody::Dynamic,
            Collider::compound(shapes),
            GravityScale(0.0),
            Velocity::angular(Vec3::new(0.0, 20.0, 0.1)),
            gyroscopic_forces,
            ColliderDebugColor(color),
        ));
    }
}
