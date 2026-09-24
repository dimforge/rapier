//! Revolute joint motors without gravity: the bottom row uses position motors reaching increasing
//! target angles, and the top row uses velocity motors stopped by increasing angular limits.

use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

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

pub fn setup_graphics(mut commands: Commands) {
    // The scene is expressed in meters: zoom in so that one meter spans 40 pixels.
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 40.0,
            ..OrthographicProjection::default_2d()
        }),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut rapier_config: Query<&mut RapierConfiguration>,
) -> Result<()> {
    rapier_config.single_mut()?.gravity = Vec2::ZERO;

    /*
     * Fixed ground to attach one end of the joints.
     */
    let ground = commands
        .spawn((Transform::default(), RigidBody::Fixed))
        .id();

    /*
     * Rectangles on motors with a target position.
     */
    for num in 0..9 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vec2::new(x_pos, 1.5))
            .local_anchor2(Vec2::new(0.0, -0.5))
            .motor_position(-PI + PI / 4.0 * num as f32, 1000.0, 150.0);

        commands.spawn((
            Transform::from_xyz(x_pos, 2.0, 0.0),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::cuboid(0.1, 0.5),
            ImpulseJoint::new(ground, joint),
        ));
    }

    /*
     * Rectangles on velocity motors, stopped by their limits.
     */
    for num in 0..8 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vec2::new(x_pos, 5.0))
            .local_anchor2(Vec2::new(0.0, -0.5))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits([-PI, -PI + PI / 4.0 * num as f32]);

        commands.spawn((
            Transform::from_xyz(x_pos, 4.5, 0.0).with_rotation(Quat::from_rotation_z(PI)),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::cuboid(0.1, 0.5),
            ImpulseJoint::new(ground, joint),
        ));
    }

    Ok(())
}
