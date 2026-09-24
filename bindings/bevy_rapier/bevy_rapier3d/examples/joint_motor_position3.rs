//! Revolute joint motors without gravity: the bottom row uses position motors reaching increasing
//! target angles, and the top row uses velocity motors stopped by increasing angular limits.
//!
//! The current and target angles of every joint are printed once per second.

use std::f32::consts::PI;

use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use bevy_rapier3d::prelude::*;
use std::time::Duration;

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
        .add_systems(
            Update,
            print_joint_angles.run_if(on_timer(Duration::from_secs(1))),
        )
        .run();
}

/// The angle a joint motor is expected to settle at.
#[derive(Component)]
struct TargetAngle(f32);

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 14.0).looking_at(Vec3::new(0.0, 3.5, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut rapier_config: Query<&mut RapierConfiguration>,
) -> Result<()> {
    rapier_config.single_mut()?.gravity = Vec3::ZERO;

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
        let target_angle = -PI + PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(Vec3::new(x_pos, 1.5, 0.0))
            .local_anchor2(Vec3::new(0.0, -0.5, 0.0))
            .motor_position(target_angle, 1000.0, 150.0);

        commands.spawn((
            Transform::from_xyz(x_pos, 2.0, 0.0),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::cuboid(0.1, 0.5, 0.1),
            ImpulseJoint::new(ground, joint),
            TargetAngle(target_angle),
        ));
    }

    /*
     * Rectangles on velocity motors, stopped by their limits.
     */
    for num in 0..8 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let max_angle_limit = -PI + PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(Vec3::new(x_pos, 5.0, 0.0))
            .local_anchor2(Vec3::new(0.0, -0.5, 0.0))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits([-PI, max_angle_limit]);

        commands.spawn((
            Transform::from_xyz(x_pos, 4.5, 0.0).with_rotation(Quat::from_rotation_z(PI)),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::cuboid(0.1, 0.5, 0.1),
            ImpulseJoint::new(ground, joint),
            TargetAngle(max_angle_limit),
        ));
    }

    Ok(())
}

fn print_joint_angles(
    context: ReadRapierContext,
    joints: Query<(Entity, &TargetAngle), With<ImpulseJoint>>,
) -> Result<()> {
    let context = context.single()?;
    for (entity, target) in &joints {
        if let Some(angle) = context.impulse_revolute_joint_angle(entity) {
            println!("{entity}: rev angle: {angle:.3} (target = {:.3})", target.0);
        }
    }

    Ok(())
}
