//! Spring joints with increasing damping: each ball hangs above a fixed anchor point with a spring
//! whose damping ratio goes from zero (left) to twice the critical damping (right), with the
//! middle one critically damped. Heavy boxes fall on the balls to excite the springs.

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

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(15.0, 5.0, 42.0).looking_at(Vec3::new(13.0, 1.0, 1.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Fixed ground to attach one end of the joints.
     */
    let ground = commands
        .spawn((Transform::default(), RigidBody::Fixed))
        .id();

    /*
     * Spring joints with a variety of damping ratios.
     */
    let num = 30;
    let radius = 0.5;
    // The mass of a ball with a density of 1.
    let mass = 4.0 / 3.0 * std::f32::consts::PI * radius * radius * radius;
    let stiffness = 1.0e3;
    let critical_damping = 2.0 * (stiffness * mass).sqrt();

    for i in 0..=num {
        let x_pos = -6.0 + 1.5 * i as f32;
        let ball_pos = Vec3::new(x_pos, 4.5, 0.0);
        let damping_ratio = i as f32 / (num as f32 / 2.0);
        let damping = damping_ratio * critical_damping;
        let joint = SpringJointBuilder::new(0.0, stiffness, damping)
            .local_anchor1(ball_pos - Vec3::Y * 3.0);

        commands.spawn((
            Transform::from_translation(ball_pos),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::ball(radius),
            ImpulseJoint::new(ground, joint),
        ));

        // A heavy box falling on top of the ball, to make the springs more interesting to watch.
        commands.spawn((
            Transform::from_translation(ball_pos + Vec3::Y * 5.0),
            RigidBody::Dynamic,
            Collider::cuboid(radius, radius, radius),
            ColliderMassProperties::Density(100.0),
        ));
    }
}
