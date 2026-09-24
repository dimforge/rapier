//! Joint softness: chains of balls linked by spherical impulse joints, from the default (rigid)
//! softness on the left to increasingly soft joints on the right, which stretch like bungee cords.
//!
//! Press Space to toggle the `ImpulseJointDisabled` component on the joints attaching each chain
//! to its anchor: disabled joints are ignored by the solver, and re-enabled joints pull their chain
//! back up.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// The number of balls of each chain.
const NUM_BALLS: usize = 6;
/// The radius of each ball.
const BALL_RADIUS: f32 = 0.25;
/// The rest distance between two consecutive balls.
const LINK_LENGTH: f32 = 0.8;
/// The height of the fixed anchors of the chains.
const ANCHOR_HEIGHT: f32 = 8.0;

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
        .add_systems(Update, toggle_joints)
        .run();
}

/// Marker for the joints attaching the chains to their anchors, toggled with Space.
#[derive(Component)]
struct TopJoint;

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 16.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        Text::new(
            "Joint softness, left to right: default, 20 Hz, 12 Hz, 8 Hz, 6 Hz\n\
             Space: toggle the top joints",
        ),
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
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(10.0, 0.1, 3.0),
    ));

    /*
     * One chain per softness. The natural frequency of the joint constraints controls how fast
     * they resolve their errors: lower frequencies give softer joints.
     */
    let softnesses = [
        SpringCoefficients::joint_defaults(),
        SpringCoefficients::new(20.0, 1.0),
        SpringCoefficients::new(12.0, 1.0),
        SpringCoefficients::new(8.0, 1.0),
        SpringCoefficients::new(6.0, 1.0),
    ];

    for (k, softness) in softnesses.into_iter().enumerate() {
        let x = (k as f32 - (softnesses.len() - 1) as f32 * 0.5) * 3.0;
        let mut prev = commands
            .spawn((
                Transform::from_xyz(x, ANCHOR_HEIGHT, 0.0),
                RigidBody::Fixed,
                Collider::cuboid(0.2, 0.2, 0.2),
            ))
            .id();

        for i in 1..=NUM_BALLS {
            let joint = SphericalJointBuilder::new()
                .local_anchor1(Vec3::new(0.0, -LINK_LENGTH / 2.0, 0.0))
                .local_anchor2(Vec3::new(0.0, LINK_LENGTH / 2.0, 0.0))
                .softness(softness);

            let mut ball = commands.spawn((
                Transform::from_xyz(x, ANCHOR_HEIGHT - i as f32 * LINK_LENGTH, 0.0),
                RigidBody::Dynamic,
                Collider::ball(BALL_RADIUS),
                ImpulseJoint::new(prev, joint),
            ));

            if i == 1 {
                ball.insert(TopJoint);
            }

            prev = ball.id();
        }
    }
}

/// Toggles the `ImpulseJointDisabled` component of the top joints when Space is pressed.
fn toggle_joints(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    joints: Query<(Entity, Has<ImpulseJointDisabled>), With<TopJoint>>,
) {
    if !keys.just_pressed(KeyCode::Space) {
        return;
    }

    for (entity, disabled) in &joints {
        if disabled {
            commands.entity(entity).remove::<ImpulseJointDisabled>();
        } else {
            commands.entity(entity).insert(ImpulseJointDisabled);
        }
    }
}
