//! Multibody pendulums, each configured with a different per-joint multibody option: default
//! joints, `MultibodyJointDamping`, `MultibodyJointFriction`, `MultibodyJointSprings`,
//! `MultibodyJointArmature`, and a `KinematicMultibodyJoint` spinning the first link at a constant
//! velocity.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// Half the length of each pendulum segment.
const HALF_LENGTH: f32 = 1.0;
/// The number of segments of each pendulum.
const SEGMENTS: usize = 4;
/// The angular velocity of the kinematic joints, in radians per second.
const KINEMATIC_SPEED: f32 = 1.0;

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
        .add_systems(Update, drive_kinematic_joints)
        .run();
}

/// The multibody joint option showcased by a pendulum.
#[derive(Copy, Clone)]
enum PendulumKind {
    Default,
    Damping,
    Friction,
    Springs,
    Armature,
    Kinematic,
}

pub fn setup_graphics(mut commands: Commands) {
    // The scene is expressed in meters: zoom in so that one meter spans 25 pixels.
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 0.04,
            ..OrthographicProjection::default_2d()
        }),
    ));
    commands.spawn((
        Text::new(
            "Top row: default joints, damping, friction\n\
             Bottom row: springs, armature, kinematic first joint",
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
    let kinds = [
        PendulumKind::Default,
        PendulumKind::Damping,
        PendulumKind::Friction,
        PendulumKind::Springs,
        PendulumKind::Armature,
        PendulumKind::Kinematic,
    ];

    /*
     * Grid layout. A chain is built towards +x and swings down, so each cell must be a full chain
     * long in both directions to keep neighbors from touching.
     */
    let chain_length = 2.0 * SEGMENTS as f32 * HALF_LENGTH;
    let spacing = chain_length + 4.0 * HALF_LENGTH;
    let cols = 3;
    let rows = kinds.len().div_ceil(cols);

    for (i, kind) in kinds.into_iter().enumerate() {
        let (col, row) = (i % cols, i / cols);
        let base_pos = Vec2::new(
            (col as f32 - (cols - 1) as f32 * 0.5) * spacing - chain_length * 0.5,
            ((rows - 1) as f32 * 0.5 - row as f32) * spacing + chain_length * 0.5,
        );
        create_pendulum(&mut commands, base_pos, kind);
    }
}

fn create_pendulum(commands: &mut Commands, base_pos: Vec2, kind: PendulumKind) {
    /*
     * A fixed base, with a small collider so the debug-renderer shows it.
     */
    let mut prev = commands
        .spawn((
            Transform::from_translation(base_pos.extend(0.0)),
            RigidBody::Fixed,
            Collider::ball(0.2 * HALF_LENGTH),
            ColliderDebugColor(Hsla::hsl(0.0, 0.0, 0.2)),
        ))
        .id();

    for i in 0..SEGMENTS {
        let center = base_pos + Vec2::X * (2.0 * i as f32 + 1.0) * HALF_LENGTH;
        let anchor1 = if i == 0 {
            Vec2::ZERO
        } else {
            Vec2::X * HALF_LENGTH
        };
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(anchor1)
            .local_anchor2(-Vec2::X * HALF_LENGTH)
            .contacts_enabled(false);

        let mut segment = commands.spawn((
            Transform::from_translation(center.extend(0.0)),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::capsule_x(HALF_LENGTH, 0.2 * HALF_LENGTH),
            MultibodyJoint::new(prev, joint),
        ));

        // In 2D, the only rotational degree of freedom of a joint is `JointAxis::AngX`.
        match kind {
            PendulumKind::Default => {}
            PendulumKind::Damping => {
                segment.insert(MultibodyJointDamping::default().with(JointAxis::AngX, 5.0));
            }
            PendulumKind::Friction => {
                // The largest torque the friction can apply to stop each joint.
                segment.insert(MultibodyJointFriction::default().with(JointAxis::AngX, 20.0));
            }
            PendulumKind::Springs => {
                // Pull each joint back towards its initial (straight) configuration.
                segment.insert(MultibodyJointSprings::default().with(JointAxis::AngX, 200.0, 0.0));
            }
            PendulumKind::Armature => {
                // Extra rotor inertia makes each joint react more slowly.
                segment.insert(MultibodyJointArmature::default().with(JointAxis::AngX, 20.0));
            }
            PendulumKind::Kinematic => {
                // Only the first joint is kinematic: it is driven by `drive_kinematic_joints`.
                if i == 0 {
                    segment.insert(KinematicMultibodyJoint);
                }
            }
        }

        prev = segment.id();
    }
}

/// Sets the velocity of the kinematic joints, which the physics engine never modifies.
fn drive_kinematic_joints(
    mut context: WriteRapierContext,
    joints: Query<Entity, With<KinematicMultibodyJoint>>,
) -> Result<()> {
    let mut context = context.single_mut()?;

    for entity in &joints {
        // This is `None` until the joint is inserted into the physics scene.
        if let Some(velocity) = context.multibody_joint_velocity_mut(entity) {
            velocity[0] = KINEMATIC_SPEED;
        }
    }

    Ok(())
}
