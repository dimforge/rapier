//! Multibody joints in 3D: a grid of capsule chains built from spherical multibody joints (tied
//! together with impulse joints) falling on a tilted plate, and a row of revolute multibody chains
//! showcasing `MultibodyJointDamping`, `MultibodyJointFriction`, `MultibodyJointSprings`,
//! `MultibodyJointArmature` and a `KinematicMultibodyJoint` spinning its chain around the vertical
//! axis.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Half the length of the segments of the showcase chains.
const HALF_LENGTH: f32 = 0.5;
/// The number of segments of each showcase chain.
const SEGMENTS: usize = 4;
/// The angular velocity of the kinematic joints, in radians per second.
const KINEMATIC_SPEED: f32 = 2.0;

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

/// The multibody joint option showcased by a chain.
#[derive(Copy, Clone)]
enum ChainKind {
    Default,
    Damping,
    Friction,
    Springs,
    Armature,
    Kinematic,
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(15.0, 8.0, 42.0).looking_at(Vec3::new(13.0, 3.0, 1.0), Vec3::Y),
    ));
    commands.spawn((
        Text::new(
            "Upper chains, left to right: default joints, damping, friction, springs,\n\
             armature, kinematic first joint",
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
     * A dynamic plate resting on a fixed tilted ground.
     */
    let tilt = Quat::from_scaled_axis(Vec3::new(0.1, 0.0, 0.1));
    commands.spawn((
        Transform::from_xyz(0.0, -3.02, 0.0).with_rotation(tilt),
        Collider::cuboid(30.0, 0.01, 30.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, -3.0, 0.0).with_rotation(tilt),
        RigidBody::Dynamic,
        Collider::cuboid(30.0, 0.01, 30.0),
    ));

    create_ball_articulations(&mut commands, 8);

    /*
     * The showcase chains.
     */
    let kinds = [
        ChainKind::Default,
        ChainKind::Damping,
        ChainKind::Friction,
        ChainKind::Springs,
        ChainKind::Armature,
        ChainKind::Kinematic,
    ];
    for (i, kind) in kinds.into_iter().enumerate() {
        let base_pos = Vec3::new(-4.0 + 7.0 * i as f32, 12.0, 0.0);
        create_showcase_chain(&mut commands, base_pos, kind);
    }
}

/// A grid of capsules: each column is a multibody built with spherical joints, and neighboring
/// columns are tied together with spherical impulse joints.
fn create_ball_articulations(commands: &mut Commands, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let rigid_body = if i == 0 {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let mut body = commands.spawn((
                Transform::from_xyz(fk * shift, 0.0, fi * shift * 2.0),
                rigid_body,
                Collider::capsule_z(rad * 1.25, rad),
            ));

            // Vertical multibody joint.
            if i > 0 {
                let parent = *body_entities.last().unwrap();
                let joint =
                    SphericalJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift * 2.0));
                body.insert(MultibodyJoint::new(parent, joint));
            }

            // Horizontal impulse joint. It can live on the same entity as the multibody joint
            // since they are different components.
            if k > 0 && i > 0 {
                let parent = body_entities[body_entities.len() - num];
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                body.insert(ImpulseJoint::new(parent, joint));
            }

            body_entities.push(body.id());
        }
    }
}

/// A horizontal chain of revolute multibody joints, configured according to `kind`.
fn create_showcase_chain(commands: &mut Commands, base_pos: Vec3, kind: ChainKind) {
    let mut prev = commands
        .spawn((
            Transform::from_translation(base_pos),
            RigidBody::Fixed,
            Collider::ball(0.2),
            ColliderDebugColor(Hsla::hsl(0.0, 0.0, 0.2)),
        ))
        .id();

    for i in 0..SEGMENTS {
        let center = base_pos + Vec3::X * (2.0 * i as f32 + 1.0) * HALF_LENGTH;
        let anchor1 = if i == 0 {
            Vec3::ZERO
        } else {
            Vec3::X * HALF_LENGTH
        };
        // The kinematic chain spins around the vertical axis, the others swing around Z.
        let axis = if matches!(kind, ChainKind::Kinematic) && i == 0 {
            Vec3::Y
        } else {
            Vec3::Z
        };
        let joint = RevoluteJointBuilder::new(axis)
            .local_anchor1(anchor1)
            .local_anchor2(-Vec3::X * HALF_LENGTH)
            .contacts_enabled(false);

        let mut segment = commands.spawn((
            Transform::from_translation(center),
            RigidBody::Dynamic,
            Sleeping::disabled(),
            Collider::capsule_x(HALF_LENGTH, 0.1),
            MultibodyJoint::new(prev, joint),
        ));

        // The free rotational degree of freedom of a revolute joint is `JointAxis::AngX`, mapped
        // to the joint axis.
        match kind {
            ChainKind::Default => {}
            ChainKind::Damping => {
                segment.insert(MultibodyJointDamping::default().with(JointAxis::AngX, 0.3));
            }
            ChainKind::Friction => {
                // The largest torque the friction can apply to stop each joint.
                segment.insert(MultibodyJointFriction::default().with(JointAxis::AngX, 0.5));
            }
            ChainKind::Springs => {
                // Pull each joint back towards its initial (straight) configuration.
                segment.insert(MultibodyJointSprings::default().with(JointAxis::AngX, 3.0, 0.0));
            }
            ChainKind::Armature => {
                // Extra rotor inertia makes each joint react more slowly.
                segment.insert(MultibodyJointArmature::default().with(JointAxis::AngX, 1.0));
            }
            ChainKind::Kinematic => {
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
