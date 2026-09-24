//! Spawns the same URDF robot twice, with impulse joints and with multibody joints, and drives
//! their joints with motors.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::loaders::urdf::{
    spawn_urdf_robot, UrdfLoaderOptions, UrdfModel, UrdfMultibodyOptions, UrdfSpawnOptions,
};
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
        .add_systems(Update, drive_arms)
        .run();
}

/// The joints of a spawned arm driven by [`drive_arms`].
#[derive(Component)]
struct DrivenArm {
    base_yaw: Entity,
    shoulder: Entity,
    elbow: Entity,
    finger: Entity,
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 2.0, 4.0).looking_at(Vec3::new(0.0, 0.6, 0.0), Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(5.0, 0.1, 5.0),
    ));

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robots/arm.urdf");
    let model = UrdfModel::from_file(path, UrdfLoaderOptions::default(), None)
        .expect("Failed to load the URDF file.");

    for (x, multibody) in [(-0.8, false), (0.8, true)] {
        let robot = spawn_urdf_robot(
            &mut commands,
            &model,
            &UrdfSpawnOptions {
                multibody,
                multibody_options: UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
                // URDF files are Z-up.
                root_transform: Transform::from_xyz(x, 0.0, 0.0)
                    .with_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
                ..default()
            },
        );
        info!(
            "Spawned `{}` with {} links and {} joints ({}).",
            model.urdf.name,
            robot.links.len(),
            robot.joints.len(),
            if multibody { "multibody" } else { "impulse" }
        );
        commands.spawn(DrivenArm {
            base_yaw: robot.joints_by_name["base_yaw"],
            shoulder: robot.joints_by_name["shoulder"],
            elbow: robot.joints_by_name["elbow"],
            finger: robot.joints_by_name["left_finger_joint"],
        });
    }
}

/// Moves the joints of the arms with position motors.
fn drive_arms(
    time: Res<Time>,
    arms: Query<&DrivenArm>,
    mut impulse_joints: Query<&mut ImpulseJoint>,
    mut multibody_joints: Query<&mut MultibodyJoint>,
) {
    let t = time.elapsed_secs();
    for arm in arms.iter() {
        let targets = [
            (arm.base_yaw, JointAxis::AngX, t * 0.5, 30.0),
            (arm.shoulder, JointAxis::AngX, 0.6 * (t * 0.8).sin(), 100.0),
            (
                arm.elbow,
                JointAxis::AngX,
                1.0 + 0.6 * (t * 1.3).sin(),
                100.0,
            ),
            (arm.finger, JointAxis::LinX, 0.03 * (t * 2.0).sin(), 100.0),
        ];
        for (entity, axis, target, stiffness) in targets {
            let data = if let Ok(joint) = impulse_joints.get_mut(entity) {
                &mut joint.into_inner().data
            } else if let Ok(joint) = multibody_joints.get_mut(entity) {
                &mut joint.into_inner().data
            } else {
                continue;
            };
            data.as_mut()
                .set_motor_position(axis, target, stiffness, 0.2 * stiffness);
        }
    }
}
