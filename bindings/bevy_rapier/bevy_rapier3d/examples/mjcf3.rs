//! Spawns a MuJoCo MJCF scene (an actuated cart-pole, a double pendulum and a few free bodies)
//! with multibody joints, and drives its actuators.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::loaders::mjcf::{
    spawn_mjcf_model, MjcfActuator, MjcfLoaderOptions, MjcfPhysicsHooks, MjcfPlugin, MjcfRobot,
    MjcfSpawnOptions,
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
            // The hooks apply the `<contact>` rules of the model.
            RapierPhysicsPlugin::<MjcfPhysicsHooks>::default(),
            RapierDebugRenderPlugin::default(),
            // Applies the actuator controls to the joints.
            MjcfPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, drive_actuators)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.5, 1.5, 4.0).looking_at(Vec3::new(0.0, 1.0, -0.5), Vec3::Y),
    ));
}

fn setup_physics(
    mut commands: Commands,
    mut configuration: Query<&mut RapierConfiguration>,
) -> Result {
    // The model's floor plane is skipped by the loader, so add our own.
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(5.0, 0.1, 5.0),
    ));

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robots/cart_pole.xml");
    let (robot, _) = MjcfRobot::from_file(path, MjcfLoaderOptions::default())
        .expect("Failed to load the MJCF file.");
    let model = spawn_mjcf_model(
        &mut commands,
        &robot,
        &MjcfSpawnOptions {
            // MJCF files are Z-up.
            root_transform: Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
            ..default()
        },
    );
    info!(
        "Spawned `{}` with {} bodies, {} joints and {} actuators.",
        robot.name.as_deref().unwrap_or_default(),
        model.bodies.iter().flatten().count(),
        model.joints.iter().flatten().count(),
        model.actuators.len()
    );

    // Use the gravity declared by the model (expressed in the Y-up frame).
    configuration.single_mut()?.gravity = model.gravity;
    Ok(())
}

/// Moves the cart back and forth, and swings the elbow servo of the double pendulum.
fn drive_actuators(time: Res<Time>, mut actuators: Query<(&Name, &mut MjcfActuator)>) {
    let t = time.elapsed_secs();
    for (name, mut actuator) in actuators.iter_mut() {
        match name.as_str() {
            "cart_motor" => actuator.ctrl = (t * 1.5).sin(),
            "elbow_servo" => actuator.ctrl = 1.2 * (t * 0.7).sin(),
            _ => {}
        }
    }
}
