use bevy::prelude::*;
use bevy_rapier3d::loaders::meshloader::load_mesh_file_colliders;
use bevy_rapier3d::loaders::mjcf::{
    spawn_mjcf_model, MjcfActuator, MjcfLoaderOptions, MjcfPhysicsHooks, MjcfPlugin, MjcfRobot,
    MjcfSpawnOptions,
};
use bevy_rapier3d::loaders::urdf::{
    spawn_urdf_robot, UrdfLoaderOptions, UrdfModel, UrdfMultibodyOptions, UrdfSpawnOptions,
};
use bevy_rapier3d::prelude::*;
use std::f32::consts::FRAC_PI_2;

// The asset files are not shipped with this example: it only checks that the snippets compile.
fn main() {
    // DOCUSAURUS: MjcfPlugins start
    App::new()
        .add_plugins((
            DefaultPlugins,
            // The hooks apply the `<contact>` rules of the MJCF models.
            RapierPhysicsPlugin::<MjcfPhysicsHooks>::default(),
            // Applies the controls of the actuators to the joints they drive.
            MjcfPlugin::default(),
        ))
        // DOCUSAURUS: MjcfPlugins stop
        .add_systems(Startup, (urdf, mjcf, meshes, mesh_parts))
        .add_systems(Update, drive_actuators)
        .run();
}

fn urdf(mut commands: Commands) -> Result {
    // DOCUSAURUS: Urdf start
    // Read the robot, then spawn its links and joints as entities.
    let model = UrdfModel::from_file("robot.urdf", UrdfLoaderOptions::default(), None)?;
    let robot = spawn_urdf_robot(
        &mut commands,
        &model,
        &UrdfSpawnOptions {
            multibody: true,
            multibody_options: UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
            // URDF files are generally Z-up, whereas Bevy is Y-up.
            root_transform: Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
            ..default()
        },
    );
    println!("The robot has {} links.", robot.links.len());
    let _elbow = robot.joints_by_name["elbow"];
    // DOCUSAURUS: Urdf stop
    Ok(())
}

fn mjcf(
    mut commands: Commands,
    mut configurations: Query<&mut RapierConfiguration, With<DefaultRapierContext>>,
) -> Result {
    // DOCUSAURUS: Mjcf start
    // Read the model, then spawn its bodies, joints, and actuators as entities.
    let (robot, _model) = MjcfRobot::from_file("robot.xml", MjcfLoaderOptions::default())?;
    let model = spawn_mjcf_model(
        &mut commands,
        &robot,
        &MjcfSpawnOptions {
            // MJCF files are Z-up, whereas Bevy is Y-up.
            root_transform: Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
            ..default()
        },
    );
    // The gravity of the model isn't applied automatically.
    configurations.single_mut()?.gravity = model.gravity;
    // DOCUSAURUS: Mjcf stop
    Ok(())
}

// DOCUSAURUS: MjcfActuators start
fn drive_actuators(time: Res<Time>, mut actuators: Query<(&Name, &mut MjcfActuator)>) {
    for (name, mut actuator) in actuators.iter_mut() {
        if name.as_str() == "cart_motor" {
            actuator.ctrl = time.elapsed_secs().sin();
        }
    }
}
// DOCUSAURUS: MjcfActuators stop

fn meshes(mut commands: Commands) -> Result {
    // DOCUSAURUS: Meshes start
    // All the meshes of the file are combined into a single collider, each of them being
    // converted here into its convex hull.
    let collider = Collider::from_mesh_file("asset.obj", &MeshConverter::ConvexHull, Vec3::ONE)?;
    commands.spawn((Transform::default(), RigidBody::Dynamic, collider));
    // DOCUSAURUS: Meshes stop
    Ok(())
}

fn mesh_parts(mut commands: Commands) -> Result {
    // DOCUSAURUS: MeshParts start
    // Every mesh of the file becomes its own collider.
    let parts = load_mesh_file_colliders("asset.obj", &MeshConverter::TriMesh, Vec3::ONE)?;
    for part in parts {
        commands.spawn((part.transform, part.collider));
    }
    // DOCUSAURUS: MeshParts stop
    Ok(())
}
