use rapier3d::prelude::*;
use rapier3d_meshloader::load_from_path;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};

fn main() {
    // The asset files are not shipped with this example: it only checks that the snippets compile.
    let _ = urdf();
    let _ = mjcf();
    let _ = meshes();
}

fn urdf() -> Result<(), Box<dyn std::error::Error>> {
    // DOCUSAURUS: Urdf start
    let mut world = PhysicsWorld::new();

    // Read the robot, then insert its links and joints into the world.
    let (robot, _) = UrdfRobot::from_file("robot.urdf", UrdfLoaderOptions::default(), None)?;
    let handles = robot.insert_using_multibody_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.multibody_joints,
        Default::default(),
    );
    println!("The robot has {} links.", handles.links.len());
    // DOCUSAURUS: Urdf stop
    Ok(())
}

fn mjcf() -> Result<(), Box<dyn std::error::Error>> {
    // DOCUSAURUS: Mjcf start
    let mut world = PhysicsWorld::new();

    // Read the model, then insert its bodies and joints into the world.
    let (robot, _model) = MjcfRobot::from_file("robot.xml", MjcfLoaderOptions::default())?;
    robot.insert_using_impulse_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.impulse_joints,
    );
    // DOCUSAURUS: Mjcf stop
    Ok(())
}

fn meshes() -> Result<(), Box<dyn std::error::Error>> {
    // DOCUSAURUS: Meshes start
    let mut world = PhysicsWorld::new();

    // Every mesh of the file becomes one shape, converted here into its convex hull.
    let shapes = load_from_path("asset.obj", &MeshConverter::ConvexHull, Vector::splat(1.0))?;
    for shape in shapes.into_iter().flatten() {
        world.insert_collider(ColliderBuilder::new(shape.shape).position(shape.pose), None);
    }
    // DOCUSAURUS: Meshes stop
    Ok(())
}
