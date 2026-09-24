use rapier3d::prelude::*;
use rapier_testbed3d::{ExampleEntry, TestbedViewer};

// DOCUSAURUS: Scene start
async fn bouncing_ball(viewer: &mut TestbedViewer) {
    // The scene itself, built like in any other application.
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1, 100.0), None);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    // Hand the world to the viewer, and place the camera.
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);

    // The rendering loop: it ends when the user closes the window or selects another scene.
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
}
// DOCUSAURUS: Scene stop

// DOCUSAURUS: Main start
#[kiss3d::main]
async fn main() {
    // The scenes listed in the side panel of the testbed, as (group, name) pairs.
    let entries = vec![ExampleEntry::new("Demos", "Bouncing ball")];
    let mut viewer = TestbedViewer::new(entries).await;
    bouncing_ball(&mut viewer).await;
}
// DOCUSAURUS: Main stop
