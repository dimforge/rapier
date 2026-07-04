use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0));
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    let (_handle, _) = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
