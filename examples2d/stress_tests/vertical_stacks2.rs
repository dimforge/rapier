use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let num = 80;
    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = num as f32 * rad * 10.0;
    let ground_thickness = 1.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */

    let shiftx_centerx = [
        (rad * 2.0 + 0.0002, -(num as f32) * rad * 2.0 * 1.5),
        (rad * 2.0 + rad, num as f32 * rad * 2.0 * 1.5),
    ];

    for (shiftx, centerx) in shiftx_centerx {
        let shifty = rad * 2.0;
        let centery = shifty / 2.0 + ground_thickness;

        for i in 0..num {
            for j in 0usize..1 + i * 2 {
                let fj = j as f32;
                let fi = i as f32;
                let x = (fj - fi) * shiftx + centerx;
                let y = (num as f32 - fi - 1.0) * shifty + centery;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vec2::new(x, y));
                let collider = ColliderBuilder::cuboid(rad, rad);
                let _ = world.insert(rigid_body, collider);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 2.5), 5.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
