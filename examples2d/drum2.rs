use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Create the boxes
     */
    let num = 30;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift * num as f32 / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift - centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
            let collider = ColliderBuilder::cuboid(rad, rad);
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_velocity_based();
    let velocity_based_platform_handle = world.insert_body(platform_body);

    let sides = [
        (10.0, 0.25, Vector::new(0.0, 10.0)),
        (10.0, 0.25, Vector::new(0.0, -10.0)),
        (0.25, 10.0, Vector::new(10.0, 0.0)),
        (0.25, 10.0, Vector::new(-10.0, 0.0)),
    ];
    let balls = [
        (1.25, Vector::new(6.0, 6.0)),
        (1.25, Vector::new(-6.0, 6.0)),
        (1.25, Vector::new(6.0, -6.0)),
        (1.25, Vector::new(-6.0, -6.0)),
    ];

    for (hx, hy, pos) in sides {
        let collider = ColliderBuilder::cuboid(hx, hy).translation(pos);
        world.insert_collider(collider, Some(velocity_based_platform_handle));
    }
    for (r, pos) in balls {
        let collider = ColliderBuilder::ball(r).translation(pos);
        world.insert_collider(collider, Some(velocity_based_platform_handle));
    }

    /*
     * Run the simulation.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 1.0), 40.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            // Update the velocity-based kinematic body by setting its velocity.
            if let Some(platform) = world.bodies.get_mut(velocity_based_platform_handle) {
                platform.set_angvel(-0.15, true);
            }
        }
    }
    Ok(())
}
