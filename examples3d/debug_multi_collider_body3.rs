//! Debug scene for https://github.com/dimforge/rapier/issues/970
//!
//! One dynamic body carrying 8000 colliders (a 20x20x20 block of unit boxes) dropped tilted
//! from 40m: the issue reported such a body keeping the step permanently slow, even though
//! same-parent colliders never collide. Watch the step cost collapse once it sleeps (~5.7s).

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// Collider grid on the single dynamic body: 20 x 20 x 20 = 8000 colliders.
const SZ: i32 = 20;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    /*
     * Ground
     */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.5, 100.0), None);

    /*
     * A single dynamic body with `SZ^3` colliders, dropped from 40m with a tilt so
     * it tumbles and lands on an edge before settling.
     */
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 40.0, 0.0))
            .rotation(Vector::new(1.0, 2.0, 3.0)),
    );

    for i in 0..SZ {
        for j in 0..SZ {
            for k in 0..SZ {
                world.insert_collider(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(
                        i as f32 - SZ as f32 / 2.0,
                        j as f32 - SZ as f32 / 2.0,
                        k as f32 - SZ as f32 / 2.0,
                    )),
                    Some(body),
                );
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(40.0, 30.0, 40.0), Vec3::ZERO);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
