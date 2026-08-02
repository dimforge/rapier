//! Port of box2d's `washer` benchmark (`CreateWasher`,
//! `box2d/shared/benchmarks.c`). Release: a spinning kinematic ring (built from
//! ~40 convex quads) tumbling a 90x90 grid of small squares.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;
use std::f32::consts::PI;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    let _ground = world.insert_body(RigidBodyBuilder::fixed());

    // Spinning kinematic washer (constant angular + tiny linear velocity).
    let motor_speed = (PI / 180.0) * 25.0;
    let washer = world.insert_body(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(0.0, 10.0))
            .angvel(motor_speed)
            .linvel(Vector::new(0.001, -0.002)),
    );

    let (r0, r1, r2) = (14.0f32, 16.0f32, 18.0f32);
    let angle = PI / 18.0;
    let q = Rotation::new(angle);
    let qo = Rotation::new(0.1 * angle);
    let mut u1 = Vector::new(1.0, 0.0);
    for i in 0..36 {
        let u2 = if i == 35 {
            Vector::new(1.0, 0.0)
        } else {
            q * u1
        };

        let a1 = qo.inverse() * u1;
        let a2 = qo * u2;
        let seg = [r1 * a1, r2 * a1, r1 * a2, r2 * a2];
        world.insert_collider(ColliderBuilder::convex_hull(&seg).unwrap(), Some(washer));

        if i % 9 == 0 {
            let inner = [r0 * u1, r1 * u1, r0 * u2, r1 * u2];
            world.insert_collider(ColliderBuilder::convex_hull(&inner).unwrap(), Some(washer));
        }

        u1 = u2;
    }

    // Grid of small squares.
    let grid_count = 90i32;
    let a = 0.1f32;
    let mut y = -1.1 * a * grid_count as f32 + 10.0;
    for _ in 0..grid_count {
        let mut x = -1.1 * a * grid_count as f32;
        for _ in 0..grid_count {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(x, y)),
                ColliderBuilder::cuboid(a, a),
            );
            x += 2.1 * a;
        }
        y += 2.1 * a;
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 10.0), 12.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
