//! Port of box3d's `washer` benchmark (`CreateWasher`,
//! `box3d/shared/benchmarks.c`). Release settings: a spinning kinematic
//! "washer" (a ring built from ~40 convex hulls) tumbling a 20x20x20 grid of
//! small cubes.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;
use std::f32::consts::PI;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    // Ground.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(60.0, 1.0, 60.0),
    );

    // Spinning kinematic washer body (velocity-based: constant angular +
    // tiny linear velocity, matching box3d's kinematic branch).
    let motor_speed = 25.0f32;
    let washer = world.insert_body(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(0.0, 21.0, 0.0))
            .angvel(Vector::new(0.0, 0.0, (PI / 180.0) * motor_speed))
            .linvel(Vector::new(0.001, -0.002, 0.0)),
    );

    let r0 = 14.0f32;
    let r1 = 16.0f32;
    let r2 = 18.0f32;
    let neg_d = Vector::new(0.0, 0.0, -10.0);
    let pos_d = Vector::new(0.0, 0.0, 10.0);

    let angle = PI / 18.0;
    let q = Rotation::from_axis_angle(Vector::Z, angle);
    let qo = Rotation::from_axis_angle(Vector::Z, 0.1 * angle);
    let mut u1 = Vector::new(1.0, 0.0, 0.0);
    for i in 0..36 {
        let u2 = if i == 35 {
            Vector::new(1.0, 0.0, 0.0)
        } else {
            q * u1
        };

        {
            let a1 = qo.inverse() * u1;
            let a2 = qo * u2;
            let points = [
                neg_d + r1 * a1,
                neg_d + r2 * a1,
                neg_d + r1 * a2,
                neg_d + r2 * a2,
                pos_d + r1 * a1,
                pos_d + r2 * a1,
                pos_d + r1 * a2,
                pos_d + r2 * a2,
            ];
            world.insert_collider(ColliderBuilder::convex_hull(&points).unwrap(), Some(washer));
        }

        if i % 9 == 0 {
            let points = [
                neg_d + r0 * u1,
                neg_d + r1 * u1,
                neg_d + r0 * u2,
                neg_d + r1 * u2,
                pos_d + r0 * u1,
                pos_d + r1 * u1,
                pos_d + r0 * u2,
                pos_d + r1 * u2,
            ];
            world.insert_collider(ColliderBuilder::convex_hull(&points).unwrap(), Some(washer));
        }

        u1 = u2;
    }

    // Grid of small cubes.
    let grid_count = 20i32;
    let a = 0.2f32;
    let mut x = -2.0 * a * grid_count as f32;
    for _ in 0..grid_count {
        let mut y = -2.0 * a * grid_count as f32 + 21.0;
        for _ in 0..grid_count {
            let mut z = -2.0 * a * grid_count as f32;
            for _ in 0..grid_count {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z)),
                    ColliderBuilder::cuboid(a, a, a).density(1000.0),
                );
                z += 4.0 * a;
            }
            y += 4.0 * a;
        }
        x += 4.0 * a;
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(60.0, 35.0, 60.0), Vec3::new(0.0, 15.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
