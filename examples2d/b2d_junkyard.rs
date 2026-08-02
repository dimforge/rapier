//! Port of box2d's `junkyard` benchmark (`CreateJunkyard` + `StepJunkyard`,
//! `box2d/shared/benchmarks.c`). Release: a walled bin filled with 200x40
//! pentagon "rocks", stirred by a sweeping kinematic paddle.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;
use std::f32::consts::PI;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    // Walls: a floor of boxes plus two side columns.
    let grid = 1.0f32;
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    let mut x = -80.0 * grid;
    for _ in 0..161 {
        world.insert_collider(
            ColliderBuilder::cuboid(0.55 * grid, 0.5 * grid).translation(Vector::new(x, 0.0)),
            Some(ground),
        );
        x += grid;
    }
    for wall_x in [-80.0 * grid, 80.0 * grid] {
        let mut y = grid;
        for _ in 0..50 {
            world.insert_collider(
                ColliderBuilder::cuboid(0.5 * grid, 0.55 * grid)
                    .translation(Vector::new(wall_x, y)),
                Some(ground),
            );
            y += grid;
        }
    }

    // Pentagon "rocks" (one shared hull, matching box2d and enabling instancing).
    let radius = 0.25f32;
    let pentagon = SharedShape::convex_hull(&junkyard_pentagon(radius)).unwrap();
    let column_count = 200i32;
    let row_count = 40i32;
    let mut side = -0.1f32;
    let y_start = 15.0f32;
    for i in 0..column_count {
        let x = 1.5 * (2.0 * i as f32 - column_count as f32) * radius;
        for j in 0..row_count {
            let y = 4.0 * j as f32 * radius + y_start;
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(x + side, y)),
                ColliderBuilder::new(pentagon.clone()),
            );
            side = -side;
        }
    }

    // Sweeping kinematic paddle.
    let pusher = world.insert_body(RigidBodyBuilder::kinematic_position_based());
    world.insert_collider(
        ColliderBuilder::cuboid(2.0, 4.0).translation(Vector::new(0.0, 4.0)),
        Some(pusher),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 20.0), 4.0);

    let time_step = 1.0 / 60.0;
    let mut step_count = 0i32;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            let time = time_step * step_count as f32;
            let target = Vector::new(60.0 * (0.2 * time).sin(), 0.0);
            world.bodies[pusher].set_next_kinematic_translation(target);
            step_count += 1;
            world.step();
        }
    }
    Ok(())
}

/// box2d's junkyard "rock": a 5-point Fibonacci-lattice convex polygon of the
/// given `radius`.
fn junkyard_pentagon(radius: f32) -> Vec<Vector> {
    let phi = PI * (5.0f32.sqrt() - 1.0);
    (0..5)
        .map(|i| {
            let theta = phi * i as f32;
            Vector::new(radius * theta.cos(), radius * theta.sin())
        })
        .collect()
}
