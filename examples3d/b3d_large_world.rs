//! Port of box3d's `large_world` benchmark (`CreateLargeWorld` +
//! `StepLargeWorld`, `box3d/shared/benchmarks.c`). Release settings: a
//! 1000x1000 static box floor (one million static shapes) onto which 100
//! dynamic spheres are dropped, one every 5 steps.
//!
//! box3d creates one static *body* per floor box; rapier's idiomatic (and
//! perf-equivalent) static geometry is a parentless collider, so the floor is
//! built from one million standalone fixed colliders. This is a heavy scene —
//! expect a long build time and high memory use, matching the benchmark's
//! intent of stressing the broad-phase with a huge static set.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

const CELL_SIZE: f32 = 10.0;
const GRID: i32 = 1000;
const SPHERES: i32 = 100;
const DROP_INTERVAL: i32 = 5;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    let cell = CELL_SIZE;
    let half_span = 0.5 * cell * GRID as f32;

    for i in 0..GRID {
        let x = -half_span + (i as f32 + 0.5) * cell;
        for j in 0..GRID {
            let z = -half_span + (j as f32 + 0.5) * cell;
            world.insert_collider(
                ColliderBuilder::cuboid(0.5 * cell, 0.25, 0.5 * cell)
                    .translation(Vector::new(x, 0.0, z)),
                None,
            );
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 60.0, 250.0), Vec3::new(0.0, 0.0, 0.0));

    // `StepLargeWorld`: drop one sphere every `DROP_INTERVAL` steps, spread on a
    // coarse grid over the inner 80% of the floor, up to `SPHERES` total.
    let mut side = 1i32;
    while side * side < SPHERES {
        side += 1;
    }
    let mut step_count = 0i32;
    let mut dropped = 0i32;

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            if dropped < SPHERES && step_count > 0 && step_count % DROP_INTERVAL == 0 {
                let idx = dropped;
                let gi = idx % side;
                let gj = idx / side;
                let inset = 0.1 * 2.0 * half_span;
                let usable = 2.0 * half_span - 2.0 * inset;
                let x = -half_span + inset + (gi as f32 + 0.5) * (usable / side as f32);
                let z = -half_span + inset + (gj as f32 + 0.5) * (usable / side as f32);
                let (handle, _) = world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, 1.5, z)),
                    ColliderBuilder::ball(0.5),
                );
                // Register with the renderer (spawned after `set_world`).
                viewer.add_body(handle, &world);
                dropped += 1;
            }
            step_count += 1;
            world.step();
        }
    }
    Ok(())
}
