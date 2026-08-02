//! Port of box2d's `many_pyramids` benchmark (`CreateManyPyramids`,
//! `box2d/shared/benchmarks.c`). Release: a 20x20 grid of 10-base pyramids on a
//! stack of segment "floors", sleeping disabled.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

fn create_small_pyramid(
    world: &mut PhysicsWorld,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_y: f32,
) {
    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent + base_y;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            let body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y))
                .can_sleep(false);
            world.insert(body, ColliderBuilder::cuboid(extent, extent));
        }
    }
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    let base_count = 10i32;
    let extent = 0.5f32;
    let row_count = 20i32;
    let column_count = 20i32;

    let ground_delta_y = 2.0 * extent * (base_count as f32 + 1.0);
    let ground_width = 2.0 * extent * column_count as f32 * (base_count as f32 + 1.0);

    // Ground: one static body carrying `row_count` horizontal segments.
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    let mut ground_y = 0.0f32;
    for _ in 0..row_count {
        world.insert_collider(
            ColliderBuilder::segment(
                Vector::new(-0.5 * ground_width, ground_y),
                Vector::new(0.5 * ground_width, ground_y),
            ),
            Some(ground),
        );
        ground_y += ground_delta_y;
    }

    let base_width = 2.0 * extent * base_count as f32;
    let mut base_y = 0.0f32;
    for _ in 0..row_count {
        for j in 0..column_count {
            let center_x =
                -0.5 * ground_width + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(&mut world, base_count, extent, center_x, base_y);
        }
        base_y += ground_delta_y;
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 60.0), 2.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
