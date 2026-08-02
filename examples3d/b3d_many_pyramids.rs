//! Port of box3d's `many_pyramids` benchmark (`CreateManyPyramids`,
//! `box3d/shared/benchmarks.c`). Release settings: a 14x14 grid of 10-base
//! pyramids of small cubes (density 100) on a ground box, sleeping disabled.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

fn create_small_pyramid(
    world: &mut PhysicsWorld,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_z: f32,
) {
    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            let body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y, base_z))
                .can_sleep(false);
            world.insert(
                body,
                ColliderBuilder::cuboid(extent, extent, extent).density(100.0),
            );
        }
    }
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    let base_count = 10i32;
    let extent = 0.5f32;
    let row_count = 14i32;
    let column_count = 14i32;
    let ground_extent = extent * column_count as f32 * (base_count as f32 + 1.0);

    // Ground.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(ground_extent, 1.0, ground_extent),
    );

    let base_width = 2.0 * extent * base_count as f32;
    let mut base_z = -ground_extent + 2.0 * extent;
    let delta_z = 2.0 * (ground_extent - 2.0 * extent) / (row_count as f32 - 1.0);

    for _ in 0..row_count {
        for j in 0..column_count {
            let center_x = -ground_extent + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(&mut world, base_count, extent, center_x, base_z);
        }
        base_z += delta_z;
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 30.0, 120.0), Vec3::new(0.0, 5.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
