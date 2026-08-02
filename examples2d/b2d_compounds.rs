//! Port of box2d's `compounds` benchmark (`CreateCompounds`,
//! `box2d/shared/benchmarks.c`, from the Barrel sample's compound branch).
//! Release: a 20x150 grid of two-triangle compound bodies dropped into a
//! walled bin.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

fn create_walls(world: &mut PhysicsWorld) {
    let grid = 1.0f32;
    let ground = world.insert_body(RigidBodyBuilder::fixed());

    let mut x = -40.0 * grid;
    for _ in 0..81 {
        world.insert_collider(
            ColliderBuilder::cuboid(0.55 * grid, 0.5 * grid).translation(Vector::new(x, 0.0)),
            Some(ground),
        );
        x += grid;
    }
    for wall_x in [-40.0 * grid, 40.0 * grid] {
        let mut y = grid;
        for _ in 0..100 {
            world.insert_collider(
                ColliderBuilder::cuboid(0.5 * grid, 0.55 * grid)
                    .translation(Vector::new(wall_x, y)),
                Some(ground),
            );
            y += grid;
        }
    }
    world.insert_collider(
        ColliderBuilder::segment(Vector::new(-800.0, -80.0), Vector::new(800.0, -80.0)),
        Some(ground),
    );
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);
    create_walls(&mut world);

    let column_count = 20i32;
    let row_count = 150i32;

    let left = [
        Vector::new(-1.0, 0.0),
        Vector::new(0.5, 1.0),
        Vector::new(0.0, 2.0),
    ];
    let right = [
        Vector::new(1.0, 0.0),
        Vector::new(-0.5, 1.0),
        Vector::new(0.0, 2.0),
    ];
    let left = SharedShape::convex_hull(&left).unwrap();
    let right = SharedShape::convex_hull(&right).unwrap();

    let shift = 2.0f32;
    let extray = 0.25f32;
    let mut side = 0.25f32;
    let centerx = shift * column_count as f32 / 2.0 - 1.0;
    let centery = 1.15 / 2.0;
    let y_start = 5.0f32;

    for i in 0..column_count {
        let x = i as f32 * shift - centerx;
        for j in 0..row_count {
            let y = j as f32 * (shift + extray) + centery + y_start;
            let handle = world
                .insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(x + side, y)));
            side = -side;
            world.insert_collider(
                ColliderBuilder::new(left.clone())
                    .density(1.0)
                    .friction(0.5),
                Some(handle),
            );
            world.insert_collider(
                ColliderBuilder::new(right.clone())
                    .density(1.0)
                    .friction(0.5),
                Some(handle),
            );
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 120.0), 2.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
