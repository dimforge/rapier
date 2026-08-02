//! Port of box2d's `smash` benchmark (`CreateSmash`,
//! `box2d/shared/benchmarks.c`). Release: zero gravity; a heavy 8x8 box flung
//! at 40 m/s into a 120x80 grid of small squares that start asleep.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);
    world.gravity = Vector::ZERO;

    // The smasher.
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-20.0, 0.0))
            .linvel(Vector::new(40.0, 0.0)),
        ColliderBuilder::cuboid(4.0, 4.0).density(8.0),
    );

    // The wall of small squares (start asleep, box2d `isAwake = false`).
    let d = 0.4f32;
    let columns = 120i32;
    let rows = 80i32;
    for i in 0..columns {
        for j in 0..rows {
            let x = i as f32 * d + 30.0;
            let y = (j as f32 - rows as f32 / 2.0) * d;
            world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x, y))
                    .sleeping(true),
                ColliderBuilder::cuboid(0.5 * d, 0.5 * d),
            );
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(20.0, 0.0), 8.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
