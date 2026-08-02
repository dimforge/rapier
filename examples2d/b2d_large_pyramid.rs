//! Port of box2d's `large_pyramid` benchmark (`CreateLargePyramid`,
//! `box2d/shared/benchmarks.c`). Release: a 200-box base pyramid of unit
//! squares on a ground box, sleeping disabled.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    let base_count = 200i32;

    // Ground: b2MakeBox(120, 1) at y = -1.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0)),
        ColliderBuilder::cuboid(120.0, 1.0),
    );

    let a = 0.5f32;
    let shift = a;
    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - a * base_count as f32;
            let body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y))
                .can_sleep(false);
            world.insert(body, ColliderBuilder::cuboid(a, a).density(1.0));
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 50.0), 2.5);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
