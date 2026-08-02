//! Port of box3d's `large_pyramid` benchmark (`CreateLargePyramid`,
//! `box3d/shared/benchmarks.c`). Release settings: a 90-box base pyramid of
//! unit cubes (density 100) on a large ground box, sleeping disabled.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    let base_count = 90i32;

    // Ground: b3MakeBoxHull(400, 1, 400) at y = -1.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(400.0, 1.0, 400.0),
    );

    let h = 0.5f32;
    let shift = 1.0 * h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * base_count as f32;
            let body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y, 0.0))
                .can_sleep(false);
            world.insert(body, ColliderBuilder::cuboid(h, h, h).density(100.0));
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 40.0, 110.0), Vec3::new(0.0, 20.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
