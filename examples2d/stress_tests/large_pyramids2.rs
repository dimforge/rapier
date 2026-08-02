use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// A few *large* independent pyramids: 8 piles of 1540 boxes (base row of 55),
/// 12 320 dynamic bodies, sleeping disabled.
///
/// Unlike the many-small-pyramids scene, each pile here is large enough to
/// exercise intra-island parallelism: a few big piles stress how well the
/// staged solver distributes constraints within an island.
pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let num_pyramids = 8;
    let base_count = 55;
    let rad = 0.5;
    let gap = 10.0;

    /*
     * Ground
     */
    let pyramid_width = base_count as f32 * 2.0 * rad;
    let total_width = num_pyramids as f32 * (pyramid_width + gap);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vec2::new(0.0, -1.0));
    let collider = ColliderBuilder::cuboid(total_width, 1.0);
    let _ = world.insert(rigid_body, collider);

    /*
     * The pyramids.
     */
    let shift = rad * 2.0;

    for p in 0..num_pyramids {
        let x0 = p as f32 * (pyramid_width + gap) - 0.5 * total_width;

        for i in 0..base_count {
            for j in i..base_count {
                let x = x0 + (i as f32 * shift / 2.0) + (j - i) as f32 * shift;
                let y = i as f32 * shift * 1.001 + rad;

                let rigid_body = RigidBodyBuilder::dynamic()
                    .translation(Vec2::new(x, y))
                    .can_sleep(false);
                let collider = ColliderBuilder::cuboid(rad, rad);
                let _ = world.insert(rigid_body, collider);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 0.5 * base_count as f32 * shift), 3.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
