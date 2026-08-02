use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// A 20×20 grid of 10-base box
/// pyramids (22 000 dynamic bodies) resting on one segment per row, with
/// sleeping disabled. Many equally-sized independent piles is the worst case
/// for the solver's parallel load balancing.
pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let base_count = 10;
    let extent = 0.5;
    let row_count = 20;
    let column_count = 20;

    /*
     * Ground: one static body carrying one segment per pyramid row.
     */
    let ground_delta_y = 2.0 * extent * (base_count as f32 + 1.0);
    let ground_width = 2.0 * extent * column_count as f32 * (base_count as f32 + 1.0);

    let ground = world.insert_body(RigidBodyBuilder::fixed());
    for i in 0..row_count {
        let ground_y = i as f32 * ground_delta_y;
        let _ = world.insert_collider(
            ColliderBuilder::segment(
                Vec2::new(-0.5 * ground_width, ground_y),
                Vec2::new(0.5 * ground_width, ground_y),
            ),
            Some(ground),
        );
    }

    /*
     * The pyramids.
     */
    let base_width = 2.0 * extent * base_count as f32;

    for row in 0..row_count {
        let base_y = row as f32 * ground_delta_y;

        for column in 0..column_count {
            let center_x =
                -0.5 * ground_width + column as f32 * (base_width + 2.0 * extent) + 2.0 * extent;

            for i in 0..base_count {
                let y = (2.0 * i as f32 + 1.0) * extent + base_y;

                for j in i..base_count {
                    let x =
                        (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;

                    let rigid_body = RigidBodyBuilder::dynamic()
                        .translation(Vec2::new(x, y))
                        .can_sleep(false);
                    let collider = ColliderBuilder::cuboid(extent, extent);
                    let _ = world.insert(rigid_body, collider);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 0.5 * row_count as f32 * ground_delta_y), 3.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
