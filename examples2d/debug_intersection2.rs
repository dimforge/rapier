use kiss3d::color::{Color, GREY, RED};
use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let rad = 1.0;
    let collider = ColliderBuilder::ball(rad);

    let count = 100;
    for x in 0..count {
        for y in 0..count {
            let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(
                (x as f32 - count as f32 / 2.0) * rad * 3.0,
                (y as f32 - count as f32 / 2.0) * rad * 3.0,
            ));
            let (handle, _) = world.insert(rigid_body, collider.clone());
            viewer.set_initial_body_color(
                handle,
                Color::new(
                    x as f32 / count as f32,
                    (count - y) as f32 / count as f32,
                    0.5,
                    1.0,
                ),
            );
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::ZERO, 50.0);

    let mut step_id = 0usize;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            step_id += 1;

            let slow_time = step_id as f32 / 3.0;

            let query_pipeline = world.broad_phase.as_query_pipeline(
                world.narrow_phase.query_dispatcher(),
                &world.bodies,
                &world.colliders,
                QueryFilter::default(),
            );

            let ball = Ball::new(rad / 2.0);
            let intersections: Vec<_> = query_pipeline
                .intersect_shape(
                    Pose::from_translation(Vector::new(
                        slow_time.cos() * 10.0,
                        slow_time.sin() * 10.0,
                    )),
                    &ball,
                )
                .collect();

            for intersection in intersections {
                for (handle, _) in world.bodies.iter() {
                    viewer.set_body_color(handle, GREY, false);
                }

                let collider = world.colliders.get(intersection.0).unwrap();
                let body_handle = collider.parent().unwrap();

                viewer.set_body_color(body_handle, RED, false);
            }
        }
    }
    Ok(())
}
