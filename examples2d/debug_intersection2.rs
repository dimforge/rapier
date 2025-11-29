use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let rad = 1.0;
    let collider = ColliderBuilder::ball(rad);

    let count = 100;
    for x in 0..count {
        for y in 0..count {
            let rigid_body = RigidBodyBuilder::fixed().translation(vector![
                (x as f32 - count as f32 / 2.0) * rad * 3.0,
                (y as f32 - count as f32 / 2.0) * rad * 3.0
            ]);
            let handle = bodies.insert(rigid_body);
            colliders.insert_with_parent(collider.clone(), handle, &mut bodies);
            testbed.set_initial_body_color(
                handle,
                [
                    x as f32 / count as f32,
                    (count - y) as f32 / count as f32,
                    0.5,
                ],
            );
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 50.0);

    testbed.add_callback(move |mut graphics, physics, _, run| {
        let slow_time = run.timestep_id as f32 / 3.0;

        let query_pipeline = physics.broad_phase.as_query_pipeline(
            physics.narrow_phase.query_dispatcher(),
            &physics.bodies,
            &physics.colliders,
            QueryFilter::default(),
        );

        for intersection in query_pipeline.intersect_shape(
            Isometry::translation(slow_time.cos() * 10.0, slow_time.sin() * 10.0),
            &Ball::new(rad / 2.0),
        ) {
            if let Some(graphics) = graphics.as_deref_mut() {
                for (handle, _) in physics.bodies.iter() {
                    graphics.set_body_color(handle, [0.5, 0.5, 0.5]);
                }

                let collider = physics.colliders.get(intersection.0).unwrap();
                let body_handle = collider.parent().unwrap();

                graphics.set_body_color(body_handle, [1.0, 0.0, 0.0]);
            }
        }
    });
}
