use rapier_testbed2d::Testbed;
use rapier2d::{na::UnitComplex, prelude::*};

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let rad = 0.5;

    let positions = [vector![5.0, -1.0], vector![-5.0, -1.0]];

    let platform_handles = positions
        .into_iter()
        .map(|pos| {
            let rigid_body = RigidBodyBuilder::kinematic_position_based().translation(pos);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
            colliders.insert_with_parent(collider, handle, &mut bodies);
            handle
        })
        .collect::<Vec<_>>();

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, _, state| {
        let rot = -state.time;
        for rb_handle in &platform_handles {
            let rb = physics.bodies.get_mut(*rb_handle).unwrap();
            rb.set_next_kinematic_rotation(UnitComplex::new(rot));
        }

        if state.timestep_id % 10 == 0 {
            let x = rand::random::<f32>() * 10.0 - 5.0;
            let y = rand::random::<f32>() * 10.0 + 10.0;
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let handle = physics.bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad);
            physics
                .colliders
                .insert_with_parent(collider, handle, &mut physics.bodies);

            if let Some(graphics) = &mut graphics {
                graphics.add_body(handle, &physics.bodies, &physics.colliders);
            }
        }

        let to_remove: Vec<_> = physics
            .bodies
            .iter()
            .filter(|(_, b)| b.position().translation.vector.y < -10.0)
            .map(|e| e.0)
            .collect();
        for handle in to_remove {
            physics.bodies.remove(
                handle,
                &mut physics.islands,
                &mut physics.colliders,
                &mut physics.impulse_joints,
                &mut physics.multibody_joints,
                true,
            );

            if let Some(graphics) = &mut graphics {
                graphics.remove_body(handle);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 20.0);
}
