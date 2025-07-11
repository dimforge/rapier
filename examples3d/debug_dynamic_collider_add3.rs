use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground.
     */
    let ground_size = 20.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4)
        .friction(0.15)
        // .restitution(0.5)
        ;
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 0.2, 0.0])
        .linvel(vector![10.0, 0.0, 0.0]);
    let ball_handle = bodies.insert(rb);
    let collider = ColliderBuilder::ball(ball_rad).density(100.0);
    colliders.insert_with_parent(collider, ball_handle, &mut bodies);

    let mut linvel = Vector::zeros();
    let mut angvel = Vector::zeros();
    let mut pos = Isometry::identity();
    let mut step = 0;
    let mut extra_colliders = Vec::new();
    let snapped_frame = 51;

    testbed.add_callback(move |mut graphics, physics, _, _| {
        step += 1;

        // Add a bigger ball collider
        let collider = ColliderBuilder::ball(ball_rad + 0.01 * (step as f32)).density(100.0);
        let new_ball_collider_handle =
            physics
                .colliders
                .insert_with_parent(collider, ball_handle, &mut physics.bodies);

        if let Some(graphics) = &mut graphics {
            graphics.add_collider(new_ball_collider_handle, &physics.colliders);
        }

        extra_colliders.push(new_ball_collider_handle);

        // Snap the ball velocity or restore it.
        let ball = physics.bodies.get_mut(ball_handle).unwrap();

        if step == snapped_frame {
            linvel = *ball.linvel();
            angvel = *ball.angvel();
            pos = *ball.position();
        }

        if step == 100 {
            ball.set_linvel(linvel, true);
            ball.set_angvel(angvel, true);
            ball.set_position(pos, true);
            step = snapped_frame;

            for handle in &extra_colliders {
                if let Some(graphics) = &mut graphics {
                    graphics.remove_collider(*handle, &physics.colliders);
                }

                physics
                    .colliders
                    .remove(*handle, &mut physics.islands, &mut physics.bodies, true);
            }

            extra_colliders.clear();
        }

        // Remove then re-add the ground collider.
        // let ground = physics.bodies.get_mut(ground_handle).unwrap();
        // ground.set_position(Isometry::translation(0.0, step as f32 * 0.001, 0.0), false);
        // let coll = physics
        //     .colliders
        //     .remove(ground_collider_handle, &mut physics.bodies, true)
        //     .unwrap();
        let coll = ColliderBuilder::cuboid(ground_size, ground_height + step as f32 * 0.01, 0.4)
            .friction(0.15);
        let new_ground_collider_handle =
            physics
                .colliders
                .insert_with_parent(coll, ground_handle, &mut physics.bodies);

        if let Some(graphics) = &mut graphics {
            graphics.add_collider(new_ground_collider_handle, &physics.colliders);
        }

        extra_colliders.push(new_ground_collider_handle);
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
