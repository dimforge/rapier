use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

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
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
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
    let ball_coll_handle = colliders.insert_with_parent(collider, ball_handle, &mut bodies);

    /*
     * Colliders without bodies
     */
    let shape_size = 3.0;
    let static_collider =
        ColliderBuilder::ball(shape_size).translation(vector![-15.0, shape_size, 18.0]);
    colliders.insert(static_collider);

    let shapes = [
        SharedShape::ball(shape_size),
        SharedShape::cuboid(shape_size, shape_size, shape_size),
        SharedShape::cone(shape_size, shape_size),
        SharedShape::cylinder(shape_size, shape_size),
    ];
    let mut shape_idx = 0;
    let shapeshifting_collider = ColliderBuilder::new(shapes[shape_idx].clone())
        .translation(vector![-15.0, shape_size, 9.0]);
    let shapeshifting_coll_handle = colliders.insert(shapeshifting_collider);

    let mut linvel = Vector::zeros();
    let mut angvel = Vector::zeros();
    let mut pos = Isometry::identity();
    let mut step = 0;
    let snapped_frame = 51;

    testbed.add_callback(move |mut gfx, physics, _, _| {
        step += 1;

        // Snap the ball velocity or restore it.
        let ball = physics.bodies.get_mut(ball_handle).unwrap();

        if step == snapped_frame {
            linvel = *ball.linvel();
            angvel = *ball.angvel();
            pos = *ball.position();
        }

        let shapeshifting_coll = physics
            .colliders
            .get_mut(shapeshifting_coll_handle)
            .unwrap();
        if step % 50 == 0 {
            shape_idx = (shape_idx + 1) % 4;
            shapeshifting_coll.set_shape(shapes[shape_idx].clone())
        }

        if step == 100 {
            ball.set_linvel(linvel, true);
            ball.set_angvel(angvel, true);
            ball.set_position(pos, true);
            step = snapped_frame;
        }

        let ball_coll = physics.colliders.get_mut(ball_coll_handle).unwrap();
        ball_coll.set_shape(SharedShape::ball(ball_rad * step as f32 * 2.0));

        if let Some(gfx) = &mut gfx {
            gfx.update_collider(ball_coll_handle, &physics.colliders);
            gfx.update_collider(shapeshifting_coll_handle, &physics.colliders);
        }
    });

    /*
     * Create the primitives
     */
    let num = 8;
    let rad = 1.0;

    let shiftx = rad * 2.0 + rad;
    let shifty = rad * 2.0 + rad;
    let shiftz = rad * 2.0 + rad;
    let centerx = shiftx * (num / 2) as f32;
    let centery = shifty / 2.0;
    let centerz = shiftz * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shiftx - centerx + offset + 5.0;
                let y = j as f32 * shifty + centery + 3.0;
                let z = k as f32 * shiftz - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(rigid_body);

                let collider = match j % 5 {
                    0 => ColliderBuilder::cuboid(rad, rad, rad),
                    1 => ColliderBuilder::ball(rad),
                    // Rounded cylinders are much more efficient that cylinder, even if the
                    // rounding margin is small.
                    2 => ColliderBuilder::round_cylinder(rad, rad, rad / 10.0),
                    3 => ColliderBuilder::cone(rad, rad),
                    _ => ColliderBuilder::capsule_y(rad, rad),
                };

                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![40.0, 40.0, 40.0], Point::origin());
}
