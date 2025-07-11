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

    /*
     * Ground
     */
    let width = 75.0;
    let thickness = 2.0;
    let ys = [-30.0 - thickness, 30.0 + thickness];

    for y in ys {
        let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, y]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(width, thickness);
        colliders.insert_with_parent(collider, handle, &mut bodies);
    }

    // Build two compression boxes rigid body.
    let half_height = (ys[1] - ys[0]) / 2.0 - thickness;
    let xs = [-width + thickness, width - thickness];
    let mut handles = [RigidBodyHandle::invalid(); 2];

    for i in 0..2 {
        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![xs[i], 0.0]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(thickness, half_height);
        colliders.insert_with_parent(collider, handle, &mut bodies);
        handles[i] = handle;
    }

    // Build the balls.
    let num = 8;
    let rad = half_height / (num as f32);
    for i in 0..num {
        for j in 0..num {
            let x = i as f32 * rad * 2.0 - num as f32 * rad;
            let y = j as f32 * rad * 2.0 - num as f32 * rad + rad;
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    let mut force = vector![0.0, 0.0];

    testbed.add_callback(move |_, physics, _, _| {
        let left_plank = &mut physics.bodies[handles[0]];
        left_plank.reset_forces(true);
        left_plank.add_force(force, true);

        let right_plank = &mut physics.bodies[handles[1]];
        right_plank.reset_forces(true);
        right_plank.add_force(-force, true);

        force.x += 10000.0;

        println!("force: {}", force.x);
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 50.0);
}
