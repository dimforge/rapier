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
     * Create the boxes
     */
    let num = 30;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift * num as f32 / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift - centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_velocity_based();
    let velocity_based_platform_handle = bodies.insert(platform_body);

    let sides = [
        (10.0, 0.25, vector![0.0, 10.0]),
        (10.0, 0.25, vector![0.0, -10.0]),
        (0.25, 10.0, vector![10.0, 0.0]),
        (0.25, 10.0, vector![-10.0, 0.0]),
    ];
    let balls = [
        (1.25, vector![6.0, 6.0]),
        (1.25, vector![-6.0, 6.0]),
        (1.25, vector![6.0, -6.0]),
        (1.25, vector![-6.0, -6.0]),
    ];

    for (hx, hy, pos) in sides {
        let collider = ColliderBuilder::cuboid(hx, hy).translation(pos);
        colliders.insert_with_parent(collider, velocity_based_platform_handle, &mut bodies);
    }
    for (r, pos) in balls {
        let collider = ColliderBuilder::ball(r).translation(pos);
        colliders.insert_with_parent(collider, velocity_based_platform_handle, &mut bodies);
    }

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, physics, _, _| {
        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(velocity_based_platform_handle) {
            platform.set_angvel(-0.15, true);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 1.0], 40.0);
}
