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
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4);
    let mut ground_collider_handle =
        colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::dynamic().translation(vector![0.0, 0.2, 0.0]);
    let ball_handle = bodies.insert(rb);
    let collider = ColliderBuilder::ball(ball_rad).density(100.0);
    colliders.insert_with_parent(collider, ball_handle, &mut bodies);

    testbed.add_callback(move |_, physics, _, _| {
        // Remove then re-add the ground collider.
        let removed_collider_handle = ground_collider_handle;
        let coll = physics
            .colliders
            .remove(
                removed_collider_handle,
                &mut physics.islands,
                &mut physics.bodies,
                true,
            )
            .unwrap();
        ground_collider_handle =
            physics
                .colliders
                .insert_with_parent(coll, ground_handle, &mut physics.bodies);
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
