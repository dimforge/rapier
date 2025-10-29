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
    let _ground_size = 25.0;

    /*
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert_with_parent(co);
    */

    /*
     * Create the balls
     */
    let num = 50;
    let rad = 1.0;

    let shiftx = rad * 2.5;
    let shifty = rad * 2.0;
    let centerx = shiftx * (num as f32) / 2.0;
    let centery = shifty / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shiftx - centerx;
            let y = j as f32 * shifty + centery;

            let status = if j == 0 {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new(status).translation(vector![x, y]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 5.0);
}
