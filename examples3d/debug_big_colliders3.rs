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
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let halfspace = SharedShape::new(HalfSpace::new(Vector::y_axis()));
    let collider = ColliderBuilder::new(halfspace);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let mut curr_y = 0.0;
    let mut curr_width = 10_000.0;

    for _ in 0..12 {
        let curr_height = 0.1f32.min(curr_width);
        curr_y += curr_height * 4.0;

        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![0.0, curr_y, 0.0]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(curr_width, curr_height, curr_width);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        curr_width /= 5.0;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
