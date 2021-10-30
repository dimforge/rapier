use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(vector![0.0, -ground_height, 0.0])
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Cube 1
     */
    let rb = RigidBodyBuilder::new_dynamic()
        .translation(vector![0.0, 3.0, 2.0])
        .build();
    let cube1 = bodies.insert(rb);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5).build();
    colliders.insert_with_parent(collider, cube1, &mut bodies);

    /*
     * Cube 2
     */
    let rb = RigidBodyBuilder::new_dynamic()
        .translation(vector![0.0, 3.0, -2.0])
        .build();
    let cube2 = bodies.insert(rb);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5).build();
    colliders.insert_with_parent(collider, cube2, &mut bodies);

    /*
     * Spring Joint
     */
    let mut spring = SpringJoint::new(
        point![0.0, 0.0, 0.0], 
        point![0.0, 0.0, 0.0], 
        6.0,
        25.0,
        0.0,
    );
    spring.limits_enabled = true;
    spring.limits_max_length = 7.0;
    spring.limits_min_length = 4.5;
    let _spring = joints.insert(cube1, cube2, spring);

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(bodies, colliders, joints, Vector::zeros(), ());
    testbed.look_at(point![10.0, 3.0, 0.0], point![0.0, 3.0, 0.0]);
}
