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

    let radius = 0.5;
    let grid_count = 25;
    let friction = 0.6;
    let max_count = grid_count * grid_count;

    /*
     * Ground
     */
    let collider =
        ColliderBuilder::capsule_from_endpoints(point![-10.5, 0.0], point![10.5, 0.0], radius)
            .friction(friction);
    colliders.insert(collider);
    let collider =
        ColliderBuilder::capsule_from_endpoints(point![-10.5, 0.0], point![-10.5, 20.5], radius)
            .friction(friction);
    colliders.insert(collider);
    let collider =
        ColliderBuilder::capsule_from_endpoints(point![10.5, 0.0], point![10.5, 20.5], radius)
            .friction(friction);
    colliders.insert(collider);
    let collider =
        ColliderBuilder::capsule_from_endpoints(point![-10.5, 20.5], point![10.5, 20.5], radius)
            .friction(friction);
    colliders.insert(collider);

    /*
     * Create the spheres
     */
    let mut row;
    let mut count = 0;
    let mut column = 0;

    while count < max_count {
        row = 0;
        for _ in 0..grid_count {
            let x = -8.75 + column as f32 * 18.0 / (grid_count as f32);
            let y = 1.5 + row as f32 * 18.0 / (grid_count as f32);
            let body = RigidBodyBuilder::dynamic()
                .translation(vector![x, y])
                .gravity_scale(0.0);
            let body_handle = bodies.insert(body);
            let ball = ColliderBuilder::ball(radius).friction(friction);
            colliders.insert_with_parent(ball, body_handle, &mut bodies);

            count += 1;
            row += 1;
        }

        column += 1;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
