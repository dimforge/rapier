use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn prismatic_repro(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    box_center: Point<f32>,
) {
    let box_rb = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![
        box_center.x,
        box_center.y,
        box_center.z
    ]));
    colliders.insert_with_parent(ColliderBuilder::cuboid(1.0, 0.25, 1.0), box_rb, bodies);

    let wheel_y = -1.0;
    let wheel_positions = vec![
        vector![1.0, wheel_y, -1.0],
        vector![-1.0, wheel_y, -1.0],
        vector![1.0, wheel_y, 1.0],
        vector![-1.0, wheel_y, 1.0],
    ];

    for pos in wheel_positions {
        let wheel_pos_in_world = box_center + pos;
        let wheel_rb = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![
            wheel_pos_in_world.x,
            wheel_pos_in_world.y,
            wheel_pos_in_world.z
        ]));
        colliders.insert_with_parent(ColliderBuilder::ball(0.5), wheel_rb, bodies);

        let (stiffness, damping) = (0.05, 0.2);

        let prismatic = PrismaticJointBuilder::new(Vector::y_axis())
            .local_anchor1(point![pos.x, pos.y, pos.z])
            .motor_position(0.0, stiffness, damping);
        impulse_joints.insert(box_rb, wheel_rb, prismatic, true);
    }

    // put a small box under one of the wheels
    let gravel = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![
        box_center.x + 1.0,
        box_center.y - 2.4,
        -1.0
    ]));
    colliders.insert_with_parent(ColliderBuilder::cuboid(0.5, 0.1, 0.5), gravel, bodies);
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    prismatic_repro(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        point![0.0, 5.0, 0.0],
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
