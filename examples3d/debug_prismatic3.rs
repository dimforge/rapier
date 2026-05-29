use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn prismatic_repro(world: &mut PhysicsWorld, box_center: Vector) {
    let (box_rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(box_center),
        ColliderBuilder::cuboid(1.0, 0.25, 1.0),
    );

    let wheel_y = -1.0;
    let wheel_positions = vec![
        Vector::new(1.0, wheel_y, -1.0),
        Vector::new(-1.0, wheel_y, -1.0),
        Vector::new(1.0, wheel_y, 1.0),
        Vector::new(-1.0, wheel_y, 1.0),
    ];

    for pos in wheel_positions {
        let wheel_pos_in_world = box_center + pos;
        let (wheel_rb, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(wheel_pos_in_world),
            ColliderBuilder::ball(0.5),
        );

        let (stiffness, damping) = (0.05, 0.2);

        let prismatic = PrismaticJointBuilder::new(Vector::Y)
            .local_anchor1(pos)
            .motor_position(0.0, stiffness, damping);
        world.insert_impulse_joint(box_rb, wheel_rb, prismatic);
    }

    // put a small box under one of the wheels
    let (_gravel, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(
            box_center.x + 1.0,
            box_center.y - 2.4,
            -1.0,
        )),
        ColliderBuilder::cuboid(0.5, 0.1, 0.5),
    );
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

    prismatic_repro(&mut world, Vector::new(0.0, 5.0, 0.0));

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
