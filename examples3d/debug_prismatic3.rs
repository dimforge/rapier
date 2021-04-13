use na::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

fn prismatic_repro(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    box_center: Point3<f32>,
) {
    let box_rb = bodies.insert(
        RigidBodyBuilder::new_dynamic()
            .translation(box_center.x, box_center.y, box_center.z)
            .build(),
    );
    colliders.insert(
        ColliderBuilder::cuboid(1.0, 0.25, 1.0).build(),
        box_rb,
        bodies,
    );

    let wheel_y = -1.0;
    let wheel_positions = vec![
        Vector3::new(1.0, wheel_y, -1.0),
        Vector3::new(-1.0, wheel_y, -1.0),
        Vector3::new(1.0, wheel_y, 1.0),
        Vector3::new(-1.0, wheel_y, 1.0),
    ];

    for pos in wheel_positions {
        let wheel_pos_in_world = box_center + pos;
        let wheel_rb = bodies.insert(
            RigidBodyBuilder::new_dynamic()
                .translation(
                    wheel_pos_in_world.x,
                    wheel_pos_in_world.y,
                    wheel_pos_in_world.z,
                )
                .build(),
        );
        colliders.insert(ColliderBuilder::ball(0.5).build(), wheel_rb, bodies);

        let mut prismatic = rapier3d::dynamics::PrismaticJoint::new(
            Point3::new(pos.x, pos.y, pos.z),
            Vector3::y_axis(),
            Vector3::default(),
            Point3::new(0.0, 0.0, 0.0),
            Vector3::y_axis(),
            Vector3::default(),
        );
        prismatic.configure_motor_model(rapier3d::dynamics::SpringModel::VelocityBased);
        let (stiffness, damping) = (0.05, 0.2);
        prismatic.configure_motor_position(0.0, stiffness, damping);

        joints.insert(bodies, box_rb, wheel_rb, prismatic);
    }

    // put a small box under one of the wheels
    let gravel = bodies.insert(
        RigidBodyBuilder::new_dynamic()
            .translation(box_center.x + 1.0, box_center.y - 2.4, -1.0)
            .build(),
    );
    colliders.insert(
        ColliderBuilder::cuboid(0.5, 0.1, 0.5).build(),
        gravel,
        bodies,
    );
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    prismatic_repro(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(0.0, 5.0, 0.0),
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(10.0, 10.0, 10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
