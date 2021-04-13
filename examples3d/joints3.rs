use na::{Isometry3, Point3, Unit, UnitQuaternion, Vector3};
use rapier3d::dynamics::{
    BallJoint, BodyStatus, FixedJoint, JointSet, PrismaticJoint, RevoluteJoint, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

fn create_prismatic_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::new_static()
        .translation(origin.x, origin.y, origin.z)
        .build();
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
    colliders.insert(collider, curr_parent, bodies);

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(origin.x, origin.y, z)
            .build();
        let curr_child = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
        colliders.insert(collider, curr_child, bodies);

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let z = Vector3::z();
        let mut prism = PrismaticJoint::new(
            Point3::new(0.0, 0.0, 0.0),
            axis,
            z,
            Point3::new(0.0, 0.0, -shift),
            axis,
            z,
        );
        prism.limits_enabled = true;
        prism.limits[0] = -2.0;
        prism.limits[1] = 2.0;

        joints.insert(bodies, curr_parent, curr_child, prism);

        curr_parent = curr_child;
    }
}

fn create_actuated_prismatic_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::new_static()
        .translation(origin.x, origin.y, origin.z)
        .build();
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
    colliders.insert(collider, curr_parent, bodies);

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(origin.x, origin.y, z)
            .build();
        let curr_child = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
        colliders.insert(collider, curr_child, bodies);

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let z = Vector3::z();
        let mut prism = PrismaticJoint::new(
            Point3::new(0.0, 0.0, 0.0),
            axis,
            z,
            Point3::new(0.0, 0.0, -shift),
            axis,
            z,
        );

        if i == 1 {
            prism.configure_motor_velocity(1.0, 1.0);
            prism.limits_enabled = true;
            prism.limits[1] = 5.0;
            // We set a max impulse so that the motor doesn't fight
            // the limits with large forces.
            prism.motor_max_impulse = 1.0;
        } else if i > 1 {
            prism.configure_motor_position(2.0, 0.01, 1.0);
        } else {
            prism.configure_motor_velocity(1.0, 1.0);
            // We set a max impulse so that the motor doesn't fight
            // the limits with large forces.
            prism.motor_max_impulse = 0.7;
            prism.limits_enabled = true;
            prism.limits[0] = -2.0;
            prism.limits[1] = 5.0;
        }

        joints.insert(bodies, curr_parent, curr_child, prism);

        curr_parent = curr_child;
    }
}

fn create_revolute_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::new_static()
        .translation(origin.x, origin.y, 0.0)
        .build();
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
    colliders.insert(collider, curr_parent, bodies);

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Isometry3::translation(origin.x, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z + shift),
            Isometry3::translation(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .position(positions[k])
                .build();
            handles[k] = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
            colliders.insert(collider, handles[k], bodies);
        }

        // Setup four joints.
        let o = Point3::origin();
        let x = Vector3::x_axis();
        let z = Vector3::z_axis();

        let revs = [
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(-shift, 0.0, 0.0), x),
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(shift, 0.0, 0.0), x),
        ];

        joints.insert(bodies, curr_parent, handles[0], revs[0]);
        joints.insert(bodies, handles[0], handles[1], revs[1]);
        joints.insert(bodies, handles[1], handles[2], revs[2]);
        joints.insert(bodies, handles[2], handles[3], revs[3]);

        curr_parent = handles[3];
    }
}

fn create_fixed_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let status = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status)
                .translation(origin.x + fk * shift, origin.y, origin.z + fi * shift)
                .build();
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad).build();
            colliders.insert(collider, child_handle, bodies);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(0.0, 0.0, -shift),
                );
                joints.insert(bodies, parent_handle, child_handle, joint);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(-shift, 0.0, 0.0),
                );
                joints.insert(bodies, parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }
}

fn create_ball_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    num: usize,
) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status)
                .translation(fk * shift, 0.0, fi * shift * 2.0)
                .build();
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::capsule_z(rad * 1.25, rad).build();
            colliders.insert(collider, child_handle, bodies);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift * 2.0));
                joints.insert(bodies, parent_handle, child_handle, joint);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
                joints.insert(bodies, parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }
}

fn create_actuated_revolute_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 2.0;

    // We will reuse this base configuration for all the joints here.
    let joint_template = RevoluteJoint::new(
        Point3::origin(),
        Vector3::z_axis(),
        Point3::new(0.0, 0.0, -shift),
        Vector3::z_axis(),
    );

    let mut parent_handle = RigidBodyHandle::invalid();

    for i in 0..num {
        let fi = i as f32;

        // NOTE: the num - 2 test is to avoid two consecutive
        // fixed bodies. Because physx will crash if we add
        // a joint between these.
        let status = if i == 0 {
            BodyStatus::Static
        } else {
            BodyStatus::Dynamic
        };

        let shifty = (i >= 1) as u32 as f32 * -2.0;

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(origin.x, origin.y + shifty, origin.z + fi * shift)
            // .rotation(Vector3::new(0.0, fi * 1.1, 0.0))
            .build();

        let child_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad * 2.0, rad * 6.0 / (fi + 1.0), rad).build();
        colliders.insert(collider, child_handle, bodies);

        if i > 0 {
            let mut joint = joint_template.clone();

            if i % 3 == 1 {
                joint.configure_motor_velocity(-20.0, 0.1);
            } else if i == num - 1 {
                let stiffness = 0.2;
                let damping = 1.0;
                joint.configure_motor_position(3.14 / 2.0, stiffness, damping);
            }

            if i == 1 {
                joint.local_anchor2.y = 2.0;
                joint.configure_motor_velocity(-2.0, 0.1);
            }

            joints.insert(bodies, parent_handle, child_handle, joint);
        }

        parent_handle = child_handle;
    }
}

fn create_actuated_ball_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    joints: &mut JointSet,
    origin: Point3<f32>,
    num: usize,
) {
    let rad = 0.4;
    let shift = 2.0;

    // We will reuse this base configuration for all the joints here.
    let joint_template = BallJoint::new(Point3::new(0.0, 0.0, shift), Point3::origin());

    let mut parent_handle = RigidBodyHandle::invalid();

    for i in 0..num {
        let fi = i as f32;

        // NOTE: the num - 2 test is to avoid two consecutive
        // fixed bodies. Because physx will crash if we add
        // a joint between these.
        let status = if i == 0 {
            BodyStatus::Static
        } else {
            BodyStatus::Dynamic
        };

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(origin.x, origin.y, origin.z + fi * shift)
            // .rotation(Vector3::new(0.0, fi * 1.1, 0.0))
            .build();

        let child_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::capsule_y(rad * 2.0 / (fi + 1.0), rad).build();
        colliders.insert(collider, child_handle, bodies);

        if i > 0 {
            let mut joint = joint_template.clone();

            if i == 1 {
                joint.configure_motor_velocity(Vector3::new(0.0, 0.5, -2.0), 0.1);
            } else if i == num - 1 {
                let stiffness = 0.2;
                let damping = 1.0;
                joint.configure_motor_position(
                    UnitQuaternion::new(Vector3::new(0.0, 1.0, 3.14 / 2.0)),
                    stiffness,
                    damping,
                );
            }

            joints.insert(bodies, parent_handle, child_handle, joint);
        }

        parent_handle = child_handle;
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    create_prismatic_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(20.0, 5.0, 0.0),
        4,
    );
    create_actuated_prismatic_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(25.0, 5.0, 0.0),
        4,
    );
    create_revolute_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(20.0, 0.0, 0.0),
        3,
    );
    create_fixed_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(0.0, 10.0, 0.0),
        10,
    );
    create_actuated_revolute_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(20.0, 10.0, 0.0),
        6,
    );
    create_actuated_ball_joints(
        &mut bodies,
        &mut colliders,
        &mut joints,
        Point3::new(13.0, 10.0, 0.0),
        3,
    );
    create_ball_joints(&mut bodies, &mut colliders, &mut joints, 15);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(15.0, 5.0, 42.0), Point3::new(13.0, 1.0, 1.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
