use na::{Isometry3, Point3, Unit, Vector3};
use rapier3d::dynamics::{
    BallJoint, BodyStatus, FixedJoint, JointSet, PrismaticJoint, RevoluteJoint, RigidBodyBuilder,
    RigidBodySet,
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
    let shift = 1.0;

    let ground = RigidBodyBuilder::new_static()
        .translation(origin.x, origin.y, origin.z)
        .build();
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::new_cuboid(rad, rad, rad).build();
    colliders.insert(bodies, collider, curr_parent);

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let density = 1.0;
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(origin.x, origin.y, z)
            .build();
        let curr_child = bodies.insert(rigid_body);
        let collider = ColliderBuilder::new_cuboid(rad, rad, rad)
            .density(density)
            .build();
        colliders.insert(bodies, collider, curr_child);

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let z = Vector3::z();
        let mut prism = PrismaticJoint::new(
            Point3::origin(),
            axis,
            z,
            Point3::new(0.0, 0.0, -shift),
            axis,
            z,
        );
        prism.limits_enabled = true;
        prism.limits[0] = -2.0;
        prism.limits[1] = 2.0;
        joints.insert(bodies, prism, curr_parent, curr_child);

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
    let collider = ColliderBuilder::new_cuboid(rad, rad, rad).build();
    colliders.insert(bodies, collider, curr_parent);

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
            let density = 1.0;
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .position(positions[k])
                .build();
            handles[k] = bodies.insert(rigid_body);
            let collider = ColliderBuilder::new_cuboid(rad, rad, rad)
                .density(density)
                .build();
            colliders.insert(bodies, collider, handles[k]);
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

        joints.insert(bodies, revs[0], curr_parent, handles[0]);
        joints.insert(bodies, revs[1], handles[0], handles[1]);
        joints.insert(bodies, revs[2], handles[1], handles[2]);
        joints.insert(bodies, revs[3], handles[2], handles[3]);

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
            let collider = ColliderBuilder::new_ball(rad).density(1.0).build();
            colliders.insert(bodies, collider, child_handle);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(0.0, 0.0, -shift),
                );
                joints.insert(bodies, joint, parent_handle, child_handle);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(-shift, 0.0, 0.0),
                );
                joints.insert(bodies, joint, parent_handle, child_handle);
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
                .translation(fk * shift, 0.0, fi * shift)
                .build();
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::new_ball(rad).density(1.0).build();
            colliders.insert(bodies, collider, child_handle);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift));
                joints.insert(bodies, joint, parent_handle, child_handle);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
                joints.insert(bodies, joint, parent_handle, child_handle);
            }

            body_handles.push(child_handle);
        }
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
        Point3::new(20.0, 10.0, 0.0),
        5,
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
        5,
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
