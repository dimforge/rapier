use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let ground = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0, 0.0));
    let body = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
    colliders.insert_with_parent(collider, body, &mut bodies);

    let rigid_body =
        RigidBodyBuilder::dynamic().pose(Pose::from_translation(Vector::new(0.0, 1.0, 0.0)));
    let body_part = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0).density(1.0);
    colliders.insert_with_parent(collider, body_part, &mut bodies);

    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 4.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0, 0.0))
        .motor_position(JointAxis::AngX, 1.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngY, 0.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngZ, 0.0, 1000.0, 200.0);

    multibody_joints.insert(body, body_part, joint, true);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(20.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
}
