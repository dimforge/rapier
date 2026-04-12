use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    let mut world = PhysicsWorld::new();

    let ground = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0, 0.0));
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
    let (body, _) = world.insert(ground, collider);

    let rigid_body =
        RigidBodyBuilder::dynamic().pose(Pose::from_translation(Vector::new(0.0, 1.0, 0.0)));
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0).density(1.0);
    let (body_part, _) = world.insert(rigid_body, collider);

    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 4.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0, 0.0))
        .motor_position(JointAxis::AngX, 1.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngY, 0.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngZ, 0.0, 1000.0, 200.0);

    world.insert_multibody_joint(body, body_part, joint);

    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(20.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
}
