use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

// Simulate the the Dzhanibekov effect:
// https://en.wikipedia.org/wiki/Tennis_racket_theorem
pub fn init_world(testbed: &mut Testbed) {
    let mut colliders = ColliderSet::new();
    let mut bodies = RigidBodySet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let shapes = vec![
        (Pose::IDENTITY, SharedShape::cuboid(2.0, 0.2, 0.2)),
        (
            Pose::from_translation(Vector::new(0.0, 0.8, 0.0)),
            SharedShape::cuboid(0.2, 0.4, 0.2),
        ),
    ];

    let body = RigidBodyBuilder::dynamic()
        .gravity_scale(0.0)
        .angvel(Vector::new(0.0, 20.0, 0.1))
        .gyroscopic_forces_enabled(true);
    let body_handle = bodies.insert(body);
    let collider = ColliderBuilder::compound(shapes);
    colliders.insert_with_parent(collider, body_handle, &mut bodies);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(8.0, 0.0, 8.0), Vec3::new(0.0, 0.0, 0.0));
}
