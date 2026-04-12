use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

// Simulate the the Dzhanibekov effect:
// https://en.wikipedia.org/wiki/Tennis_racket_theorem
pub fn init_world(testbed: &mut Testbed) {
    let mut world = PhysicsWorld::new();

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
    let collider = ColliderBuilder::compound(shapes);
    let (_body_handle, _) = world.insert(body, collider);

    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(8.0, 0.0, 8.0), Vec3::new(0.0, 0.0, 0.0));
}
