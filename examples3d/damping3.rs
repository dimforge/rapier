use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    /*
     * Create the cubes
     */
    let num = 10;
    let rad = 0.2;

    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * std::f32::consts::PI * 2.0).sin_cos();

        // Build the rigid body.
        let rb = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y, 0.0))
            .linvel(Vector::new(x * 10.0, y * 10.0, 0.0))
            .angvel(Vector::Z * 100.0)
            .linear_damping((i + 1) as f32 * subdiv * 10.0)
            .angular_damping((num - i) as f32 * subdiv * 10.0);
        // Build the collider.
        let co = ColliderBuilder::cuboid(rad, rad, rad);
        let (_rb_handle, _) = world.insert(rb, co);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(2.0, 2.5, 20.0), Vec3::new(2.0, 2.5, 0.0));
}
