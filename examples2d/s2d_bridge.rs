use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground = world.insert_body(RigidBodyBuilder::fixed());

    /*
     * Create the bridge.
     */
    let density = 20.0;
    let x_base = -80.0;
    let count = 160;
    let mut prev = ground;

    for i in 0..count {
        let rigid_body = RigidBodyBuilder::dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation(Vector::new(x_base + 0.5 + 1.0 * i as f32, 20.0));
        let collider = ColliderBuilder::cuboid(0.5, 0.125).density(density);
        let (handle, _) = world.insert(rigid_body, collider);

        let pivot = Vector::new(x_base + 1.0 * i as f32, 20.0);
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(world.bodies[prev].position().inverse_transform_point(pivot))
            .local_anchor2(world.bodies[handle].position().inverse_transform_point(pivot))
            .contacts_enabled(false);
        world.insert_impulse_joint(prev, handle, joint);
        prev = handle;
    }

    let pivot = Vector::new(x_base + 1.0 * count as f32, 20.0);
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(world.bodies[prev].position().inverse_transform_point(pivot))
        .local_anchor2(world.bodies[ground].position().inverse_transform_point(pivot))
        .contacts_enabled(false);
    world.insert_impulse_joint(prev, ground, joint);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
