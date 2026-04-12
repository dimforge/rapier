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
    let count = 40;
    let hx = 0.5;
    let density = 20.0;
    let friction = 0.6;
    let capsule = ColliderBuilder::capsule_x(hx, 0.125)
        .friction(friction)
        .density(density);

    let mut prev = ground;
    for i in 0..count {
        let rigid_body = RigidBodyBuilder::dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation(Vector::new((1.0 + 2.0 * i as f32) * hx, count as f32 * hx));
        let (handle, _) = world.insert(rigid_body, capsule.clone());

        let pivot = Vector::new((2.0 * i as f32) * hx, count as f32 * hx);
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(world.bodies[prev].position().inverse_transform_point(pivot))
            .local_anchor2(world.bodies[handle].position().inverse_transform_point(pivot))
            .contacts_enabled(false);
        world.insert_impulse_joint(prev, handle, joint);
        prev = handle;
    }

    let radius = 8.0;
    let rigid_body = RigidBodyBuilder::dynamic()
        .linear_damping(0.1)
        .angular_damping(0.1)
        .translation(Vector::new(
            (1.0 + 2.0 * count as f32) * hx + radius - hx,
            count as f32 * hx,
        ));
    let collider = ColliderBuilder::ball(radius)
        .friction(friction)
        .density(density);
    let (handle, _) = world.insert(rigid_body, collider);

    let pivot = Vector::new((2.0 * count as f32) * hx, count as f32 * hx);
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(world.bodies[prev].position().inverse_transform_point(pivot))
        .local_anchor2(world.bodies[handle].position().inverse_transform_point(pivot))
        .contacts_enabled(false);
    world.insert_impulse_joint(prev, handle, joint);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
