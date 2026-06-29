use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

use rand::distr::{Distribution, StandardUniform};
use rand::{SeedableRng, rngs::StdRng};

const HALF_LENGTH: f32 = 1.0;

pub fn init_world(testbed: &mut Testbed) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    let settings = testbed.example_settings_mut();

    let randomize = settings.get_or_set_bool("Randomize", false);
    let count = settings.get_or_set_u32("Pendulum Count", 6, 1..=16);
    let segments = settings.get_or_set_u32("Pendulum Segments", 3, 1..=8);

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = StandardUniform;

    for i in 0..count {
        let base_pos = Vector::new(i as f32 * 10.0 * HALF_LENGTH, 0.0);
        let base = RigidBodyBuilder::fixed().translation(base_pos);
        let base_handle = world.insert_body(base);

        let mut prev_handle = base_handle;
        let mut prev_end_local = Vector::ZERO;
        let mut prev_end_world = base_pos;

        for _ in 0..segments {
            let rotation = if randomize {
                let rand_val: f32 = distribution.sample(&mut rng);
                (rand_val - 0.5) * std::f32::consts::PI
            } else {
                0.0
            };

            let body = RigidBodyBuilder::dynamic()
                .translation(prev_end_world)
                .rotation(rotation)
                .can_sleep(false);

            let collider = ColliderBuilder::capsule_x(HALF_LENGTH, 0.2 * HALF_LENGTH)
                .translation(Vector::new(HALF_LENGTH, 0.0));

            let (handle, _) = world.insert(body, collider);

            let joint = RevoluteJointBuilder::new()
                .local_anchor1(prev_end_local)
                .local_anchor2(Vector::ZERO)
                .contacts_enabled(false);

            world.insert_impulse_joint(prev_handle, handle, joint);

            let end_local = Vector::new(2.0 * HALF_LENGTH, 0.0);

            prev_handle = handle;
            prev_end_local = end_local;
            prev_end_world += Vector::from_angle(rotation).rotate(end_local);
        }
    }

    testbed.set_physics_world(world);
    testbed.look_at(Vector::ZERO, 10.0);
}
