use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

use rand::distr::{Distribution, StandardUniform};
use rand::{SeedableRng, rngs::StdRng};

/*
 * Length of each pendulum segment.
 */
const HALF_LENGTH: f32 = 1.0;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * Physics world with gravity.
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    /*
     * User‑controllable parameters from the testbed UI.
     */
    let settings = viewer.example_settings_mut();

    let randomize = settings.get_or_set_bool("Randomize", false);
    // The defaults are picked to exercise the SIMD joint-constraint path: a constraint color
    // needs at least 64 joints before joints get batched into SIMD groups, and 32 pendulums of
    // 4 segments is the smallest combination here that reaches it. Smaller scenes fall back to
    // the scalar path entirely (see the `issue_952_simd_joint_offset_com` regression tests).
    let count = settings.get_or_set_u32("Pendulum Count", 32, 1..=64);
    let segments = settings.get_or_set_u32("Pendulum Segments", 4, 1..=40);

    /*
     * Random number generator for randomized pendulum angles.
     */
    let mut rng = StdRng::seed_from_u64(0);
    let distribution = StandardUniform;

    /*
     * Square grid layout. A chain is built towards +x and settles hanging down, so each cell
     * must be a full chain long in both directions to keep neighbors from touching.
     */
    let chain_length = 2.0 * segments as f32 * HALF_LENGTH;
    let spacing = chain_length + 4.0 * HALF_LENGTH;
    let cols = (count as f32).sqrt().ceil() as u32;
    let rows = count.div_ceil(cols);

    /*
     * Create pendulums
     */
    for i in 0..count {
        /*
         * Create a fixed base for the pendulum, centering the grid on the origin.
         */
        let (col, row) = (i % cols, i / cols);
        let base_pos = Vector::new(
            (col as f32 - (cols - 1) as f32 * 0.5) * spacing,
            ((rows - 1) as f32 * 0.5 - row as f32) * spacing,
        );
        let base = RigidBodyBuilder::fixed().translation(base_pos);
        let base_handle = world.insert_body(base);

        /*
         * Record the end for next iteration.
         */
        let mut prev_handle = base_handle;
        let mut prev_end_local = Vector::ZERO;
        let mut prev_end_world = base_pos;

        for _ in 0..segments {
            /*
             * Create a new segment, with a random rotation if randomize is set to true.
             */
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

            /*
             * Join the newly created segment to the previous one.
             */
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(prev_end_local)
                .local_anchor2(Vector::ZERO)
                .contacts_enabled(false);

            world.insert_impulse_joint(prev_handle, handle, joint);

            /*
             * Record the end for next iteration.
             */
            let end_local = Vector::new(2.0 * HALF_LENGTH, 0.0);

            prev_handle = handle;
            prev_end_local = end_local;
            prev_end_world += Vector::from_angle(rotation).rotate(end_local);
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);

    // Frame the whole grid, whatever the count/segments settings are (the visible width is
    // roughly the window width divided by the zoom).
    let span = (cols.max(rows) as f32) * spacing;
    viewer.look_at(Vector::ZERO, (1000.0 / span).min(20.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
