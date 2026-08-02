//! Regression test for `RigidBody::additional_solver_iterations` as extra
//! substeps: a hanging chain with a very heavy end ball (1000:1 mass ratio) —
//! the `debug_chain_high_mass_ratio3` testbed scene — must hold together much
//! more tightly with extra substeps than without.
//!
//! The old semantics (extra flat velocity-PGS sweeps over a frozen bias) had
//! no effect here: the velocity solve converges after a few sweeps, while the
//! visible stretch is a property of the once-per-substep soft position bias.
//! Extra *substeps* re-derive the bias at a smaller dt and re-integrate
//! positions, which is what actually closes the gap.

use rapier3d::pipeline::PhysicsWorld;
use rapier3d::prelude::*;

/// Builds the chain scene; every body requests `extra` additional iterations.
fn chain_world(extra: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    let num = 17;
    let rad = 0.2;
    let mut prev_handle: Option<RigidBodyHandle> = None;

    for i in 0..num {
        let fi = i as Real;
        let status = if i == 0 {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        };
        let ball_rad = if i == num - 1 { rad * 10.0 } else { rad };
        let shift1 = rad * 1.1;
        let shift2 = ball_rad + rad * 0.1;
        let z = if i == 0 {
            0.0
        } else {
            (fi - 1.0) * 2.0 * shift1 + shift1 + shift2
        };

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(Vector::new(0.0, 0.0, z))
            .can_sleep(false)
            .additional_solver_iterations(extra);
        let collider = ColliderBuilder::ball(ball_rad);
        let (child_handle, _) = world.insert(rigid_body, collider);

        if let Some(parent_handle) = prev_handle {
            let joint = if i == 1 {
                SphericalJointBuilder::new().local_anchor2(Vector::new(0.0, 0.0, -shift1 * 2.0))
            } else {
                SphericalJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, 0.0, shift1))
                    .local_anchor2(Vector::new(0.0, 0.0, -shift2))
            };
            world.insert_impulse_joint(parent_handle, child_handle, joint);
        }
        prev_handle = Some(child_handle);
    }

    world
}

/// Max world-space separation of the two anchor points over all joints — the
/// visible "chain stretch" constraint violation.
fn max_joint_stretch(world: &PhysicsWorld) -> Real {
    let mut max = 0.0f32;
    for (_, joint) in world.impulse_joints.iter() {
        let rb1 = &world.bodies[joint.body1()];
        let rb2 = &world.bodies[joint.body2()];
        let a1 = rb1.position() * joint.data.local_frame1.translation;
        let a2 = rb2.position() * joint.data.local_frame2.translation;
        max = max.max((a1 - a2).length());
    }
    max
}

fn peak_stretch(extra: usize, steps: usize) -> Real {
    let mut world = chain_world(extra);
    let mut peak = 0.0f32;
    for _ in 0..steps {
        world.step();
        peak = peak.max(max_joint_stretch(&world));
    }
    peak
}

/// Locality of the per-group substep ring: in a mixed scene (a big default
/// pile + one elevated chain), only the chain's group runs the extra
/// substeps. Stepping must be substantially cheaper than the same scene with
/// everything elevated (the global-elevation cost model).
///
/// Timing-sensitive: run with `--ignored --test-threads=1`.
#[test]
#[ignore = "timing comparison; run with --ignored --test-threads=1"]
fn substep_groups_locality_timing() {
    fn pile_world(pile_extra: usize, chain_extra: usize) -> PhysicsWorld {
        let mut world = chain_world(chain_extra);
        // Ground + a 10x8 wall of boxes, far from the chain.
        let ground = RigidBodyBuilder::fixed().translation(Vector::new(50.0, -1.0, 0.0));
        world.insert(ground, ColliderBuilder::cuboid(30.0, 0.5, 30.0));
        for i in 0..10 {
            for j in 0..8 {
                let body = RigidBodyBuilder::dynamic()
                    .translation(Vector::new(40.0 + i as Real * 1.01, j as Real * 1.01, 0.0))
                    .can_sleep(false)
                    .additional_solver_iterations(pile_extra);
                world.insert(body, ColliderBuilder::cuboid(0.5, 0.5, 0.5));
            }
        }
        world
    }

    fn time_steps(world: &mut PhysicsWorld, steps: usize) -> f64 {
        let start = std::time::Instant::now();
        for _ in 0..steps {
            world.step();
        }
        start.elapsed().as_secs_f64()
    }

    let warmup = 30;
    let steps = 120;

    // A: only the chain elevated -> the pile's group stays at base substeps.
    let mut grouped = pile_world(0, 16);
    time_steps(&mut grouped, warmup);
    let t_grouped = time_steps(&mut grouped, steps);

    // B: everything elevated -> the whole scene pays the extra substeps.
    let mut global = pile_world(16, 16);
    time_steps(&mut global, warmup);
    let t_global = time_steps(&mut global, steps);

    println!(
        "mixed-scene step time: per-group={:.3}ms, all-elevated={:.3}ms ({:.2}x)",
        t_grouped * 1e3 / steps as f64,
        t_global * 1e3 / steps as f64,
        t_global / t_grouped,
    );
    assert!(
        t_grouped < t_global * 0.7,
        "per-group substeps did not pay off: grouped={t_grouped}s vs global={t_global}s"
    );
}

#[test]
fn substep_chain_high_mass_ratio_stretch() {
    // 5 simulated seconds: covers the initial swing (the worst transient).
    let steps = 300;
    let baseline = peak_stretch(0, steps);
    let elevated = peak_stretch(16, steps);

    println!("chain peak stretch: baseline={baseline}, extra-substeps={elevated}");

    // The elevated chain must hold together at least 4x more tightly than the
    // baseline. (Measured: substeps 4 -> 20 shrinks the peak stretch far more
    // than that; the loose factor keeps the test robust across platforms.)
    assert!(
        elevated < baseline / 4.0,
        "extra substeps did not stiffen the chain: baseline={baseline}, elevated={elevated}"
    );
}
