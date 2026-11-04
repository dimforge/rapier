//! `PhysicsPipeline::counters`: the contact pair, contact and constraint counts are filled by
//! the step, and the CCD substep count is zero unless the CCD actually ran.

use rapier3d::prelude::*;

#[cfg(feature = "profiler")]
use rapier3d::counters::{Counters, Timer};
#[cfg(feature = "profiler")]
use std::sync::Arc;
#[cfg(feature = "profiler")]
use std::sync::atomic::{AtomicUsize, Ordering};
#[cfg(feature = "profiler")]
use std::time::Duration;

#[cfg(feature = "profiler")]
fn timers(counters: &mut Counters) -> Vec<&mut Timer> {
    vec![
        &mut counters.step_time,
        &mut counters.custom,
        &mut counters.stages.update_time,
        &mut counters.stages.collision_detection_time,
        &mut counters.stages.island_construction_time,
        &mut counters.stages.island_constraints_collection_time,
        &mut counters.stages.solver_time,
        &mut counters.stages.ccd_time,
        &mut counters.stages.user_changes,
        &mut counters.cd.broad_phase_time,
        &mut counters.cd.final_broad_phase_time,
        &mut counters.cd.narrow_phase_time,
        &mut counters.solver.velocity_resolution_time,
        &mut counters.solver.velocity_assembly_time,
        &mut counters.solver.velocity_assembly_time_solver_bodies,
        &mut counters.solver.velocity_assembly_time_constraints_init,
        &mut counters.solver.velocity_update_time,
        &mut counters.solver.velocity_writeback_time,
        &mut counters.ccd.toi_computation_time,
        &mut counters.ccd.solver_time,
        &mut counters.ccd.broad_phase_time,
        &mut counters.ccd.narrow_phase_time,
    ]
}

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

#[test]
fn contact_and_constraint_counters() {
    let mut world = world_with_ground();
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    let (cube, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(5.0, 0.5, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    // A joint between the two resting bodies.
    world.insert_impulse_joint(
        ball,
        cube,
        SphericalJointBuilder::new()
            .local_anchor1(Vector::new(2.5, 0.0, 0.0))
            .local_anchor2(Vector::new(-2.5, 0.0, 0.0)),
    );

    for _ in 0..5 {
        world.step();
        let counters = &world.physics_pipeline.counters;
        // The ball-ground and cube-ground pairs.
        assert_eq!(counters.cd.ncontact_pairs, 2);
        // One manifold per pair plus the joint.
        assert_eq!(counters.solver.nconstraints, 3);
        // One contact point for the ball, four for the cube's face.
        assert_eq!(counters.solver.ncontacts, 5);
        // No CCD-enabled body.
        assert_eq!(counters.ccd.num_substeps, 0);
    }

    // Once the scene sleeps, nothing is solved anymore (the pairs are still tracked).
    for _ in 0..300 {
        world.step();
    }
    assert!(world.bodies[ball].is_sleeping());
    let counters = &world.physics_pipeline.counters;
    assert_eq!(counters.cd.ncontact_pairs, 2);
    assert_eq!(counters.solver.nconstraints, 0);
    assert_eq!(counters.solver.ncontacts, 0);
}

#[test]
fn ccd_substeps_counter() {
    let mut world = world_with_ground();
    world.integration_parameters.max_ccd_substeps = 4;
    world.gravity = Vector::ZERO;

    // A slow CCD-enabled body: the CCD never needs to act.
    let (slow, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0, 0.0))
            .ccd_enabled(true),
        ColliderBuilder::ball(0.5),
    );
    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 0);

    // A fast one heading to the ground: the CCD splits the step.
    world.bodies[slow].set_linvel(Vector::new(0.0, -500.0, 0.0), true);
    world.step();
    assert!(world.physics_pipeline.counters.ccd.num_substeps >= 1);
    assert!(world.bodies[slow].translation().y > -0.5);
}

#[cfg(feature = "profiler")]
#[test]
fn disabled_counters_freeze_direct_timers() {
    let mut counters = Counters::new(false);
    for timer in timers(&mut counters) {
        // Both entry points must be inert while their parent counters are disabled.
        timer.start();
        timer.resume();
    }
    std::thread::sleep(Duration::from_millis(2));
    for timer in timers(&mut counters) {
        timer.pause();
        assert_eq!(timer.time(), Duration::ZERO);
    }

    counters.enable();
    for timer in timers(&mut counters) {
        timer.start();
    }
    std::thread::sleep(Duration::from_millis(2));
    let measured: Vec<_> = timers(&mut counters)
        .into_iter()
        .map(|timer| {
            timer.pause();
            assert!(timer.time() > Duration::ZERO);
            timer.resume();
            timer.time()
        })
        .collect();

    counters.disable();
    std::thread::sleep(Duration::from_millis(2));
    counters.enable();
    for (timer, expected) in timers(&mut counters).into_iter().zip(measured) {
        timer.pause();
        assert_eq!(timer.time(), expected);
    }
}

#[cfg(feature = "profiler")]
#[test]
fn direct_enabled_flag_changes_apply_at_the_next_step() {
    let mut world = world_with_ground();
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.55, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    world.physics_pipeline.counters.enable();
    world.step();
    assert!(world.physics_pipeline.counters.step_time() > Duration::ZERO);
    let measured: Vec<_> = timers(&mut world.physics_pipeline.counters)
        .into_iter()
        .map(|timer| timer.time())
        .collect();

    world.physics_pipeline.counters.enabled = false;
    for _ in 0..3 {
        world.step();
    }
    for (timer, expected) in timers(&mut world.physics_pipeline.counters)
        .into_iter()
        .zip(measured)
    {
        assert_eq!(timer.time(), expected);
    }

    world.physics_pipeline.counters.enabled = true;
    world.step();
    assert!(world.physics_pipeline.counters.step_time() > Duration::ZERO);
    assert!(world.physics_pipeline.counters.cd.narrow_phase_time.time() > Duration::ZERO);
}

#[cfg(feature = "profiler")]
#[derive(Clone, Default)]
struct SlowContactHooks(Arc<AtomicUsize>);

#[cfg(feature = "profiler")]
const HOOK_DELAY: Duration = Duration::from_millis(2);
#[cfg(feature = "profiler")]
const MIN_CLAMP_PASSES: usize = 2;

#[cfg(feature = "profiler")]
impl PhysicsHooks for SlowContactHooks {
    fn filter_contact_pair(&self, _: &PairFilterContext<'_>) -> Option<SolverFlags> {
        self.0.fetch_add(1, Ordering::Relaxed);
        std::thread::sleep(HOOK_DELAY);
        Some(SolverFlags::COMPUTE_RIGID_IMPULSES)
    }
}

#[cfg(feature = "profiler")]
#[test]
fn ccd_timer_accumulates_work_from_multiple_clamp_passes() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.max_ccd_substeps = 4;
    for (y, x) in [(0.0, -0.3), (2.0, -0.8)] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, y, 0.0)),
            ColliderBuilder::cuboid(0.05, 0.4, 0.4),
        );
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y, 0.0))
                .linvel(Vector::new(60.0, 0.0, 0.0))
                .ccd_enabled(true),
            ColliderBuilder::cuboid(0.05, 0.05, 0.05)
                .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS),
        );
    }

    let hooks = SlowContactHooks::default();
    world.step_with_events(&hooks, &());

    assert!(
        world.physics_pipeline.counters.ccd.num_substeps >= MIN_CLAMP_PASSES,
        "fixture must execute multiple CCD clamp passes"
    );
    assert!(
        hooks.0.load(Ordering::Relaxed) >= MIN_CLAMP_PASSES,
        "fixture must inject work into multiple contact-filter calls"
    );
    let minimum_accumulated_time = HOOK_DELAY * MIN_CLAMP_PASSES as u32;
    assert!(
        world
            .physics_pipeline
            .counters
            .ccd
            .toi_computation_time
            .time()
            >= minimum_accumulated_time,
        "CCD timing must include at least two injected clamp-pass delays"
    );
}
