//! Pipeline stepping regression tests.

use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, RigidBodyBuilder,
    RigidBodySet,
};
use crate::geometry::{BroadPhaseBvh, ColliderBuilder, ColliderSet, NarrowPhase};
#[cfg(feature = "dim2")]
use crate::math::Rotation;
use crate::math::Vector;
use crate::pipeline::PhysicsPipeline;
use crate::prelude::{MultibodyJointSet, RevoluteJointBuilder, RigidBodyType};

#[test]
fn kinematic_and_fixed_contact_crash() {
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut islands = IslandManager::new();

    let rb = RigidBodyBuilder::fixed().build();
    let h1 = bodies.insert(rb.clone());
    let co = ColliderBuilder::ball(10.0).build();
    colliders.insert_with_parent(co.clone(), h1, &mut bodies);

    // The same but with a kinematic body.
    let rb = RigidBodyBuilder::kinematic_position_based().build();
    let h2 = bodies.insert(rb.clone());
    colliders.insert_with_parent(co, h2, &mut bodies);

    pipeline.step(
        Vector::ZERO,
        &IntegrationParameters::default(),
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
}

#[test]
fn rigid_body_removal_before_step() {
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();

    let mut bodies = RigidBodySet::new();

    // Check that removing the body right after inserting it works.
    // We add two dynamic bodies, one kinematic body and one fixed body before removing
    // them. This include a non-regression test where deleting a kinematic body crashes.
    let rb = RigidBodyBuilder::dynamic().build();
    let h1 = bodies.insert(rb.clone());
    let h2 = bodies.insert(rb.clone());

    // The same but with a kinematic body.
    let rb = RigidBodyBuilder::kinematic_position_based().build();
    let h3 = bodies.insert(rb.clone());

    // The same but with a fixed body.
    let rb = RigidBodyBuilder::fixed().build();
    let h4 = bodies.insert(rb.clone());

    let to_delete = [h1, h2, h3, h4];
    for h in &to_delete {
        bodies.remove(
            *h,
            &mut islands,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            true,
        );
    }

    pipeline.step(
        Vector::ZERO,
        &IntegrationParameters::default(),
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
}

#[cfg(feature = "serde-serialize")]
#[test]
fn rigid_body_removal_snapshot_handle_determinism() {
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut islands = IslandManager::new();

    let mut bodies = RigidBodySet::new();
    let rb = RigidBodyBuilder::dynamic().build();
    let h1 = bodies.insert(rb.clone());
    let h2 = bodies.insert(rb.clone());
    let h3 = bodies.insert(rb.clone());

    bodies.remove(
        h1,
        &mut islands,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        true,
    );
    bodies.remove(
        h3,
        &mut islands,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        true,
    );
    bodies.remove(
        h2,
        &mut islands,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        true,
    );

    let ser_bodies = bincode::serialize(&bodies).unwrap();
    let mut bodies2: RigidBodySet = bincode::deserialize(&ser_bodies).unwrap();

    let h1a = bodies.insert(rb.clone());
    let h2a = bodies.insert(rb.clone());
    let h3a = bodies.insert(rb.clone());

    let h1b = bodies2.insert(rb.clone());
    let h2b = bodies2.insert(rb.clone());
    let h3b = bodies2.insert(rb.clone());

    assert_eq!(h1a, h1b);
    assert_eq!(h2a, h2b);
    assert_eq!(h3a, h3b);
}

// Regression test for https://github.com/dimforge/rapier/issues/754 —
// CCD must consult `filter_contact_pair` just like the narrow phase, so
// pairs the user filtered out don't clamp a fast CCD body's motion.
#[test]
#[cfg(feature = "dim3")]
fn ccd_respects_filter_contact_pair_hook() {
    use crate::pipeline::{ActiveHooks, PairFilterContext, PhysicsHooks};
    use crate::prelude::{ColliderHandle, SolverFlags};
    use core::sync::atomic::{AtomicUsize, Ordering};

    struct RejectAllHooks {
        calls: AtomicUsize,
    }
    impl PhysicsHooks for RejectAllHooks {
        fn filter_contact_pair(&self, _: &PairFilterContext) -> Option<SolverFlags> {
            self.calls.fetch_add(1, Ordering::Relaxed);
            None // reject every pair
        }
    }

    let mut pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters::default();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut ccd = CCDSolver::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut islands = IslandManager::new();
    let hooks = RejectAllHooks {
        calls: AtomicUsize::new(0),
    };
    let event_handler = ();

    // Body A: fast-moving, CCD-enabled.
    let body_a = RigidBodyBuilder::dynamic()
        .translation(Vector::new(-5.0, 0.0, 0.0))
        .linvel(Vector::new(200.0, 0.0, 0.0))
        .ccd_enabled(true)
        .build();
    let a_handle = bodies.insert(body_a);
    let _: ColliderHandle = colliders.insert_with_parent(
        ColliderBuilder::ball(0.5)
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .build(),
        a_handle,
        &mut bodies,
    );

    // Body B: the stationary target, *fixed* — must not be a bullet: the target
    // tiering (`tier_allows`) never sweeps bullet-vs-bullet, so that pair would never
    // reach CCD and the test would pass vacuously, asserting nothing about the hook.
    let body_b = RigidBodyBuilder::fixed().build();
    let b_handle = bodies.insert(body_b);
    let _: ColliderHandle = colliders.insert_with_parent(
        ColliderBuilder::ball(0.5)
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .build(),
        b_handle,
        &mut bodies,
    );

    for _ in 0..5 {
        pipeline.step(
            Vector::ZERO,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &hooks,
            &event_handler,
        );
    }

    // Hook must be called at least once (from CCD, since they never
    // reach narrow-phase contact in a single step at 200 m/s × 1/60s).
    assert!(
        hooks.calls.load(Ordering::Relaxed) > 0,
        "filter_contact_pair was never called",
    );

    // Without the fix: CCD clamps A's motion at the predicted impact
    // with B (hook ignored). A stalls near B.
    // With the fix: A flies straight through at 200 m/s for 5 steps of
    // dt=1/60s ≈ 16.67 units, so it ends near +11.67.
    let a_pos = bodies[a_handle].translation().x;
    assert!(
        a_pos > 10.0,
        "body A should have passed through filtered body B, but x={a_pos}",
    );
}

#[test]
fn collider_removal_before_step() {
    let mut pipeline = PhysicsPipeline::new();
    let gravity = Vector::Y * -9.81;
    let integration_parameters = IntegrationParameters::default();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut ccd = CCDSolver::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut islands = IslandManager::new();
    let physics_hooks = ();
    let event_handler = ();

    let body = RigidBodyBuilder::dynamic().build();
    let b_handle = bodies.insert(body);
    let collider = ColliderBuilder::ball(1.0).build();
    let c_handle = colliders.insert_with_parent(collider, b_handle, &mut bodies);
    colliders.remove(c_handle, &mut islands, &mut bodies, true);
    bodies.remove(
        b_handle,
        &mut islands,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        true,
    );

    for _ in 0..10 {
        pipeline.step(
            gravity,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &physics_hooks,
            &event_handler,
        );
    }
}

#[test]
fn rigid_body_type_changed_dynamic_is_in_active_set() {
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();

    let mut bodies = RigidBodySet::new();

    // Initialize body as kinematic with mass
    let rb = RigidBodyBuilder::kinematic_position_based()
        .additional_mass(1.0)
        .build();
    let h = bodies.insert(rb.clone());

    // Step once
    let gravity = Vector::Y * -9.81;
    pipeline.step(
        gravity,
        &IntegrationParameters::default(),
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );

    // Switch body type to Dynamic
    bodies
        .get_mut(h)
        .unwrap()
        .set_body_type(RigidBodyType::Dynamic, true);

    // Step again
    pipeline.step(
        gravity,
        &IntegrationParameters::default(),
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );

    let body = bodies.get(h).unwrap();
    let h_y = body.pos.position.translation.y;

    // Expect gravity to be applied on second step after switching to Dynamic
    assert!(h_y < 0.0);

    // Expect body to now be awake (not sleeping)
    assert!(!body.is_sleeping());
}

#[test]
fn joint_step_delta_time_0() {
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();

    let mut bodies = RigidBodySet::new();

    // Initialize bodies
    let rb = RigidBodyBuilder::fixed().additional_mass(1.0).build();
    let h = bodies.insert(rb.clone());
    let rb_dynamic = RigidBodyBuilder::dynamic().additional_mass(1.0).build();
    let h_dynamic = bodies.insert(rb_dynamic.clone());

    // Add joint
    #[cfg(feature = "dim2")]
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 1.0))
        .local_anchor2(Vector::new(0.0, -3.0));
    #[cfg(feature = "dim3")]
    let joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(0.0, 1.0, 0.0))
        .local_anchor2(Vector::new(0.0, -3.0, 0.0));
    impulse_joints.insert(h, h_dynamic, joint, true);

    let parameters = IntegrationParameters {
        dt: 0.0,
        ..Default::default()
    };
    // Step once
    let gravity = Vector::Y * -9.81;
    pipeline.step(
        gravity,
        &parameters,
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
    let translation = bodies[h_dynamic].translation();
    let rotation = bodies[h_dynamic].rotation();
    assert!(translation.x.is_finite());
    assert!(translation.y.is_finite());
    #[cfg(feature = "dim2")]
    {
        assert!(rotation.re.is_finite());
        assert!(rotation.im.is_finite());
    }
    #[cfg(feature = "dim3")]
    {
        assert!(translation.z.is_finite());
        assert!(rotation.x.is_finite());
        assert!(rotation.y.is_finite());
        assert!(rotation.z.is_finite());
        assert!(rotation.w.is_finite());
    }
}

#[test]
#[cfg(feature = "dim2")]
fn test_multi_sap_disable_body() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1);
    collider_set.insert(collider);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0));
    let collider = ColliderBuilder::ball(0.5).restitution(0.7);
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    /* Create other structures necessary for the simulation. */
    let gravity = Vector::new(0.0, -9.81);
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    // Test RigidBodyChanges::POSITION and disable
    {
        let ball_body = &mut rigid_body_set[ball_body_handle];

        // Also, change the translation and rotation to different values
        ball_body.set_translation(Vector::new(1.0, 1.0), true);
        ball_body.set_rotation(Rotation::from_angle(1.0), true);
        ball_body.set_enabled(false);
    }

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    // Test RigidBodyChanges::POSITION and enable
    {
        let ball_body = &mut rigid_body_set[ball_body_handle];

        // Also, change the translation and rotation to different values
        ball_body.set_translation(Vector::new(0.0, 0.0), true);
        ball_body.set_rotation(Rotation::from_angle(0.0), true);
        ball_body.set_enabled(true);
    }

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );
}

#[test]
fn user_force_persists_across_steps() {
    // Regression test for issue #903: user-added forces are NOT cleared automatically.
    // They keep being applied at every physics step until `reset_forces()` is called.
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut bodies = RigidBodySet::new();
    let params = IntegrationParameters::default();

    let handle = bodies.insert(RigidBodyBuilder::dynamic().additional_mass(1.0));
    bodies[handle].add_force(Vector::X, true);

    // Step once and record the resulting velocity along X.
    pipeline.step(
        Vector::ZERO, // No gravity.
        &params,
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
    let vel_after_1 = bodies[handle].linvel().x;

    // Step again *without* re-adding the force.
    pipeline.step(
        Vector::ZERO,
        &params,
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
    let vel_after_2 = bodies[handle].linvel().x;

    // A constant force of 1N on a 1kg body increases the velocity by the same amount
    // every step. If the force had been cleared after the first step, `vel_after_2`
    // would equal `vel_after_1`.
    assert!(vel_after_1 > 0.0);
    assert!(
        (vel_after_2 - 2.0 * vel_after_1).abs() < 1.0e-5,
        "force should persist across steps: v1 = {vel_after_1}, v2 = {vel_after_2}"
    );
    // The force is still registered on the body.
    assert_eq!(bodies[handle].user_force(), Vector::X);

    // After `reset_forces`, stepping no longer accelerates the body.
    bodies[handle].reset_forces(true);
    pipeline.step(
        Vector::ZERO,
        &params,
        &mut islands,
        &mut bf,
        &mut nf,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );
    let vel_after_reset = bodies[handle].linvel().x;
    assert!((vel_after_reset - vel_after_2).abs() < 1.0e-5);
}

/// Contact-force events must follow *runtime* `CONTACT_FORCE_EVENTS` flips on an
/// already-touching pair: no change flag or contact update fires, so this exercises
/// the user-modified-collider reconciliation path of the incremental force-event list.
#[test]
fn contact_force_events_follow_runtime_active_events_flips() {
    use crate::geometry::ContactPair;
    use crate::pipeline::{ActiveEvents, EventHandler, PhysicsWorld};
    use core::sync::atomic::{AtomicUsize, Ordering};

    #[derive(Default)]
    struct ForceEventCounter(AtomicUsize);
    impl EventHandler for ForceEventCounter {
        fn handle_collision_event(
            &self,
            _: &RigidBodySet,
            _: &ColliderSet,
            _: crate::geometry::CollisionEvent,
            _: Option<&ContactPair>,
        ) {
        }
        fn handle_contact_force_event(
            &self,
            _: crate::math::Real,
            _: &RigidBodySet,
            _: &ColliderSet,
            _: &ContactPair,
            _: crate::math::Real,
        ) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[cfg(feature = "dim2")]
    fn vector_y(y: crate::math::Real) -> Vector {
        Vector::new(0.0, y)
    }
    #[cfg(feature = "dim3")]
    fn vector_y(y: crate::math::Real) -> Vector {
        Vector::new(0.0, y, 0.0)
    }

    let events = ForceEventCounter::default();
    let mut world = PhysicsWorld::new();

    let _ = world.insert(
        RigidBodyBuilder::fixed(),
        #[cfg(feature = "dim2")]
        ColliderBuilder::cuboid(2.0, 0.5),
        #[cfg(feature = "dim3")]
        ColliderBuilder::cuboid(2.0, 0.5, 2.0),
    );
    let (_, ball_collider) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(vector_y(1.05))
            .can_sleep(false),
        ColliderBuilder::ball(0.5),
    );

    // Settle into resting contact; no force events are enabled yet.
    for _ in 0..30 {
        world.step_with_events(&(), &events);
    }
    assert_eq!(events.0.load(Ordering::Relaxed), 0);

    // Enable force events at runtime on the already-touching pair: the
    // resting support force (~m*g > 0) must now fire an event every step.
    let co = world.colliders.get_mut(ball_collider).unwrap();
    co.set_active_events(ActiveEvents::CONTACT_FORCE_EVENTS);
    co.set_contact_force_event_threshold(0.0);
    for _ in 0..2 {
        world.step_with_events(&(), &events);
    }
    let after_enable = events.0.load(Ordering::Relaxed);
    assert!(
        after_enable >= 2,
        "no force events after runtime enable: {after_enable}"
    );

    // Disable again: no further events.
    world
        .colliders
        .get_mut(ball_collider)
        .unwrap()
        .set_active_events(ActiveEvents::empty());
    for _ in 0..2 {
        world.step_with_events(&(), &events);
    }
    assert_eq!(events.0.load(Ordering::Relaxed), after_enable);
}
