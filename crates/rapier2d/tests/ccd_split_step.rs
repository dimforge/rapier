//! Regression tests for the steps the CCD splits into several passes (`max_ccd_substeps > 1`):
//! what predicts the next step uses the full step's `dt`, not the length of its last pass.

use rapier2d::prelude::*;

/// Whether the broad-phase leaf of `co` contains the point `p`.
fn leaf_reaches(world: &PhysicsWorld, co: ColliderHandle, p: Vector) -> bool {
    world
        .intersect_aabb_conservative(Aabb::new(p, p), QueryFilter::default())
        .any(|(handle, _)| handle == co)
}

/// A fixed wall at the origin and a fast CCD ball that hits it `hit_time` into the steps
/// (in units of `dt`), far below the rest of the scene.
fn wall_and_ccd_ball(world: &mut PhysicsWorld, hit_time: Real) -> RigidBodyHandle {
    let dt = world.integration_parameters.dt;
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -20.0)),
        ColliderBuilder::cuboid(0.1, 2.0),
    );
    let speed = 200.0;
    world
        .insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(-0.2 - hit_time * speed * dt, -20.0))
                .linvel(Vector::new(speed, 0.0))
                .ccd_enabled(true),
            ColliderBuilder::ball(0.1),
        )
        .0
}

/// The end-of-step broad-phase AABB of a soft-CCD body covers its soft-CCD prediction over the
/// full coming step, even when the CCD split the step and its last pass was short.
#[test]
fn soft_ccd_aabb_spans_the_full_step_when_the_ccd_splits_it() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.max_ccd_substeps = 2;
    let dt = world.integration_parameters.dt;
    // Hit late in the second step: the first pass stops at the impact, the last one only covers
    // a tenth of the step.
    wall_and_ccd_ball(&mut world, 1.8);
    let speed = 120.0;
    let (_, co) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 20.0))
            .linvel(Vector::new(speed, 0.0))
            .soft_ccd_prediction(10.0),
        ColliderBuilder::ball(0.1),
    );

    world.step();
    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 2);
    let aabb = world.colliders[co].compute_aabb();
    let reach = speed * dt;
    let p = Vector::new(aabb.maxs.x + 0.9 * reach, aabb.center().y);
    assert!(
        leaf_reaches(&world, co, p),
        "the broad-phase leaf misses the soft-CCD prediction at {p:?}"
    );
}

/// A CCD body inserted already moving fast gets its first step split at its impact, like any
/// later step: the pre-solve CCD activation reads its current velocity, not the (zero) motion
/// solved on a previous step it didn't take part in.
#[test]
fn first_step_of_a_fast_ccd_body_is_split_at_its_impact() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.max_ccd_substeps = 2;
    // Hit 80% into the first step.
    let ball = wall_and_ccd_ball(&mut world, 0.8);

    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 2);
    let x = world.bodies[ball].translation().x;
    // Touching the wall at -0.2, the contact's softness letting it sink in a little.
    assert!(
        (-0.21..-0.15).contains(&x),
        "the ball didn't stop at the wall: x = {x}"
    );
    // The pass after the impact solved the contact: the ball no longer approaches the wall.
    let vx = world.bodies[ball].linvel().x;
    assert!(vx < 1.0, "the ball still approaches the wall at {vx}");
}

/// The contact history driving the impact-adaptive substeps of a soft body covers its last two
/// steps, however many passes the CCD splits them into: a contact on the step before a split
/// step still counts.
#[test]
fn soft_contact_history_covers_steps_not_ccd_passes() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.max_ccd_substeps = 2;
    let max_extra = world.integration_parameters.soft_bodies.max_extra_substeps;
    // Splits the second step, far from the square.
    wall_and_ccd_ball(&mut world, 1.8);
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05);
    let h = world.insert_soft_body(square);
    let root = world.soft_bodies[h].root_body();
    // Fast enough to request the most substeps while in contact (10 substeps to travel one
    // particle radius per substep).
    let velocity = Vector::new(30.0, 0.0);
    let sb = &mut world.soft_bodies[h];
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, velocity);
    }
    // A ball moving along with the square, touching its leading side.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.8, 1.0))
            .linvel(velocity),
        ColliderBuilder::ball(0.3),
    );

    world.step();
    assert_eq!(world.bodies[root].additional_solver_iterations(), max_extra);
    // No contact from now on: the first step's still counts after the (split) second one.
    world.remove_body(ball);
    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 2);
    assert_eq!(world.bodies[root].additional_solver_iterations(), max_extra);
    // And no longer after a third step.
    world.step();
    assert!(world.bodies[root].additional_solver_iterations() < max_extra);
}
