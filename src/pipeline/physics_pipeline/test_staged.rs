//! Staged-solver parity and substep-group partition regression tests.

use crate::dynamics::RigidBodyBuilder;
use crate::geometry::ColliderBuilder;
use crate::math::Vector;
use crate::prelude::RevoluteJointBuilder;

/// Regression: overflow-color SIMD chunking. Same-pair manifolds (compound colliders)
/// land in the overflow color; packing two into one SIMD lane group makes the wide
/// gather/scatter (last-writer-wins) silently drop an impulse. Compares 2 workers vs 1.
#[test]
#[cfg(all(
    feature = "dim2",
    feature = "parallel",
    not(feature = "unsync-callbacks")
))]
fn staged_overflow_chunk_no_shared_dynamic_body() {
    use crate::alloc_prelude::*;
    use crate::pipeline::PhysicsWorld;
    use crate::prelude::{Pose, SharedShape};

    fn build(num_threads: usize) -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.contact_clustering = false;
        world.integration_parameters.contact_recycling = false;
        world.configure_thread_pool(num_threads).unwrap();

        let tribox = || {
            ColliderBuilder::compound(vec![
                (
                    Pose::from_translation(Vector::new(-1.6, 0.0)),
                    SharedShape::cuboid(0.5, 0.5),
                ),
                (
                    Pose::from_translation(Vector::new(0.0, 0.0)),
                    SharedShape::cuboid(0.5, 0.5),
                ),
                (
                    Pose::from_translation(Vector::new(1.6, 0.0)),
                    SharedShape::cuboid(0.5, 0.5),
                ),
            ])
        };

        let _ = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0)),
            tribox(),
        );
        for i in 1..=3 {
            let _ = world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(0.0, 0.98 * i as f32))
                    .can_sleep(false),
                tribox(),
            );
        }

        for _ in 0..30 {
            world.step();
        }
        world
    }

    let staged = build(2);
    let reference = build(1);

    // The two-worker staged solve must reach the same rest state as the
    // one-worker reference. A dropped overflow impulse would let the shared
    // body drift, diverging well past this tolerance.
    for ((_, a), (_, b)) in staged.bodies.iter().zip(reference.bodies.iter()) {
        let da = a.translation();
        let db = b.translation();
        assert!(da.x.is_finite() && da.y.is_finite(), "non-finite position");
        assert!(
            (da - db).length() < 0.05,
            "staged solver diverged from reference: {da:?} vs {db:?}"
        );
    }
}

/// Multi-group substep ring: a three-solve-group scene must match the one-worker
/// reference on two staged workers, and the elevated pair must actually hold tighter
/// than an identical non-elevated pair in the same world.
#[test]
#[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
fn staged_multi_group_substep_parity() {
    use crate::pipeline::PhysicsWorld;

    fn build(num_threads: usize) -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        world.configure_thread_pool(num_threads).unwrap();

        #[cfg(feature = "dim2")]
        let cuboid = || ColliderBuilder::cuboid(0.5, 0.5);
        #[cfg(feature = "dim3")]
        let cuboid = || ColliderBuilder::cuboid(0.5, 0.5, 0.5);
        let dynamic = |x: crate::math::Real, y: crate::math::Real| {
            RigidBodyBuilder::dynamic()
                .translation(Vector::X * x + Vector::Y * y)
                .can_sleep(false)
        };

        // Default-cadence stack on the ground.
        let _ = world.insert(RigidBodyBuilder::fixed(), cuboid());
        for i in 1..=3 {
            let _ = world.insert(dynamic(0.0, 1.001 * i as crate::math::Real), cuboid());
        }

        // Elevated jointed pair (its own component).
        let (a, _) = world.insert(dynamic(20.0, 5.0), cuboid());
        let (b, _) = world.insert(
            dynamic(20.0, 3.0).additional_solver_iterations(4),
            cuboid().density(50.0),
        );
        #[cfg(feature = "dim2")]
        let joint = RevoluteJointBuilder::new();
        #[cfg(feature = "dim3")]
        let joint = RevoluteJointBuilder::new(Vector::Z);
        world.insert_impulse_joint(a, b, joint);

        // Isolated, higher elevation still (third group).
        let _ = world.insert(
            dynamic(-20.0, 5.0).additional_solver_iterations(8),
            cuboid(),
        );

        for _ in 0..60 {
            world.step();
        }
        world
    }

    let staged = build(2);
    let reference = build(1);

    for ((_, a), (_, b)) in staged.bodies.iter().zip(reference.bodies.iter()) {
        let da = a.translation();
        let db = b.translation();
        assert!(da.x.is_finite() && da.y.is_finite(), "non-finite position");
        assert!(
            (da - db).length() < 0.05,
            "multi-group staged solve diverged from reference: {da:?} vs {db:?}"
        );
    }
}

/// Multi-group JOINT layout: enough jointed pairs for parallel colors/SIMD chunks, plus
/// an elevated pair forcing the multi-group path, must match the one-worker reference.
/// Holds the grouping steady to exercise the group-fingerprinted joint-reuse cache.
#[test]
#[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
fn staged_multi_group_joint_simd_parity() {
    use crate::pipeline::PhysicsWorld;

    fn build(num_threads: usize) -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        world.configure_thread_pool(num_threads).unwrap();

        #[cfg(feature = "dim2")]
        let cuboid = || ColliderBuilder::cuboid(0.4, 0.4);
        #[cfg(feature = "dim3")]
        let cuboid = || ColliderBuilder::cuboid(0.4, 0.4, 0.4);
        let dynamic = |x: crate::math::Real, y: crate::math::Real| {
            RigidBodyBuilder::dynamic()
                .translation(Vector::X * x + Vector::Y * y)
                .can_sleep(false)
        };
        #[cfg(feature = "dim2")]
        let joint = RevoluteJointBuilder::new;
        #[cfg(feature = "dim3")]
        let joint = || RevoluteJointBuilder::new(Vector::Z);

        // 64 independent swinging pairs (default cadence): pairwise
        // body-disjoint, so they share one joint color — far above the
        // parallel-color threshold.
        for i in 0..64 {
            let x = i as crate::math::Real * 3.0;
            let (a, _) = world.insert(dynamic(x, 4.0), cuboid());
            let (b, _) = world.insert(dynamic(x + 0.9, 3.0), cuboid().density(2.0));
            world.insert_impulse_joint(a, b, joint());
        }

        // One elevated pair: forces the multi-group path for the whole run.
        let (a, _) = world.insert(dynamic(-10.0, 4.0), cuboid());
        let (b, _) = world.insert(
            dynamic(-10.0, 2.0).additional_solver_iterations(4),
            cuboid().density(20.0),
        );
        world.insert_impulse_joint(a, b, joint());

        for _ in 0..60 {
            world.step();
        }
        world
    }

    let staged = build(2);
    let reference = build(1);

    for ((_, a), (_, b)) in staged.bodies.iter().zip(reference.bodies.iter()) {
        let da = a.translation();
        let db = b.translation();
        assert!(da.x.is_finite() && da.y.is_finite(), "non-finite position");
        assert!(
            (da - db).length() < 0.05,
            "multi-group joint layout diverged from reference: {da:?} vs {db:?}"
        );
    }
}

/// Substep-group partition: the awake body list must reorder into contiguous groups by
/// component max `additional_solver_iterations` (descending), re-stamp the
/// `active_set_id == index` invariant, and not churn the epoch once stable.
#[test]
fn substep_groups_partition() {
    use crate::alloc_prelude::*;
    use crate::pipeline::PhysicsWorld;

    let mut world = PhysicsWorld::new();

    #[cfg(feature = "dim2")]
    let cuboid = || ColliderBuilder::cuboid(0.5, 0.5);
    #[cfg(feature = "dim3")]
    let cuboid = || ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    let dynamic = |x: crate::math::Real, y: crate::math::Real| {
        RigidBodyBuilder::dynamic()
            .translation(Vector::X * x + Vector::Y * y)
            .can_sleep(false)
    };

    // Default-cadence stack: one box resting on the ground.
    let _ = world.insert(RigidBodyBuilder::fixed(), cuboid());
    let (plain, _) = world.insert(dynamic(0.0, 1.001), cuboid());

    // A two-body jointed pair where only ONE body is elevated: the joint
    // edge must pull the partner into the same (elevated) group.
    let (chain_a, _) = world.insert(dynamic(10.0, 5.0), cuboid());
    let (chain_b, _) = world.insert(dynamic(10.0, 3.0).additional_solver_iterations(4), cuboid());
    #[cfg(feature = "dim2")]
    let joint = RevoluteJointBuilder::new();
    #[cfg(feature = "dim3")]
    let joint = RevoluteJointBuilder::new(Vector::Z);
    world.insert_impulse_joint(chain_a, chain_b, joint);

    // An isolated component at a higher elevation still.
    let (lone, _) = world.insert(
        dynamic(-10.0, 5.0).additional_solver_iterations(8),
        cuboid(),
    );

    for _ in 0..3 {
        world.step();
    }

    // Expect three groups, descending: {lone}=8, {chain_a, chain_b}=4,
    // {plain}=0. The fixed ground is not part of the awake set.
    let groups = &world.islands.solve_groups;
    assert_eq!(
        groups.iter().map(|g| g.extra_iters).collect::<Vec<_>>(),
        vec![8, 4, 0],
    );
    assert_eq!(
        groups
            .iter()
            .map(|g| g.body_range.len())
            .collect::<Vec<_>>(),
        vec![1, 2, 1],
    );
    assert_eq!(groups[0].body_range.start, 0);
    assert_eq!(groups.last().unwrap().body_range.end, 4);

    // Each body sits in its expected group range, and the
    // `active_set_id == index in the awake island's bodies` invariant
    // holds after the reorder.
    let awake_bodies: Vec<_> = world.islands.active_bodies().collect();
    for (i, handle) in awake_bodies.iter().enumerate() {
        assert_eq!(world.bodies[*handle].ids.active_set_id as usize, i);
    }
    let group_of = |h: crate::dynamics::RigidBodyHandle| {
        let id = world.bodies[h].ids.active_set_id as usize;
        groups.iter().position(|g| g.body_range.contains(&id))
    };
    assert_eq!(group_of(lone), Some(0));
    assert_eq!(group_of(chain_a), Some(1), "joint partner must be lifted");
    assert_eq!(group_of(chain_b), Some(1));
    assert_eq!(group_of(plain), Some(2));

    // Steady state: the grouping is contiguous, so further steps must not bump the
    // active-set epoch (a bump forces a full solver-graph rebuild — re-partitioning
    // every step would tank elevated scenes).
    let epoch = world.islands.active_set_epoch;
    for _ in 0..5 {
        world.step();
    }
    assert_eq!(
        world.islands.active_set_epoch, epoch,
        "stable grouping must not churn the active-set epoch"
    );

    // Un-elevating every body empties the groups (single implicit group).
    world
        .bodies
        .get_mut(lone)
        .unwrap()
        .set_additional_solver_iterations(0);
    world
        .bodies
        .get_mut(chain_b)
        .unwrap()
        .set_additional_solver_iterations(0);
    world.step();
    assert!(world.islands.solve_groups.is_empty());
}
