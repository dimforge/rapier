//! Structural tests for the persistent islands: eager merge on
//! touch/joint-link, deferred split (one island per step) after constraint
//! removals, and link refreshes on body lifecycle edits.
//!
//! These tests observe island *equality* between bodies through the
//! test-only `IslandManager::persistent_island_of` accessor; the heavy
//! structural invariants are asserted by the debug-build validation that runs
//! inside every `world.step()`.

use rapier3d::pipeline::PhysicsWorld;
use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let ground = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0));
    world.insert(ground, ColliderBuilder::cuboid(100.0, 0.5, 100.0));
    world
}

fn island_of(world: &PhysicsWorld, h: RigidBodyHandle) -> Option<u32> {
    world.islands.persistent_island_of(&world.bodies, h)
}

fn same_island(world: &PhysicsWorld, h1: RigidBodyHandle, h2: RigidBodyHandle) -> bool {
    let i1 = island_of(world, h1);
    i1.is_some() && i1 == island_of(world, h2)
}

fn insert_box(world: &mut PhysicsWorld, x: Real, y: Real) -> RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(x, y, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0
}

/// Long enough for resting bodies to become sleep-eligible (1s = 60 steps),
/// bid the pending split, and have it run.
const SETTLE_STEPS: usize = 240;

/// Two boxes stacked touch and must share an island; two distant boxes must
/// not. After the stack's top is teleported away, the settled leftovers must
/// end up in distinct islands (deferred split).
#[test]
fn merge_on_touch_split_on_separation() {
    let mut world = world_with_ground();
    let bottom = insert_box(&mut world, 0.0, 0.5);
    let top = insert_box(&mut world, 0.0, 1.5);
    let lone = insert_box(&mut world, 20.0, 0.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        same_island(&world, bottom, top),
        "touching boxes must merge"
    );
    assert!(
        !same_island(&world, bottom, lone),
        "distant boxes must not share an island"
    );

    // Teleport the top box far away: the pair is removed, and once everything
    // settles the island must have been split.
    world.bodies[top].set_translation(Vector::new(40.0, 0.5, 0.0), true);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        !same_island(&world, bottom, top),
        "separated boxes must end up in distinct islands after the deferred split"
    );
}

/// A joint between two distant resting boxes merges their islands; removing
/// it (without waking) must eventually split them apart again.
#[test]
fn joint_links_and_split_after_removal() {
    let mut world = world_with_ground();
    let a = insert_box(&mut world, 0.0, 0.5);
    let b = insert_box(&mut world, 20.0, 0.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(!same_island(&world, a, b));

    let joint = world.impulse_joints.insert(
        a,
        b,
        RopeJointBuilder::new(30.0)
            .local_anchor1(Vector::ZERO)
            .local_anchor2(Vector::ZERO),
        true,
    );
    world.step();
    assert!(same_island(&world, a, b), "a joint must merge islands");

    world.impulse_joints.remove(joint, true);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        !same_island(&world, a, b),
        "the deferred split must separate joint-disconnected islands"
    );
}

/// Removing the middle box of a touching row must split the sides apart.
#[test]
fn body_removal_splits_row() {
    let mut world = world_with_ground();
    let left = insert_box(&mut world, 0.0, 0.5);
    let middle = insert_box(&mut world, 1.0, 0.5);
    let right = insert_box(&mut world, 2.0, 0.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(same_island(&world, left, right));

    world.remove_body(middle);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        !same_island(&world, left, right),
        "removing the bridging body must split the island"
    );
}

/// A separation is resolved by the *local* search, in the very step the contact
/// stops touching — no waiting for a body to become sleep-eligible, bid for the
/// split, and have the deferred global union-find run it.
#[test]
fn separation_splits_immediately() {
    let mut world = world_with_ground();
    let bottom = insert_box(&mut world, 0.0, 0.5);
    let top = insert_box(&mut world, 0.0, 1.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(same_island(&world, bottom, top));

    // Teleport the top box far away. The pair stops touching during this step's
    // narrow phase, and the local search must peel it off before the step ends.
    world.bodies[top].set_translation(Vector::new(40.0, 0.5, 0.0), true);
    world.step();
    assert!(
        !same_island(&world, bottom, top),
        "the local split must separate them in the very step they stop touching"
    );
}

/// A detached *multi-body* component (not just a lone straggler) is moved out
/// whole, with the links between its members following it.
#[test]
fn detached_chunk_moves_out_with_its_links() {
    let mut world = world_with_ground();
    // A row of 6 touching boxes: [0][1][2] | [3][4][5].
    let row: Vec<_> = (0..6)
        .map(|i| insert_box(&mut world, i as Real * 1.0, 0.5))
        .collect();

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(same_island(&world, row[0], row[5]), "the row is one island");

    // Lift the right half away as a block: it stays internally touching, so the
    // island splits into two *multi-body* components rather than shedding
    // singletons.
    for (i, handle) in row.iter().enumerate().skip(3) {
        let x = 30.0 + (i - 3) as Real;
        world.bodies[*handle].set_translation(Vector::new(x, 0.5, 0.0), true);
    }
    for _ in 0..SETTLE_STEPS {
        world.step();
    }

    assert!(
        !same_island(&world, row[0], row[3]),
        "the two halves must end up in different islands"
    );
    assert!(
        same_island(&world, row[0], row[2]),
        "the left half stays one island"
    );
    assert!(
        same_island(&world, row[3], row[5]),
        "the detached half moves out as one island, links included"
    );
}

/// Turning the middle box fixed removes it from the islands (fixed bodies
/// never belong to one) and must split the sides; turning it dynamic again
/// must re-merge everything.
#[test]
fn type_change_bridges_and_unbridges() {
    let mut world = world_with_ground();
    let left = insert_box(&mut world, 0.0, 0.5);
    let middle = insert_box(&mut world, 1.0, 0.5);
    let right = insert_box(&mut world, 2.0, 0.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(same_island(&world, left, right));

    world.bodies[middle].set_body_type(RigidBodyType::Fixed, true);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        island_of(&world, middle),
        None,
        "fixed bodies have no island"
    );
    assert!(
        !same_island(&world, left, right),
        "a fixed body doesn't connect its neighbors"
    );

    world.bodies[middle].set_body_type(RigidBodyType::Dynamic, true);
    for _ in 0..8 {
        world.step();
    }
    assert!(
        same_island(&world, left, right),
        "back to dynamic, the middle body must re-merge the row"
    );
}

/// Disabling the middle box must split the row; re-enabling must re-merge it.
#[test]
fn disable_enable_bridges_and_unbridges() {
    let mut world = world_with_ground();
    let left = insert_box(&mut world, 0.0, 0.5);
    let middle = insert_box(&mut world, 1.0, 0.5);
    let right = insert_box(&mut world, 2.0, 0.5);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(same_island(&world, left, right));

    world.bodies[middle].set_enabled(false);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(island_of(&world, middle), None);
    assert!(!same_island(&world, left, right));

    world.bodies[middle].set_enabled(true);
    for _ in 0..8 {
        world.step();
    }
    assert!(same_island(&world, left, right));
}

/// Two multibody branches hanging under a *fixed* root must share one island
/// (multibodies are atomic), even though a joint edge to a fixed body doesn't
/// connect anything by itself. Removing one branch's joint must split it off.
#[test]
fn multibody_fixed_root_branches_share_island() {
    let mut world = world_with_ground();
    let root = world
        .insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, 5.0, 0.0)),
            ColliderBuilder::ball(0.1),
        )
        .0;
    let child_a = insert_box(&mut world, -2.0, 5.0);
    let child_b = insert_box(&mut world, 2.0, 5.0);

    let joint_a = world
        .multibody_joints
        .insert(
            root,
            child_a,
            FixedJointBuilder::new().local_anchor1(Vector::new(-2.0, 0.0, 0.0)),
            true,
        )
        .unwrap();
    let _joint_b = world
        .multibody_joints
        .insert(
            root,
            child_b,
            FixedJointBuilder::new().local_anchor1(Vector::new(2.0, 0.0, 0.0)),
            true,
        )
        .unwrap();

    world.step();
    assert!(
        same_island(&world, child_a, child_b),
        "both branches of a fixed-root multibody must share an island"
    );
    assert_eq!(island_of(&world, root), None);

    world.multibody_joints.remove(joint_a, true);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        !same_island(&world, child_a, child_b),
        "a detached branch must split off"
    );
}
