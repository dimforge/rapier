//! The CCD fixed-target cache must not outlive the colliders it points at.
//!
//! The non-bullet continuous pass sweeps against a cached list of fixed targets, held as
//! `ColliderHandle`s from one step to the next. The scene-change signal that invalidates it
//! is rebuilt from the per-step user-change lists, which are drained whether or not the CCD
//! pass runs — so a removal on a step where nothing moved fast enough used to be forgotten,
//! leaving the cache holding a dead handle for the next fast body to resolve.

use rapier2d::prelude::*;

/// A fixed collider removed on a step without any CCD-active body must not linger in the
/// cached fixed-target list.
#[test]
fn fixed_collider_removed_on_a_ccd_less_step_leaves_no_stale_target() {
    let mut world = PhysicsWorld::new();

    // Two parentless (hence fixed-target) colliders. `stale` sits on the path `faller`
    // takes down to the ground, so a leftover entry is guaranteed to be resolved rather
    // than skipped by the swept-AABB test.
    world.insert_collider(
        ColliderBuilder::cuboid(100.0, 0.5).translation(Vector::new(0.0, 0.0)),
        None,
    );
    let stale = world.insert_collider(
        ColliderBuilder::cuboid(2.0, 2.0).translation(Vector::new(10.0, 25.0)),
        None,
    );

    // Moving fast from the first step: forces a continuous pass, which is what populates
    // the fixed-target cache.
    let fast = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0))
            .linvel(Vector::new(0.0, -400.0)),
    );
    world.insert_collider(ColliderBuilder::ball(0.2), Some(fast));

    // Starts at rest and only becomes CCD-active later, from gravity alone — no user change
    // marks the step it wakes the continuous pass back up.
    let faller =
        world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(10.0, 60.0)));
    world.insert_collider(ColliderBuilder::ball(0.05), Some(faller));

    world.step();

    // Take the fast body out so the next few steps skip the continuous pass entirely.
    world.remove_body(fast);
    world.step();

    // The removal whose invalidation signal used to be dropped: `faller` is still slow, so
    // no CCD pass runs this step to notice it.
    world.remove_collider(stale);
    world.step();

    // Nothing is touched from here on: `faller` accelerates until it is CCD-active, and the
    // pass it triggers resolves the cached fixed targets.
    for _ in 0..400 {
        world.step();
    }

    // It fell all the way through where `stale` used to be and settled on the ground.
    let y = world.bodies[faller].translation().y;
    assert!(
        (y - 0.55).abs() < 0.1,
        "faller should rest on the ground at y ≈ 0.55, got {y}"
    );
}
