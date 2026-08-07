//! Regression test for https://github.com/dimforge/rapier/issues/749
//!
//! The old owned `QueryPipeline` never pruned removed colliders, so queries could hit stale
//! leaves unless the user called `update` manually. It is now a zero-copy view over the
//! broad-phase BVH, which removes leaves eagerly.

use rapier3d::prelude::*;

#[test]
fn queries_after_removals_hit_no_stale_leaves() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 0.0, 0.0);

    // Spawn/step/remove churn: bodies at well-separated positions.
    let mut old_positions = Vec::new();
    for i in 0..20 {
        let pos = Vector::new(i as f32 * 10.0, 0.0, 0.0);
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(pos),
            ColliderBuilder::ball(1.0),
        );
        world.step();
        world.remove_body(body);
        world.step();
        old_positions.push(pos);
    }

    // One survivor so the tree is non-empty (the interesting case for stale leaves).
    let survivor_pos = Vector::new(0.0, 100.0, 0.0);
    let (_, survivor) = world.insert(
        RigidBodyBuilder::dynamic().translation(survivor_pos),
        ColliderBuilder::ball(1.0),
    );
    world.step();

    for pos in &old_positions {
        // Raycast straight at each removed collider's old position: no hit, no panic.
        let ray = Ray::new(
            *pos + Vector::new(0.0, 5.0, 0.0),
            Vector::new(0.0, -1.0, 0.0),
        );
        let hit = world.cast_ray(&ray, 10.0, true, QueryFilter::default());
        assert!(
            hit.is_none(),
            "ray at removed collider's old position {pos:?} hit {hit:?}"
        );

        // Point projection near the old position must only ever find the survivor.
        if let Some((hit, _)) = world.project_point(*pos, 5.0, true, QueryFilter::default()) {
            assert_eq!(hit, survivor, "projection found a removed collider");
        }
    }

    // The survivor is still queryable.
    let ray = Ray::new(
        survivor_pos + Vector::new(0.0, 5.0, 0.0),
        Vector::new(0.0, -1.0, 0.0),
    );
    let hit = world.cast_ray(&ray, 10.0, true, QueryFilter::default());
    assert_eq!(hit.map(|(h, _)| h), Some(survivor));
}
