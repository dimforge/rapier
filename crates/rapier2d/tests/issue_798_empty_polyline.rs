//! Regression test for issue #798: a polyline collider with zero vertices used to hit an
//! `unreachable!` (index underflow while building segment indices) inside parry. An empty
//! polyline is now simply a collider with no segments.

use rapier2d::prelude::*;

#[test]
fn empty_polyline_collider_steps_and_queries_without_panicking() {
    let mut world = PhysicsWorld::new();

    world.insert_collider(ColliderBuilder::polyline(vec![], None), None);

    // A ball falling right where the empty polyline sits: narrow-phase pairs, queries,
    // and the broad phase must all cope with the segment-less shape.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0)),
        ColliderBuilder::ball(0.5),
    );

    for _ in 0..10 {
        world.step();
    }

    // No contact is expected (there is no geometry), and nothing must panic.
    assert!(
        world.bodies[ball].translation().y < 1.0,
        "ball should fall freely"
    );

    let ray = Ray::new(Vector::new(0.0, 10.0), Vector::new(0.0, -1.0));
    let hit = world.cast_ray(&ray, Real::MAX, true, QueryFilter::default());
    // Only the ball can be hit; the empty polyline must simply never match.
    if let Some((handle, _)) = hit {
        assert_eq!(world.colliders[handle].parent(), Some(ball));
    }
}
