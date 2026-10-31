//! A partially consumed `iter_mut` must not lose the modifications made before it.

use rapier3d::prelude::*;

#[test]
fn partial_iter_mut_keeps_earlier_body_modifications() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.insert(RigidBodyBuilder::dynamic(), ColliderBuilder::ball(0.5));
    let (b, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(5.0, 0.0, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    world.step();

    // `b` is modified (and listed), then an iterator visits only the first body.
    let mass_before = world.bodies[b].mass();
    world.bodies[b].set_additional_mass(5.0, true);
    let _ = world.bodies.iter_mut().next();
    world.step();
    // The pipeline saw the edit: the mass properties were recomputed.
    assert!((world.bodies[b].mass() - (mass_before + 5.0)).abs() < 1.0e-4);
}

#[test]
fn partial_iter_mut_keeps_earlier_collider_modifications() {
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(1.0, 0.1, 1.0), None);
    let other = world.insert_collider(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(10.0, 0.0, 0.0)),
        None,
    );
    world.step();

    world.colliders[other].set_translation(Vector::new(20.0, 0.0, 0.0));
    let _ = world.colliders.iter_mut().next();
    world.step();

    // The partial iteration must not leave the collider flagged but unlisted, which drops its
    // later edits: a ray at the latest pose hits.
    world.colliders[other].set_translation(Vector::new(30.0, 0.0, 0.0));
    world.step();
    let ray = Ray::new(Vector::new(30.0, 5.0, 0.0), Vector::new(0.0, -1.0, 0.0));
    let hit = world
        .query_pipeline()
        .cast_ray(&ray, 10.0, true)
        .map(|(h, _)| h);
    assert_eq!(hit, Some(other));
}
