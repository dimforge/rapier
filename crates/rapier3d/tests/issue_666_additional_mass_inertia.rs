//! Regression test for https://github.com/dimforge/rapier/issues/666
//!
//! `RigidBodyAdditionalMassProps::Mass` with zero collider-derived mass left the angular
//! inertia at zero, so the body could never rotate: a tall cuboid kept sliding instead of
//! toppling like its collider-density twin.

use rapier3d::prelude::*;

/// Builds the issue's scene: a fast tall cuboid dropped onto a rough ground.
/// `use_additional_mass` selects between collider-density mass and
/// `RigidBody::additional_mass` with a massless collider.
fn build_world(use_additional_mass: bool) -> (PhysicsWorld, RigidBodyHandle) {
    let mut world = PhysicsWorld::new();

    let ground_height = 0.1;
    let ground = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0)));
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(100.1, ground_height, 100.1).friction(0.5),
        ground,
        &mut world.bodies,
    );

    let mut body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(-10.0, 6.0, 0.0))
        .linvel(Vector::new(20.0, 0.0, 0.0))
        .can_sleep(false);
    let mut collider = ColliderBuilder::cuboid(0.2, 5.0, 1.5).friction(0.5);

    if use_additional_mass {
        body = body.additional_mass(0.5);
        collider = collider.mass(0.0);
    } else {
        collider = collider.mass(0.5);
    }

    let handle = world.bodies.insert(body);
    world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);
    (world, handle)
}

fn max_rotation_angle(world: &mut PhysicsWorld, handle: RigidBodyHandle) -> f32 {
    let mut max_angle: f32 = 0.0;
    for _ in 0..200 {
        world.step();
        let angle = world.bodies[handle]
            .rotation()
            .angle_between(Rotation::IDENTITY);
        max_angle = max_angle.max(angle);
    }
    max_angle
}

#[test]
fn additional_mass_body_topples_like_density_twin() {
    let (mut density_world, density_body) = build_world(false);
    let (mut added_world, added_body) = build_world(true);

    // The additional-mass body must get a non-zero angular inertia derived from
    // its collider's shape (mass properties are recomputed during the first step).
    added_world.step();
    let mprops = added_world.bodies[added_body].mass_properties();
    assert!(
        mprops.local_mprops.principal_inertia().length() > 0.0,
        "additional_mass must produce a non-zero angular inertia"
    );

    let density_angle = max_rotation_angle(&mut density_world, density_body);
    let added_angle = max_rotation_angle(&mut added_world, added_body);

    // The density-based twin topples; the additional-mass body must too.
    assert!(
        density_angle > 0.5,
        "sanity: density-based body should topple (angle: {density_angle})"
    );
    assert!(
        added_angle > 0.5,
        "additional-mass body slides without toppling (angle: {added_angle})"
    );
}
