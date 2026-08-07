//! Regression test for https://github.com/dimforge/rapier/issues/78
//!
//! A body whose mass came only from `additional_mass` (zero-density collider) used to slide,
//! bounce and eventually tunnel through the floor. It must come to rest like its
//! density-based twin.

use rapier3d::prelude::*;

fn build_world(use_additional_mass: bool) -> (PhysicsWorld, RigidBodyHandle) {
    let mut world = PhysicsWorld::new();

    let ground = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(5.0, 0.5, 5.0).friction(0.5),
        ground,
        &mut world.bodies,
    );

    let mut body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 1.0, 0.0))
        .rotation(AngVector::new(0.0, 0.0, std::f32::consts::FRAC_PI_4 * 0.9));
    let mut collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5)
        .friction(0.5)
        .restitution(0.0);

    if use_additional_mass {
        body = body.additional_mass(100.0);
        collider = collider.density(0.0);
    } else {
        collider = collider.mass(100.0);
    }

    let handle = world.bodies.insert(body);
    world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);
    (world, handle)
}

fn steps_to_rest(world: &mut PhysicsWorld, handle: RigidBodyHandle) -> usize {
    for step in 0..1000 {
        world.step();
        let rb = &world.bodies[handle];
        assert!(
            rb.translation().y > -0.5,
            "body tunneled through the ground at step {step}"
        );
        if rb.is_sleeping() {
            return step;
        }
    }
    panic!("body never came to rest");
}

#[test]
fn additional_mass_body_rests_like_density_twin() {
    let (mut density_world, density_body) = build_world(false);
    let (mut added_world, added_body) = build_world(true);

    let density_steps = steps_to_rest(&mut density_world, density_body);
    let added_steps = steps_to_rest(&mut added_world, added_body);

    let density_y = density_world.bodies[density_body].translation().y;
    let added_y = added_world.bodies[added_body].translation().y;

    // Both must come to rest on top of the ground, at a similar height and in a
    // comparable number of steps (no endless sliding/bouncing).
    assert!(
        (density_y - added_y).abs() < 0.1,
        "resting heights differ: density {density_y} vs additional_mass {added_y}"
    );
    assert!(
        added_steps < density_steps.max(1) * 4,
        "additional_mass body took far longer to rest ({added_steps} vs {density_steps} steps)"
    );
}
