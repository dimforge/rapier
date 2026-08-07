//! Regression test for https://github.com/dimforge/rapier/issues/477
//!
//! Toggling body types and enabled-flags on colliding bodies used to panic on an
//! out-of-bounds `body_masks` index, because those changes left `active_set_offset`
//! inconsistent.

use rapier2d::prelude::*;

#[test]
fn body_type_and_enabled_toggle_stress_does_not_panic() {
    let mut world = PhysicsWorld::new();

    let ground = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)));
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5),
        ground,
        &mut world.bodies,
    );

    // Two overlapping/stacked colliding bodies.
    let a = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5)));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5), a, &mut world.bodies);
    let b = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.4)));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5), b, &mut world.bodies);

    for i in 0..100 {
        match i % 4 {
            0 => {
                world.bodies[a].set_body_type(RigidBodyType::Fixed, true);
            }
            1 => {
                world.bodies[b].set_enabled(false);
            }
            2 => {
                world.bodies[a].set_body_type(RigidBodyType::Dynamic, true);
            }
            _ => {
                world.bodies[b].set_enabled(true);
            }
        }
        world.step();
    }
}
