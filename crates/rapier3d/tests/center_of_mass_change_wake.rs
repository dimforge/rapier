//! Moving a sleeping body's center of mass without asking for a wake-up keeps it asleep.

use rapier3d::prelude::*;

#[test]
fn center_of_mass_change_without_wake_up_keeps_bodies_asleep() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(5.0, 0.5, 5.0),
    );
    let (bottom, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let (top, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    for _ in 0..500 {
        world.step();
    }
    assert!(world.bodies[bottom].is_sleeping() && world.bodies[top].is_sleeping());

    let props = MassProperties::new(Vector::new(0.05, 0.0, 0.0), 1.0, Vector::splat(0.2));
    world.bodies[top].set_additional_mass_properties(props, false);
    world.step();
    assert!(world.bodies[bottom].is_sleeping() && world.bodies[top].is_sleeping());
}
