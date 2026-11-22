//! Two round cylinders resting cap on cap have a flat contact where every point is equally deep
//! up to rounding, and the "deepest" one can be the point at the cap's center. Reducing the
//! manifold to four points around that center left part of the cap without support, so a stack
//! of ten round cylinders toppled.

use rapier3d::prelude::*;

#[test]
fn round_cylinder_stack_stands() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(200.0, 0.5, 200.0),
    );
    let mut stack = vec![];
    for i in 0..10 {
        let y = 0.5 + 1.001 * i as Real + 0.001;
        let (h, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y, 0.0)),
            ColliderBuilder::round_cylinder(0.4, 0.4, 0.1),
        );
        stack.push(h);
    }
    for _ in 0..600 {
        world.step();
    }
    let top = world.bodies[*stack.last().unwrap()].translation();
    let drift = (top.x * top.x + top.z * top.z).sqrt();
    assert!(
        drift < 0.01,
        "the top of the stack moved {drift} m sideways"
    );
    assert!(
        stack.iter().all(|h| world.bodies[*h].is_sleeping()),
        "the stack is still moving"
    );
}
