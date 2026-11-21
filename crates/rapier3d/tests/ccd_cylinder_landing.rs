//! A fast cylinder stopped by CCD touches the ground at one point of its cap's rim, with the
//! rest of the cap a few centimeters away and still closing in. Its contact manifold has more
//! points than the solver takes, and the reduction used to keep only the touching ones, so the
//! cylinder pivoted on that point and went through the ground.

use rapier3d::prelude::*;

#[test]
fn fast_cylinder_lands_on_the_ground() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(500.0, 0.5, 500.0),
    );
    // Thrown at 300 m/s, mostly downwards, and spinning.
    let (cylinder, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(18.0, 9.694658, 6.0))
            .linvel(Vector::new(69.13812, -287.58572, -50.143482))
            .angvel(Vector::new(-2.5501466, 6.4082184, 2.348007))
            .ccd_enabled(true),
        ColliderBuilder::cylinder(0.5, 0.5).density(500.0),
    );
    for _ in 0..60 {
        world.step();
        let y = world.bodies[cylinder].translation().y;
        assert!(y > 0.0, "the cylinder went through the ground (y = {y})");
    }
}
