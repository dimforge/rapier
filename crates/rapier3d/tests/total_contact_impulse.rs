//! The impulse reported on a contact must be the total impulse the solver applied during the
//! step (warm-start impulses included), whatever `IntegrationParameters::warmstart_coefficient`
//! is set to. A body resting under gravity pins that value exactly: its velocity is unchanged
//! over the step, so the contacts must have countered gravity with `m * g * dt`.

use rapier3d::pipeline::PhysicsWorld;
use rapier3d::prelude::*;

const GRAVITY: Real = 9.81;
const MASS: Real = 1.0;

/// Reported contact impulse magnitude after the body has come to rest.
fn resting_impulse(warmstart_coefficient: Real, multibody: bool, cuboid: bool) -> Real {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.warmstart_coefficient = warmstart_coefficient;
    world.insert_collider(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
        None,
    );

    let shape = if cuboid {
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
    } else {
        ColliderBuilder::ball(0.5)
    };
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .additional_mass(MASS)
            .can_sleep(false),
        shape.density(0.0),
    );

    if multibody {
        // Attach the body to the ground through a multibody joint so its contacts go through
        // the generic (multibody) constraint path instead of the SIMD one.
        let anchor =
            world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 10.0, 0.0)));
        let joint = PrismaticJointBuilder::new(Vector::Y)
            .local_anchor1(Vector::ZERO)
            .local_anchor2(Vector::ZERO);
        world
            .multibody_joints
            .insert(anchor, body, joint, true)
            .unwrap();
    }

    for _ in 0..300 {
        world.step();
    }

    world
        .narrow_phase
        .contact_pairs()
        .map(|p| p.total_impulse_magnitude())
        .sum()
}

#[test]
fn resting_impulse_matches_gravity_for_any_warmstart_coefficient() {
    let expected = MASS * GRAVITY / 60.0; // m * g * dt, with the default 60 Hz timestep.

    for &multibody in &[false, true] {
        for &cuboid in &[false, true] {
            for &coeff in &[1.0, 0.5, 0.0] {
                let impulse = resting_impulse(coeff, multibody, cuboid);
                assert!(
                    (impulse - expected).abs() <= expected * 1.0e-2,
                    "warmstart_coefficient = {coeff} (multibody: {multibody}, cuboid: {cuboid}): \
                     reported {impulse}, expected {expected}"
                );
            }
        }
    }
}
