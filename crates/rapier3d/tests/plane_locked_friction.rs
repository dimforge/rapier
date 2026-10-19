//! A body locked to a plane (no movement along z, no turning about x or y) must keep its
//! friction. That lock keeps 3D bodies in a plane, and is how 2D bodies are run inside a 3D
//! world. The locked tangent has no effective mass, which made the 2×2 tangent matrix
//! singular; the coupled solve then zeroed friction along both tangents and the body slid as
//! if on ice.

use rapier3d::prelude::*;

/// How far a unit cube resting on a floor slides after a 3 m/s shove along x.
fn slide(locked_to_plane: bool, model: FrictionModel, settle_first: bool) -> f32 {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.friction_model = model;

    let ground = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0).friction(0.5),
        ground,
        &mut world.bodies,
    );

    let mut body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0));
    if !settle_first {
        body = body.linvel(Vector::new(3.0, 0.0, 0.0));
    }
    if locked_to_plane {
        body = body
            .enabled_translations(true, true, false)
            .enabled_rotations(false, false, true);
    }
    let handle = world.bodies.insert(body);
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).friction(0.5),
        handle,
        &mut world.bodies,
    );

    if settle_first {
        for _ in 0..30 {
            world.step();
        }
        world.bodies[handle].set_linvel(Vector::new(3.0, 0.0, 0.0), true);
    }
    for _ in 0..180 {
        world.step();
    }
    world.bodies[handle].translation().x
}

#[test]
fn plane_locked_body_keeps_its_friction() {
    // Report every case before judging, to see which ones break.
    let mut bad = Vec::new();
    for model in [FrictionModel::Simplified, FrictionModel::Coulomb] {
        for settle_first in [true, false] {
            let free = slide(false, model, settle_first);
            let locked = slide(true, model, settle_first);
            println!(
                "{model:?} settle_first={settle_first}: free {free:.2} m, locked {locked:.2} m"
            );
            // v² / (2μg) ≈ 0.92 m; both stop well before 2 m.
            if free >= 2.0 || locked >= 2.0 || (free - locked).abs() >= 0.25 {
                bad.push(format!(
                    "{model:?} settle_first={settle_first}: free {free} m, locked {locked} m"
                ));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
