//! A fast body made of many thin colliders, of which only a few come near anything.
//!
//! Thin colliders give a body a small `ccd_thickness`, so it counts as fast (and gets swept by
//! the continuous phase) at quite ordinary speeds. The sweep then visits every collider of the
//! body each step, but only the colliders whose swept AABB meets a target have anything to cast.

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

/// A 5 mm thick plate as a convex polyhedron: a 16-sided outline of radius 0.4 m, extruded.
/// Convex hulls (rather than cuboids) are what a body built from arbitrary cut plates uses,
/// and their mass properties are not trivial to compute.
fn plate_shape() -> SharedShape {
    let half_thickness = 0.0025;
    let mut points = Vec::new();
    for i in 0..16 {
        let a = i as Real * std::f32::consts::TAU / 16.0;
        for y in [-half_thickness, half_thickness] {
            points.push(Vector::new(0.4 * a.cos(), y, 0.4 * a.sin()));
        }
    }
    SharedShape::convex_hull(&points).unwrap()
}

/// Builds a dynamic body out of a `nx * ny * nz` lattice of thin convex plates (5 mm thick,
/// 1 m apart), with its lowest layer `clearance` above a large fixed ground, moving along +X at
/// `speed`. Gravity is off, so the body keeps its velocity unless a contact or CCD stops it.
fn plate_lattice_world(
    nx: usize,
    ny: usize,
    nz: usize,
    clearance: Real,
    linvel: Vector,
    gravity: Vector,
) -> (PhysicsWorld, RigidBodyHandle) {
    let mut world = PhysicsWorld::new();
    world.gravity = gravity;

    let _ground = world.insert_collider(ColliderBuilder::cuboid(500.0, 0.5, 500.0), None);

    let half_thickness = 0.0025;
    let plate = plate_shape();
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5 + clearance + half_thickness, 0.0))
            .linvel(linvel)
            .angvel(Vector::new(0.3, 0.0, 0.2)),
    );
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                world.insert_collider(
                    ColliderBuilder::new(plate.clone()).translation(Vector::new(
                        i as Real - nx as Real / 2.0,
                        j as Real,
                        k as Real - nz as Real / 2.0,
                    )),
                    Some(body),
                );
            }
        }
    }

    (world, body)
}

/// Bit pattern of the whole body state, to compare runs exactly.
fn state_bits(world: &PhysicsWorld) -> Vec<u64> {
    let mut bits = Vec::new();
    for (_, rb) in world.bodies.iter() {
        let pos = rb.position();
        let rot = pos.rotation;
        for v in [
            pos.translation.x,
            pos.translation.y,
            pos.translation.z,
            rot.x,
            rot.y,
            rot.z,
            rot.w,
            rb.linvel().x,
            rb.linvel().y,
            rb.linvel().z,
            rb.angvel().x,
            rb.angvel().y,
            rb.angvel().z,
        ] {
            bits.push(v.to_bits() as u64);
        }
    }
    bits
}

/// A many-collider body whose first colliders (in insertion order) meet nothing must still be
/// stopped by a thin wall that only its *last* collider reaches.
#[test]
fn fast_many_collider_body_is_stopped_by_its_last_collider() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    // A 10 cm thick fixed wall at x = 2.
    let _wall = world.insert_collider(
        ColliderBuilder::cuboid(0.05, 50.0, 50.0).translation(Vector::new(2.0, 0.0, 0.0)),
        None,
    );

    // 100 m/s: 1.67 m per step, far more than the wall thickness.
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.0, 0.0))
            .linvel(Vector::new(100.0, 0.0, 0.0)),
    );
    // 200 colliders well behind the wall's face, in pairs placed symmetrically about the
    // in-line collider (so the impact applies no torque): their swept boxes never meet it.
    for i in 0..100 {
        for side in [-1.0, 1.0] {
            world.insert_collider(
                ColliderBuilder::ball(0.05).translation(Vector::new(
                    -5.0 - (i % 10) as Real,
                    side * (1.0 + (i / 10) as Real),
                    0.0,
                )),
                Some(body),
            );
        }
    }
    // The only collider in line with the wall, inserted last.
    world.insert_collider(ColliderBuilder::ball(0.05), Some(body));

    for _ in 0..30 {
        world.step();
    }

    let x = world.bodies[body].translation().x;
    assert!(
        x < 2.0,
        "body tunnelled through the wall: x = {x} (the in-line collider should stop at x < 1.95)"
    );
}

/// Step time for a fast lattice of 2,000 thin plates: gliding 1 cm above the ground (only the
/// lowest layer's swept boxes can meet it), far from anything, and thrown onto the ground
/// (CCD impacts and contacts). Prints the step times and a hash of the final state, so two
/// builds can be compared for speed and for identical results.
///
/// Timing-based, so `#[ignore]`d: run manually in release with `-- --ignored --nocapture`.
#[test]
#[ignore = "timing-based perf check; run manually in release"]
fn fast_plate_lattice_step_time() {
    // 3 m/s is 5 cm per step: ten times the plates' 5 mm thickness, so the body is fast.
    let glide = Vector::new(3.0, 0.0, 0.0);
    let gravity = Vector::new(0.0, -9.81, 0.0);
    for (label, clearance, linvel, gravity) in [
        ("near the ground", 0.01, glide, Vector::ZERO),
        ("far from anything", 50.0, glide, Vector::ZERO),
        (
            "thrown onto the ground",
            1.0,
            Vector::new(3.0, -8.0, 0.0),
            gravity,
        ),
    ] {
        let (mut world, body) = plate_lattice_world(20, 10, 10, clearance, linvel, gravity);

        let mut step_times = Vec::with_capacity(300);
        for _ in 0..300 {
            let t = Instant::now();
            world.step();
            step_times.push(t.elapsed());
        }
        step_times.sort();
        let mean = step_times.iter().sum::<Duration>() / step_times.len() as u32;
        let median = step_times[step_times.len() / 2];

        let mut hash: u64 = 0xcbf29ce484222325;
        for b in state_bits(&world) {
            hash = (hash ^ b).wrapping_mul(0x100000001b3);
        }
        println!(
            "{label}: mean step {mean:?}, median step {median:?}, x = {}, state hash {hash:016x}",
            world.bodies[body].translation().x
        );
    }
}
