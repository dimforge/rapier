//! Regression test for https://github.com/dimforge/rapier/issues/868
//!
//! Running the same 8-ball drop scene twice from identical initial conditions on
//! the same machine used to produce different results (local determinism was
//! violated without the `enhanced-determinism` feature).

use rapier3d::prelude::*;

fn run_scene() -> Vec<[u32; 3]> {
    let mut world = PhysicsWorld::new();

    world
        .colliders
        .insert(ColliderBuilder::cuboid(100.0, 0.1, 100.0));

    let mut handles = Vec::new();
    for x in 0..2 {
        for y in 0..2 {
            for z in 0..2 {
                let body = RigidBodyBuilder::dynamic().translation(Vector::new(
                    x as f32 * 0.51 + 3.0,
                    y as f32 * 0.5 + 3.5,
                    z as f32 * 0.5 + 3.0,
                ));
                let handle = world.bodies.insert(body);
                world.colliders.insert_with_parent(
                    ColliderBuilder::ball(0.5).restitution(0.7),
                    handle,
                    &mut world.bodies,
                );
                handles.push(handle);
            }
        }
    }

    for _ in 0..200 {
        world.step();
    }

    handles
        .iter()
        .map(|h| {
            let t = world.bodies[*h].translation();
            // Compare raw bits: the runs must be bitwise identical.
            [t.x.to_bits(), t.y.to_bits(), t.z.to_bits()]
        })
        .collect()
}

#[test]
fn same_machine_runs_are_bitwise_identical() {
    let first = run_scene();
    let second = run_scene();
    assert_eq!(first, second, "two identical runs diverged");
}
