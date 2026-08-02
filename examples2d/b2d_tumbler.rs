//! Port of box2d's `tumbler` benchmark (`CreateTumbler`,
//! `box2d/shared/benchmarks.c`). Release: a motor-driven hollow square drum
//! tumbling a 45x45 grid of small boxes.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;
use std::f32::consts::PI;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    let ground = world.insert_body(RigidBodyBuilder::fixed());

    // Drum: four walls forming a hollow box, driven by a revolute motor.
    let drum = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 10.0))
            .can_sleep(false),
    );
    for (hx, hy, off) in [
        (0.5, 10.0, Vector::new(10.0, 0.0)),
        (0.5, 10.0, Vector::new(-10.0, 0.0)),
        (10.0, 0.5, Vector::new(0.0, 10.0)),
        (10.0, 0.5, Vector::new(0.0, -10.0)),
    ] {
        world.insert_collider(
            ColliderBuilder::cuboid(hx, hy)
                .translation(off)
                .density(50.0),
            Some(drum),
        );
    }

    let motor_speed = (PI / 180.0) * 25.0;
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 10.0))
        .local_anchor2(Vector::new(0.0, 0.0))
        .motor_velocity(motor_speed, 1.0e5)
        .motor_max_force(1.0e8);
    world.insert_impulse_joint(ground, drum, joint);

    // Grid of small boxes inside the drum.
    let grid_count = 45i32;
    let mut y = -0.2 * grid_count as f32 + 10.0;
    for _ in 0..grid_count {
        let mut x = -0.2 * grid_count as f32;
        for _ in 0..grid_count {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(x, y)),
                ColliderBuilder::cuboid(0.125, 0.125),
            );
            x += 0.4;
        }
        y += 0.4;
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 10.0), 12.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
