//! Port of box2d's `spinner` benchmark (`CreateSpinner`,
//! `box2d/shared/benchmarks.c`). Release: a motor-driven rounded bar stirring
//! ~6000 mixed small bodies inside a circular chain container.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;
use std::f32::consts::PI;

const POINT_COUNT: usize = 360;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    // Circular chain container (radius 40, centered at (0, 32)).
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    let q = Rotation::new(-2.0 * PI / POINT_COUNT as f32);
    let mut p = Vector::new(40.0, 0.0);
    let mut points = Vec::with_capacity(POINT_COUNT);
    for _ in 0..POINT_COUNT {
        points.push(Vector::new(p.x, p.y + 32.0));
        p = q * p;
    }
    let indices: Vec<[u32; 2]> = (0..POINT_COUNT as u32)
        .map(|i| [i, (i + 1) % POINT_COUNT as u32])
        .collect();
    // Oriented (one-sided) container wall, like box2d's chain shape: only collides on
    // its interior side, so crushed/piled bodies can't squeeze through the thin wall.
    world.insert_collider(
        ColliderBuilder::oriented_polyline(points, Some(indices)).friction(0.1),
        Some(ground),
    );

    // The spinner bar + its motor.
    let spinner = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 12.0))
            .can_sleep(false),
    );
    world.insert_collider(
        ColliderBuilder::round_cuboid(0.4, 20.0, 0.2).friction(0.0),
        Some(spinner),
    );
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 12.0))
        .local_anchor2(Vector::new(0.0, 0.0))
        .motor_velocity(5.0, 1.0e5)
        .motor_max_force(1.0e9);
    world.insert_impulse_joint(ground, spinner, joint);

    // ~6000 mixed small bodies.
    let body_count = 2 * 3038;
    let capsule = || {
        ColliderBuilder::capsule_from_endpoints(
            Vector::new(-0.25, 0.0),
            Vector::new(0.25, 0.0),
            0.25,
        )
    };
    let mut x = -23.0f32;
    let mut y = 2.0f32;
    for i in 0..body_count {
        let body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
        let handle = world.insert_body(body);
        let collider = match i % 3 {
            0 => capsule(),
            1 => ColliderBuilder::ball(0.35),
            _ => ColliderBuilder::cuboid(0.35, 0.35),
        }
        .density(0.25)
        .friction(0.1)
        .restitution(0.1);
        world.insert_collider(collider, Some(handle));

        x += 0.5;
        if x >= 23.0 {
            x = -23.0;
            y += 0.5;
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 32.0), 6.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
