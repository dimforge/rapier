//! Port of box3d's `junkyard` benchmark (`CreateJunkyard` + `StepJunkyard`,
//! `box3d/shared/benchmarks.c`). Release settings: a walled arena filled with a
//! 24x21x21 stack of convex "rocks", stirred by an orbiting kinematic cylinder.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;
use std::f32::consts::PI;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    // Ground + walls: one fixed body at y = -1 with 5 box colliders.
    let ground =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)));
    world.insert_collider(ColliderBuilder::cuboid(120.0, 1.0, 120.0), Some(ground));
    for (hx, hy, hz, off) in [
        (1.0, 8.0, 50.0, Vector::new(-50.0, 8.0, 0.0)),
        (1.0, 8.0, 50.0, Vector::new(50.0, 8.0, 0.0)),
        (50.0, 8.0, 1.0, Vector::new(0.0, 8.0, -50.0)),
        (50.0, 8.0, 1.0, Vector::new(0.0, 8.0, 50.0)),
    ] {
        world.insert_collider(
            ColliderBuilder::cuboid(hx, hy, hz).translation(off),
            Some(ground),
        );
    }

    // Rocks: 24 layers of a 21x21 grid. box3d shares a single hull across all
    // rocks; do the same so the hull is computed once and the renderer can
    // instance the ~10.5k identical bodies into one draw call.
    let rock = SharedShape::convex_hull(&create_rock(1.5)).unwrap();
    let count = 24i32;
    let height = 24.0f32;
    for y in 0..count {
        for x in 0..=20 {
            for z in 0..=20 {
                let pos = Vector::new(
                    -40.0 + 4.0 * x as f32,
                    4.0 * y as f32 + height + 1.0,
                    -40.0 + 4.0 * z as f32,
                );
                world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::new(rock.clone()),
                );
            }
        }
    }

    // Orbiting kinematic pusher.
    let radius = 35.0f32;
    let pusher_hull = create_cylinder(24.0, 4.0, 0.0, 16);
    let pusher = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(radius, 0.0, 0.0)),
    );
    world.insert_collider(
        ColliderBuilder::convex_hull(&pusher_hull).unwrap(),
        Some(pusher),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 90.0, 125.0), Vec3::new(0.0, 0.0, 0.0));

    let mut degrees = 0.0f32;
    let time_step = 1.0 / 60.0;
    let omega = -6.0f32;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            degrees += omega * time_step;
            let rad = degrees * PI / 180.0;
            let target = Vector::new(radius * rad.cos(), 0.0, radius * rad.sin());
            world.bodies[pusher].set_next_kinematic_translation(target);
            world.step();
        }
    }
    Ok(())
}

/// box3d `b3CreateCylinder` (`src/hull.c`): `2 * sides` points forming a
/// cylinder of the given `height`/`radius`, its base at `y_offset`, aligned
/// with the Y axis. Returned as a convex point cloud (rapier builds the hull).
fn create_cylinder(height: f32, radius: f32, y_offset: f32, sides: usize) -> Vec<Vector> {
    let mut points = Vec::with_capacity(2 * sides);
    let delta_alpha = 2.0 * PI / sides as f32;
    let mut alpha = 0.0f32;
    for _ in 0..sides {
        let (sin_a, cos_a) = alpha.sin_cos();
        points.push(Vector::new(radius * cos_a, y_offset, radius * sin_a));
        points.push(Vector::new(
            radius * cos_a,
            y_offset + height,
            radius * sin_a,
        ));
        alpha += delta_alpha;
    }
    points
}

/// box3d `b3CreateRock` (`src/hull.c`): 10 points on a Fibonacci lattice on a
/// sphere of the given `radius`.
fn create_rock(radius: f32) -> Vec<Vector> {
    let point_count = 10usize;
    let phi = (1.0 + 5.0f32.sqrt()) / 2.0;
    let theta = 2.0 * PI / phi;
    let (delta_sin, delta_cos) = theta.sin_cos();
    let (mut cos, mut sin) = (1.0f32, 0.0f32);
    let mut points = Vec::with_capacity(point_count);
    for i in 0..point_count {
        let z = 1.0 - (2.0 * i as f32 + 1.0) / point_count as f32;
        let radius_xy = (1.0 - z * z).sqrt();
        points.push(Vector::new(
            radius * radius_xy * cos,
            radius * radius_xy * sin,
            radius * z,
        ));
        let (c0, s0) = (cos, sin);
        cos = delta_cos * c0 - delta_sin * s0;
        sin = delta_sin * c0 + delta_cos * s0;
    }
    points
}
