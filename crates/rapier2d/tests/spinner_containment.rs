//! Regression: a spinner scene (a rotating arm stirring ~6000 small bodies)
//! keeps its small bodies contained inside the oriented-polyline container.
//! Guards the CCD core-circle + exact-TOI clamp and the oriented (one-sided)
//! container wall. Prints a per-stage timing breakdown (run with `--nocapture`).

use rapier2d::prelude::*;
use std::f32::consts::PI;

const POINT_COUNT: usize = 360;
const CENTER_Y: f32 = 32.0;
const RADIUS: f32 = 40.0;

#[test]
fn spinner_keeps_bodies_contained() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();
    let gravity = Vector::new(0.0, -10.0);

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    let q = Rotation::new(-2.0 * PI / POINT_COUNT as f32);
    let mut p = Vector::new(RADIUS, 0.0);
    let mut points = Vec::with_capacity(POINT_COUNT);
    for _ in 0..POINT_COUNT {
        points.push(Vector::new(p.x, p.y + CENTER_Y));
        p = q * p;
    }
    let indices: Vec<[u32; 2]> = (0..POINT_COUNT as u32)
        .map(|i| [i, (i + 1) % POINT_COUNT as u32])
        .collect();
    colliders.insert_with_parent(
        ColliderBuilder::oriented_polyline(points, Some(indices)).friction(0.1),
        ground,
        &mut bodies,
    );

    let spinner = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 12.0))
            .can_sleep(false),
    );
    colliders.insert_with_parent(
        ColliderBuilder::round_cuboid(0.4, 20.0, 0.2).friction(0.0),
        spinner,
        &mut bodies,
    );
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 12.0))
        .local_anchor2(Vector::new(0.0, 0.0))
        .motor_velocity(5.0, 1.0e5)
        .motor_max_force(1.0e9);
    impulse_joints.insert(ground, spinner, joint, true);

    let body_count = 2 * 3038;
    let mut small = Vec::new();
    let mut x = -23.0f32;
    let mut y = 2.0f32;
    for i in 0..body_count {
        let handle = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(x, y)));
        let collider = match i % 3 {
            0 => ColliderBuilder::capsule_from_endpoints(
                Vector::new(-0.25, 0.0),
                Vector::new(0.25, 0.0),
                0.25,
            ),
            1 => ColliderBuilder::ball(0.35),
            _ => ColliderBuilder::cuboid(0.35, 0.35),
        }
        .density(0.25)
        .friction(0.1)
        .restitution(0.1);
        colliders.insert_with_parent(collider, handle, &mut bodies);
        small.push(handle);
        x += 0.5;
        if x >= 23.0 {
            x = -23.0;
            y += 0.5;
        }
    }

    let mut totals = [0.0f64; 6]; // step, collision detection, broad, narrow, solver, ccd
    let start = std::time::Instant::now();
    for _ in 0..3000 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
        totals[1] += pipeline.counters.stages.collision_detection_time.time_ms();
        totals[2] += pipeline.counters.cd.broad_phase_time.time_ms();
        totals[3] += pipeline.counters.cd.narrow_phase_time.time_ms();
        totals[4] += pipeline.counters.stages.solver_time.time_ms();
        totals[5] += pipeline.counters.ccd.toi_computation_time.time_ms();
    }
    totals[0] = start.elapsed().as_secs_f64() * 1000.0;
    println!("spinner (full scale, 3000 steps) avg per step:");
    println!("  total:               {:.3} ms", totals[0] / 3000.0);
    println!(
        "  collision detection: {:.3} ms (broad {:.3} / narrow {:.3})",
        totals[1] / 3000.0,
        totals[2] / 3000.0,
        totals[3] / 3000.0
    );
    println!("  solver:              {:.3} ms", totals[4] / 3000.0);
    println!("  ccd:                 {:.3} ms", totals[5] / 3000.0);

    let center = Vector::new(0.0, CENTER_Y);
    let escaped = small
        .iter()
        .filter(|h| (bodies[**h].translation() - center).length() > RADIUS + 1.0)
        .count();
    assert!(
        escaped <= 2,
        "spinner leaked {escaped}/{body_count} bodies through the container"
    );
}
