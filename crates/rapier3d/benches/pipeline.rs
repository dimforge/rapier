//! Criterion benchmarks for steady-state `PhysicsPipeline::step` on scenes
//! chosen to exercise the hot paths from the 2026-08 optimization audit:
//!
//! - `box_pyramid`: many persistent convex contacts — contact-constraint
//!   update (R1), narrow/broad-phase per-step scratch allocations (R4).
//! - `terrain_debris`: convex shapes on a trimesh — contact clustering
//!   warm-start carry (R5) and the trimesh manifold path in parry.
//!
//! Every scene runs a settle phase outside the timed region and all dynamic
//! bodies have sleeping disabled so each timed step does full solver work.

use criterion::{criterion_group, criterion_main, Criterion};
use rapier3d::prelude::*;
use std::hint::black_box;
use std::time::Duration;

struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    pipeline: PhysicsPipeline,
    bf: BroadPhaseBvh,
    nf: NarrowPhase,
    islands: IslandManager,
    ccd: CCDSolver,
    params: IntegrationParameters,
    gravity: Vector,
}

impl World {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            pipeline: PhysicsPipeline::new(),
            bf: BroadPhaseBvh::new(),
            nf: NarrowPhase::new(),
            islands: IslandManager::new(),
            ccd: CCDSolver::new(),
            params: IntegrationParameters::default(),
            gravity: Vector::new(0.0, -9.81, 0.0),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            self.gravity,
            &self.params,
            &mut self.islands,
            &mut self.bf,
            &mut self.nf,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }

    fn settle(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step();
        }
    }
}

fn add_ground(w: &mut World) {
    let ground = w.bodies.insert(RigidBodyBuilder::fixed());
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
        ground,
        &mut w.bodies,
    );
}

/// A pyramid of boxes: `base * (base + 1) / 2` dynamic bodies in persistent
/// resting contact.
fn box_pyramid(base: usize) -> World {
    let mut w = World::new();
    add_ground(&mut w);

    let half = 0.5;
    let spacing = half * 2.0 * 1.001;
    for layer in 0..base {
        let count = base - layer;
        let y = half + layer as f32 * spacing;
        let x0 = -(count as f32) * half + layer as f32 * half;
        for i in 0..count {
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x0 + i as f32 * spacing, y, 0.0))
                    .can_sleep(false),
            );
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(half, half, half),
                body,
                &mut w.bodies,
            );
        }
    }
    w
}

fn terrain_mesh(subdivisions: usize) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let n = subdivisions;
    let mut vertices = Vec::with_capacity((n + 1) * (n + 1));
    let mut indices = Vec::with_capacity(n * n * 2);
    for iz in 0..=n {
        for ix in 0..=n {
            let x = ix as f32 / n as f32 * 40.0 - 20.0;
            let z = iz as f32 / n as f32 * 40.0 - 20.0;
            let y = (x * 0.5).sin() * 0.6 + (z * 0.4).cos() * 0.6;
            vertices.push(Vector::new(x, y, z));
        }
    }
    let stride = n + 1;
    for iz in 0..n {
        for ix in 0..n {
            let a = (iz * stride + ix) as u32;
            let b = (iz * stride + ix + 1) as u32;
            let c = ((iz + 1) * stride + ix) as u32;
            let d = ((iz + 1) * stride + ix + 1) as u32;
            indices.push([a, b, c]);
            indices.push([b, d, c]);
        }
    }
    (vertices, indices)
}

/// Mixed convex debris resting on a trimesh terrain: exercises the trimesh
/// manifold path and contact clustering.
fn terrain_debris(count: usize) -> World {
    let mut w = World::new();

    let terrain_body = w.bodies.insert(RigidBodyBuilder::fixed());
    let (vertices, indices) = terrain_mesh(64);
    w.colliders.insert_with_parent(
        ColliderBuilder::trimesh(vertices, indices).unwrap(),
        terrain_body,
        &mut w.bodies,
    );

    // Deterministic pseudo-random placement (no RNG dependency).
    let mut seed = 0x9e3779b9u32;
    let mut next = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed >> 8) as f32 / (1u32 << 24) as f32
    };

    for i in 0..count {
        let x = next() * 30.0 - 15.0;
        let z = next() * 30.0 - 15.0;
        let y = 2.0 + next() * 3.0;
        let body = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y, z))
                .can_sleep(false),
        );
        let collider = match i % 3 {
            0 => ColliderBuilder::ball(0.4),
            1 => ColliderBuilder::cuboid(0.35, 0.35, 0.35),
            _ => ColliderBuilder::capsule_y(0.3, 0.25),
        };
        w.colliders.insert_with_parent(collider, body, &mut w.bodies);
    }
    w
}

fn bench_pipeline(c: &mut Criterion) {
    let mut g = c.benchmark_group("pipeline_step");
    g.sample_size(30);

    g.bench_function("box_pyramid_20", |b| {
        let mut w = box_pyramid(20); // 210 boxes
        w.settle(150);
        b.iter(|| {
            w.step();
            black_box(w.bodies.len())
        })
    });

    g.bench_function("terrain_debris_300", |b| {
        let mut w = terrain_debris(300);
        w.settle(200);
        b.iter(|| {
            w.step();
            black_box(w.bodies.len())
        })
    });

    g.finish();
}

fn config() -> Criterion {
    Criterion::default()
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(5))
}

criterion_group! {
    name = benches;
    config = config();
    targets = bench_pipeline
}
criterion_main!(benches);
