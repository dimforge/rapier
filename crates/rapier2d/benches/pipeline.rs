// Criterion benchmarks for steady-state `PhysicsPipeline::step` in 2D,
// mirroring the rapier3d suite: a box pyramid (persistent convex contacts)
// and mixed debris on a polyline terrain. Written against `Real` so the
// rapier2d-f64 crate reuses the body verbatim.

use criterion::{criterion_group, criterion_main, Criterion};
use rapier2d::prelude::*;
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
            gravity: Vector::new(0.0, -9.81),
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
        ColliderBuilder::cuboid(50.0, 0.5).translation(Vector::new(0.0, -0.5)),
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
        let y = half + layer as Real * spacing;
        let x0 = -(count as Real) * half + layer as Real * half;
        for i in 0..count {
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x0 + i as Real * spacing, y))
                    .can_sleep(false),
            );
            w.colliders
                .insert_with_parent(ColliderBuilder::cuboid(half, half), body, &mut w.bodies);
        }
    }
    w
}

fn terrain_vertices(segments: usize) -> Vec<Vector> {
    (0..=segments)
        .map(|i| {
            let x = i as Real / segments as Real * 60.0 - 30.0;
            Vector::new(x, (x * 0.5).sin() * 0.6)
        })
        .collect()
}

/// Mixed convex debris resting on a polyline terrain.
fn terrain_debris(count: usize) -> World {
    let mut w = World::new();

    let terrain_body = w.bodies.insert(RigidBodyBuilder::fixed());
    w.colliders.insert_with_parent(
        ColliderBuilder::polyline(terrain_vertices(256), None),
        terrain_body,
        &mut w.bodies,
    );

    // Deterministic pseudo-random placement (no RNG dependency).
    let mut seed = 0x9e3779b9u32;
    let mut next = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed >> 8) as Real / (1u32 << 24) as Real
    };

    for i in 0..count {
        let x = next() * 40.0 - 20.0;
        let y = 2.0 + next() * 6.0;
        let body = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y))
                .can_sleep(false),
        );
        let collider = match i % 3 {
            0 => ColliderBuilder::ball(0.4),
            1 => ColliderBuilder::cuboid(0.35, 0.35),
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
