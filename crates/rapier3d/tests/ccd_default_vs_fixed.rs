//! Tests for the default CCD tier: fast dynamic bodies get continuous collision
//! detection against **fixed** colliders automatically, while
//! `ccd_enabled` upgrades a body to also sweep against kinematic/dynamic bodies.

use rapier3d::prelude::*;

/// Minimal world harness so each test reads as scenario + assertions.
struct Harness {
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

impl Harness {
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
            // No gravity: keep the fast bodies on a clean 1D path along +X.
            gravity: Vector::ZERO,
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

    fn run(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step();
        }
    }

    fn x(&self, h: RigidBodyHandle) -> Real {
        self.bodies[h].translation().x
    }
}

/// A thin fixed wall centered at the origin, spanning the Y/Z plane.
fn insert_thin_fixed_wall(h: &mut Harness) {
    let wall = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.05, 5.0, 5.0), wall, &mut h.bodies);
}

/// A small dynamic body far on the -X side, moving fast toward +X. In a single
/// `1/60`s step it moves ~3.3m, far more than the wall thickness — so without CCD
/// it tunnels straight through.
fn insert_fast_dynamic(h: &mut Harness, ccd_enabled: bool) -> RigidBodyHandle {
    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0, 0.0))
            .linvel(Vector::new(200.0, 0.0, 0.0))
            .ccd_enabled(ccd_enabled),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1, 0.1), body, &mut h.bodies);
    body
}

/// A default-tier fast dynamic body must NOT tunnel through a fixed wall,
/// even without `ccd_enabled`.
#[test]
fn default_ccd_vs_fixed_no_tunnel() {
    let mut h = Harness::new();
    insert_thin_fixed_wall(&mut h);
    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(120);

    // The body should have been stopped on the near (-X) side of the wall.
    assert!(
        h.x(body) < 0.0,
        "body tunneled through the fixed wall (x = {})",
        h.x(body)
    );
}

/// Two default-tier fast dynamic bodies on a head-on course DO pass through each
/// other: the default tier only sweeps fixed colliders, so there is no
/// moving-vs-moving CCD. This locks the fixed-only scope.
#[test]
fn default_tier_ignores_dynamic() {
    let mut h = Harness::new();

    let a = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0, 0.0))
            .linvel(Vector::new(200.0, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1, 0.1), a, &mut h.bodies);

    let b = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(3.0, 0.0, 0.0))
            .linvel(Vector::new(-200.0, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1, 0.1), b, &mut h.bodies);

    h.run(5);

    // They swapped sides — passed through each other untouched.
    assert!(
        h.x(a) > 0.0 && h.x(b) < 0.0,
        "default-tier dynamic bodies should tunnel through each other (a.x = {}, b.x = {})",
        h.x(a),
        h.x(b)
    );
}

/// A `ccd_enabled` ("bullet") body still collides with a dynamic body — the
/// upgrade tier is unaffected by the change.
#[test]
fn bullet_still_hits_dynamic() {
    let mut h = Harness::new();

    let bullet = insert_fast_dynamic(&mut h, /* ccd_enabled */ true);

    // A stationary dynamic target at the origin (not ccd_enabled).
    let target = h
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.0, 0.0)));
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        target,
        &mut h.bodies,
    );

    h.run(60);

    // The bullet hit the target and pushed it along +X (it did not tunnel).
    assert!(
        h.x(target) > 0.05,
        "bullet did not hit the dynamic target (target.x = {})",
        h.x(target)
    );
    assert!(
        h.x(bullet) < h.x(target),
        "bullet passed through its target (bullet.x = {}, target.x = {})",
        h.x(bullet),
        h.x(target)
    );
}

/// Setting `max_ccd_substeps = 0` disables CCD for the whole world: the fast
/// default-tier body tunnels through the fixed wall again.
#[test]
fn global_ccd_off_tunnels() {
    let mut h = Harness::new();
    h.params.max_ccd_substeps = 0;
    insert_thin_fixed_wall(&mut h);
    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(60);

    assert!(
        h.x(body) > 1.0,
        "body should tunnel through the wall when CCD is globally disabled (x = {})",
        h.x(body)
    );
}

/// A fast dynamic **trimesh** body is never swept (there is no continuous collision for
/// moving meshes): it does not become `ccd_active`, and it tunnels through a thin fixed wall,
/// relying on speculative contacts only. This locks the never-swept fast-shape rule and
/// the `ccd_thickness` gate fix (a trimesh's zero thickness must not flag the body fast).
#[test]
fn trimesh_fast_body_is_never_swept() {
    use rapier3d::parry::shape::Ball;

    let mut h = Harness::new();
    insert_thin_fixed_wall(&mut h);

    let (vtx, idx) = Ball::new(0.1).to_trimesh(8, 8);
    let shape = SharedShape::new(TriMesh::new(vtx, idx).unwrap());
    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0, 0.0))
            .linvel(Vector::new(200.0, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::new(shape), body, &mut h.bodies);

    h.step();

    assert!(
        !h.bodies[body].is_ccd_active(),
        "a trimesh-only body must never be flagged ccd_active"
    );

    h.run(59);

    assert!(
        h.x(body) > 1.0,
        "a fast trimesh body is not swept and should tunnel through the thin wall (x = {})",
        h.x(body)
    );
}

/// A fast dynamic **compound** body sweeps each convex child: it must not tunnel through
/// a thin fixed wall.
#[test]
fn compound_fast_body_no_tunnel() {
    let mut h = Harness::new();
    insert_thin_fixed_wall(&mut h);

    let shape = SharedShape::compound(vec![
        (
            Pose::from_translation(Vector::new(0.0, 0.15, 0.0)),
            SharedShape::cuboid(0.1, 0.1, 0.1),
        ),
        (
            Pose::from_translation(Vector::new(0.0, -0.15, 0.0)),
            SharedShape::cuboid(0.1, 0.1, 0.1),
        ),
    ]);
    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0, 0.0))
            .linvel(Vector::new(200.0, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::new(shape), body, &mut h.bodies);

    h.run(120);

    assert!(
        h.x(body) < 0.0,
        "compound body tunneled through the fixed wall (x = {})",
        h.x(body)
    );
}

/// A fast dynamic compound body vs a fixed **trimesh** wall exercises the per-child
/// convex-vs-composite sweep: it must not tunnel through the (zero-thickness) mesh.
#[test]
fn compound_fast_body_vs_trimesh_wall_no_tunnel() {
    let mut h = Harness::new();

    // A trimesh wall: a two-triangle quad at x = 0 spanning the Y/Z plane.
    let vtx = vec![
        Vector::new(0.0, -5.0, -5.0),
        Vector::new(0.0, 5.0, -5.0),
        Vector::new(0.0, 5.0, 5.0),
        Vector::new(0.0, -5.0, 5.0),
    ];
    let idx = vec![[0, 1, 2], [0, 2, 3]];
    let wall = SharedShape::new(TriMesh::new(vtx, idx).unwrap());
    h.colliders.insert(ColliderBuilder::new(wall));

    let shape = SharedShape::compound(vec![(Pose::IDENTITY, SharedShape::cuboid(0.1, 0.1, 0.1))]);
    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0, 0.0))
            .linvel(Vector::new(200.0, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::new(shape), body, &mut h.bodies);

    h.run(120);

    assert!(
        h.x(body) < 0.0,
        "compound body tunneled through the trimesh wall (x = {})",
        h.x(body)
    );
}

/// Benchmark reproducing the `dynamic_trimesh3` demo shape mix: many dynamic trimesh
/// bodies falling onto a large fixed trimesh ground. Run with:
/// `cargo test --release -p rapier3d --test ccd_default_vs_fixed -- --ignored --nocapture --test-threads=1`
#[test]
#[ignore = "benchmark"]
fn bench_dynamic_trimeshes_on_trimesh_ground() {
    use rapier3d::parry::shape::Ball;

    let mut h = Harness::new();
    h.gravity = Vector::new(0.0, -9.81, 0.0);

    // ~20k-triangle wavy ground, same construction as the dynamic_trimesh3 demo.
    let nsubdivs = 100;
    let heights = Array2::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        -(i as Real * 40.0 / (nsubdivs as Real) / 2.0).cos()
            - (j as Real * 40.0 / (nsubdivs as Real) / 2.0).cos()
    });
    let heightfield = HeightField::new(heights, Vector::new(100.0, 2.0, 100.0));
    let mut ground = TriMesh::from(heightfield);
    let _ = ground.set_flags(TriMeshFlags::FIX_INTERNAL_EDGES);
    h.colliders
        .insert(ColliderBuilder::new(SharedShape::new(ground)));

    // 60 dynamic trimesh bodies (ball meshes, ~200 triangles each) in a grid.
    let (vtx, idx) = Ball::new(1.5).to_trimesh(10, 10);
    let shape = SharedShape::new(TriMesh::new(vtx, idx).unwrap());
    for k in 0..60 {
        let (i, j) = (k % 8, k / 8);
        let body = h
            .bodies
            .insert(RigidBodyBuilder::dynamic().translation(Vector::new(
                i as Real * 5.0 - 17.5,
                6.0 + (k % 3) as Real * 4.0,
                j as Real * 5.0 - 17.5,
            )));
        h.colliders
            .insert_with_parent(ColliderBuilder::new(shape.clone()), body, &mut h.bodies);
    }

    let t0 = std::time::Instant::now();
    let steps = 300;
    h.run(steps);
    let elapsed = t0.elapsed().as_secs_f64() * 1000.0;
    println!(
        "{steps} steps: {elapsed:.1} ms ({:.3} ms/step)",
        elapsed / steps as f64
    );
}

/// Kinematic bodies are NOT default-tier targets: a default-tier fast body
/// tunnels through a (stationary) kinematic wall. Contrast with
/// `default_ccd_vs_fixed_no_tunnel`, where the same body is stopped by a fixed wall.
#[test]
fn kinematic_not_a_default_target() {
    let mut h = Harness::new();

    let wall = h
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.05, 5.0, 5.0), wall, &mut h.bodies);

    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(60);

    assert!(
        h.x(body) > 1.0,
        "default-tier body should tunnel through a kinematic wall (x = {})",
        h.x(body)
    );
}
