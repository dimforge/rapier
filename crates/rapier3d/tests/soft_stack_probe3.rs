//! 3D stack scenes for the soft-body depenetration work (metrics in the 2D `soft_stack_probe`):
//! balloon and jelly-cube columns under a heavy weight, and a cube pile pressed by a kinematic
//! plate. `SOFT_STACK_PRESET` selects the recovery preset (`defaults`, `minimal`).

use rapier3d::parry::query::PointQuery;
use rapier3d::prelude::*;

/// Whether the segment `[p, q]` crosses the triangle's interior transversally.
fn segment_crosses_triangle(p: Vector, q: Vector, tri: &[Vector; 3]) -> bool {
    let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    let dp = (p - tri[0]).dot(n);
    let dq = (q - tri[0]).dot(n);
    if dp * dq >= 0.0 {
        return false;
    }
    let pq = q - p;
    let d = [
        pq.dot((tri[0] - p).cross(tri[1] - p)),
        pq.dot((tri[1] - p).cross(tri[2] - p)),
        pq.dot((tri[2] - p).cross(tri[0] - p)),
    ];
    d.iter().all(|x| *x >= 0.0) || d.iter().all(|x| *x <= 0.0)
}

/// Whether two triangles cross (an edge of one pierces the other).
fn triangles_cross(a: &[Vector; 3], b: &[Vector; 3]) -> bool {
    (0..3).any(|k| segment_crosses_triangle(a[k], a[(k + 1) % 3], b))
        || (0..3).any(|k| segment_crosses_triangle(b[k], b[(k + 1) % 3], a))
}

/// Ray-parity containment of `point` in a closed triangle mesh.
fn inside(point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody) -> bool {
    let q = point + Vector::new(0.5341, 0.6432, 0.5487) * 100.0;
    let mut crossings = 0;
    for e in mesh.indices() {
        let tri: [Vector; 3] = core::array::from_fn(|k| mesh.vertex(body, e[k] as usize));
        if segment_crosses_triangle(point, q, &tri) {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

/// Distance from `point` to the closest triangle of the mesh.
fn surface_distance(point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody) -> Real {
    let mut best: Real = Real::MAX;
    for e in mesh.indices() {
        let tri = rapier3d::parry::shape::Triangle::new(
            mesh.vertex(body, e[0] as usize),
            mesh.vertex(body, e[1] as usize),
            mesh.vertex(body, e[2] as usize),
        );
        best = best.min(tri.distance_to_local_point(point, true));
    }
    best
}

#[derive(Clone, Copy, Debug, Default)]
struct Metrics {
    self_crossings: usize,
    cross_crossings: usize,
    inside: usize,
    max_depth: Real,
    min_cell_ratio: Real,
    inverted_cells: usize,
}

fn measure(world: &PhysicsWorld, handles: &[SoftBodyHandle]) -> Metrics {
    let mut m = Metrics {
        min_cell_ratio: 1.0,
        ..Default::default()
    };
    let bodies: Vec<(&SoftBody, &SoftCollisionMesh)> = handles
        .iter()
        .map(|&h| {
            let sb = &world.soft_bodies[h];
            (sb, sb.meshes().next().unwrap())
        })
        .collect();
    let tris = |sb: &SoftBody, mesh: &SoftCollisionMesh| -> Vec<[Vector; 3]> {
        mesh.indices()
            .iter()
            .map(|e| core::array::from_fn(|k| mesh.vertex(sb, e[k] as usize)))
            .collect()
    };
    let all: Vec<Vec<[Vector; 3]>> = bodies.iter().map(|(sb, mesh)| tris(sb, mesh)).collect();
    for (bi, (sb, mesh)) in bodies.iter().enumerate() {
        let idx = mesh.indices();
        let t = &all[bi];
        for i in 0..idx.len() {
            for j in i + 1..idx.len() {
                if idx[i].iter().any(|v| idx[j].contains(v)) {
                    continue;
                }
                if triangles_cross(&t[i], &t[j]) {
                    m.self_crossings += 1;
                }
            }
        }
        for cell in sb.cells() {
            let x: [Vector; 4] =
                core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
            let vol = (x[1] - x[0]).cross(x[2] - x[0]).dot(x[3] - x[0]) / 6.0;
            let ratio = vol / cell.rest_volume;
            m.min_cell_ratio = m.min_cell_ratio.min(ratio);
            if ratio <= 0.0 {
                m.inverted_cells += 1;
            }
        }
    }
    for a in 0..bodies.len() {
        for b in 0..bodies.len() {
            if a == b {
                continue;
            }
            let (sa, ma) = bodies[a];
            let (sb, mb) = bodies[b];
            if a < b {
                for ta in &all[a] {
                    for tb in &all[b] {
                        if triangles_cross(ta, tb) {
                            m.cross_crossings += 1;
                        }
                    }
                }
            }
            if !mb.is_closed() {
                continue;
            }
            for v in 0..ma.vertex_count() {
                let x = ma.vertex(sa, v);
                if inside(x, mb, sb) {
                    m.inside += 1;
                    m.max_depth = m.max_depth.max(surface_distance(x, mb, sb));
                }
            }
        }
    }
    m
}

fn apply_preset(world: &mut PhysicsWorld) -> String {
    let preset = std::env::var("SOFT_STACK_PRESET").unwrap_or_else(|_| "defaults".into());
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    *r = Default::default();
    // Per-feature policy of the volume-patch constraints (`SoftPatchConstraints`), on any preset:
    // `SOFT_PATCH_CONSTRAINTS=stand|normal`.
    match std::env::var("SOFT_PATCH_CONSTRAINTS").as_deref() {
        Ok("stand") => r.overlap_patch_constraints = SoftPatchConstraints::StandDown,
        Ok("normal") => r.overlap_patch_constraints = SoftPatchConstraints::AlongNormal,
        _ => {}
    }
    match preset.as_str() {
        "defaults" => {}
        // The user's reference settings (2026-09-04): prevention without edge speculation,
        // detection + stand-down, the intersection-volume constraints on.
        "reference" => {
            r.edge_speculation = false;
            r.overlap_constraints = true;
        }
        // The reference settings with the crossing repulsion, unguided or guided by the
        // pair's (and the fold's) volume normal (see `crossing_repulsion_guide`).
        "reference+repel" | "reference+repel-guide" => {
            r.edge_speculation = false;
            r.overlap_constraints = true;
            r.crossing_repulsion = true;
            r.crossing_repulsion_guide = preset.ends_with("-guide");
        }
        // The intersection-volume constraints on the defaults.
        "overlap" => r.overlap_constraints = true,
        other => panic!("unknown SOFT_STACK_PRESET `{other}`"),
    }
    preset
}

fn floor(world: &mut PhysicsWorld, half: Real) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half, 0.5, half),
    );
}

fn balloon(center: Vector, radius: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::sphere(center, radius, 1)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

fn jelly(center: Vector, half: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::cuboid(center, Vector::splat(half), 3, 3, 3)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 3.0e3,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .self_contacts(true)
        .particle_mass(0.1)
        .surface_collider(ColliderBuilder::ball(0.08).friction(0.7))
}

fn run(
    name: &str,
    world: &mut PhysicsWorld,
    handles: &[SoftBodyHandle],
    steps: usize,
    every: usize,
    mut hook: impl FnMut(&mut PhysicsWorld, usize),
) -> Metrics {
    let preset = apply_preset(world);
    let mut worst = Metrics::default();
    let mut stepping = std::time::Duration::ZERO;
    for step in 0..=steps {
        if step % every == 0 {
            let m = measure(world, handles);
            println!(
                "{name} [{preset}] step {step:4}: self={} cross={} inside={} depth={:.3} \
                 min_cell={:+.3} inverted={}",
                m.self_crossings,
                m.cross_crossings,
                m.inside,
                m.max_depth,
                m.min_cell_ratio,
                m.inverted_cells
            );
            worst.cross_crossings = worst.cross_crossings.max(m.cross_crossings);
            worst.inside = worst.inside.max(m.inside);
            worst.max_depth = worst.max_depth.max(m.max_depth);
            worst.min_cell_ratio = worst.min_cell_ratio.min(m.min_cell_ratio);
        }
        if step == steps {
            break;
        }
        hook(world, step);
        let start = std::time::Instant::now();
        world.step();
        stepping += start.elapsed();
    }
    let ms = stepping.as_secs_f64() * 1000.0 / steps as f64;
    let last = measure(world, handles);
    println!(
        "{name} [{preset}] final: self={} cross={} inside={} depth={:.3} min_cell={:+.3} \
         inverted={} ({ms:.3} ms/step); worst: cross={} inside={} depth={:.3} min_cell={:+.3}",
        last.self_crossings,
        last.cross_crossings,
        last.inside,
        last.max_depth,
        last.min_cell_ratio,
        last.inverted_cells,
        worst.cross_crossings,
        worst.inside,
        worst.max_depth,
        worst.min_cell_ratio
    );
    last
}

/// A column of six balloons with a heavy box landing on top once it has settled.
#[test]
fn balloon_column() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 4.0);
    let handles: Vec<_> = (0..6)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(balloon(Vector::new(x, 0.55 + i as Real * 1.05, 0.0), 0.5))
        })
        .collect();
    let m = run(
        "balloon_column",
        &mut world,
        &handles,
        900,
        100,
        |world, step| {
            if step == 300 {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 8.0, 0.0)),
                    ColliderBuilder::cuboid(0.5, 0.3, 0.5).density(10.0),
                );
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "balloons of the column penetrate each other"
    );
    assert_eq!(
        m.self_crossings, 0,
        "a balloon of the column is self-crossed"
    );
}

/// A column of five jelly cubes under the same weight: cells compress, none may invert.
#[test]
fn jelly_column() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 4.0);
    let handles: Vec<_> = (0..5)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(jelly(Vector::new(x, 0.55 + i as Real * 1.1, 0.0), 0.5))
        })
        .collect();
    let m = run(
        "jelly_column",
        &mut world,
        &handles,
        900,
        100,
        |world, step| {
            if step == 300 {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 7.0, 0.0)),
                    ColliderBuilder::cuboid(0.5, 0.3, 0.5).density(10.0),
                );
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "cubes of the column penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a cube of the column has inverted cells"
    );
}

/// Two layers of jelly cubes in a bin, pressed by a kinematic plate to four fifths of
/// their height and held there.
#[test]
fn pressed_jelly_pile() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 3.0);
    for (x, z) in [(-2.6, 0.0), (2.6, 0.0), (0.0, -2.6), (0.0, 2.6)] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 3.0, z)),
            ColliderBuilder::cuboid(
                if z == 0.0 { 0.3 } else { 2.9 },
                3.5,
                if z == 0.0 { 2.9 } else { 0.3 },
            ),
        );
    }
    let mut handles = Vec::new();
    for j in 0..2 {
        for i in 0..2 {
            for k in 0..2 {
                let x = -0.6 + i as Real * 1.2 + (j % 2) as Real * 0.3;
                let z = -0.6 + k as Real * 1.2 + (j % 2) as Real * 0.3;
                let y = 0.55 + j as Real * 1.1;
                handles.push(world.insert_soft_body(jelly(Vector::new(x, y, z), 0.5)));
            }
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 3.0, 0.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.2, 0.25, 2.2), Some(plate));
    let m = run(
        "pressed_jelly_pile",
        &mut world,
        &handles,
        1200,
        150,
        |world, step| {
            if step >= 300 {
                let y = (3.0 - (step - 300) as Real * 0.5 / 60.0).max(1.95);
                world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y, 0.0));
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "cubes of the pressed pile penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a cube of the pressed pile has inverted cells"
    );
}

/// The 3D trigger scene: the same pile crushed to two thirds of its height. Ignored until a
/// phase passes it; see the 2D `crushed_jelly_pile`.
#[test]
#[ignore]
fn crushed_jelly_pile() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 3.0);
    for (x, z) in [(-2.6, 0.0), (2.6, 0.0), (0.0, -2.6), (0.0, 2.6)] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 3.0, z)),
            ColliderBuilder::cuboid(
                if z == 0.0 { 0.3 } else { 2.9 },
                3.5,
                if z == 0.0 { 2.9 } else { 0.3 },
            ),
        );
    }
    let mut handles = Vec::new();
    for j in 0..2 {
        for i in 0..2 {
            for k in 0..2 {
                let x = -0.6 + i as Real * 1.2 + (j % 2) as Real * 0.3;
                let z = -0.6 + k as Real * 1.2 + (j % 2) as Real * 0.3;
                let y = 0.55 + j as Real * 1.1;
                handles.push(world.insert_soft_body(jelly(Vector::new(x, y, z), 0.5)));
            }
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 3.0, 0.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.2, 0.25, 2.2), Some(plate));
    let m = run(
        "crushed_jelly_pile",
        &mut world,
        &handles,
        1200,
        150,
        |world, step| {
            if step >= 300 {
                let y = (3.0 - (step - 300) as Real * 0.5 / 60.0).max(1.6);
                world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y, 0.0));
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "cubes of the crushed pile penetrate each other"
    );
    assert_eq!(
        m.self_crossings, 0,
        "a cube of the crushed pile is self-crossed"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a cube of the crushed pile has inverted cells"
    );
}
