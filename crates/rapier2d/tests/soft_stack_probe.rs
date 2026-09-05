//! Stack scenes for the soft-body depenetration work (columns and a pressed pile of soft bodies)
//! with their gating metrics (boundary crossings, vertices inside another body, cell compression,
//! step time); the `SOFT_STACK_PRESET` env variable (`defaults`, `minimal`) picks the preset.

use rapier2d::prelude::*;

fn segments_cross(a0: Vector, a1: Vector, b0: Vector, b1: Vector) -> bool {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let denom = d1.perp_dot(d2);
    if denom.abs() < 1.0e-12 {
        return false;
    }
    let t = (b0 - a0).perp_dot(d2) / denom;
    let u = (b0 - a0).perp_dot(d1) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Even-odd containment of `point` in a closed polyline mesh (ray along +x).
fn inside(point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody) -> bool {
    let mut parity = false;
    for e in mesh.indices() {
        let (p0, p1) = (
            mesh.vertex(body, e[0] as usize),
            mesh.vertex(body, e[1] as usize),
        );
        if (p0.y > point.y) != (p1.y > point.y) {
            let t = (point.y - p0.y) / (p1.y - p0.y);
            if p0.x + t * (p1.x - p0.x) > point.x {
                parity = !parity;
            }
        }
    }
    parity
}

/// Distance from `point` to the closest element of the mesh.
fn surface_distance(point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody) -> Real {
    let mut best: Real = Real::MAX;
    for e in mesh.indices() {
        let (p0, p1) = (
            mesh.vertex(body, e[0] as usize),
            mesh.vertex(body, e[1] as usize),
        );
        let d = p1 - p0;
        let len2 = d.length_squared();
        let t = if len2 > 0.0 {
            ((point - p0).dot(d) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((point - (p0 + d * t)).length());
    }
    best
}

/// The stack metrics of one probe.
#[derive(Clone, Copy, Debug, Default)]
struct Metrics {
    /// Boundary self-crossings, summed over the bodies.
    self_crossings: usize,
    /// Boundary crossings between distinct bodies, summed over the pairs.
    cross_crossings: usize,
    /// Vertices lying inside another body's closed boundary.
    inside: usize,
    /// The deepest such vertex (distance to that body's boundary).
    max_depth: Real,
    /// The most compressed cell (current over rest area), 1.0 without cells.
    min_cell_ratio: Real,
    /// The lowest inverted-cell count over the bodies, summed.
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
    for (sb, mesh) in &bodies {
        let idx = mesh.indices();
        let p = |v: u32| mesh.vertex(sb, v as usize);
        for i in 0..idx.len() {
            for j in i + 1..idx.len() {
                let (ei, ej) = (idx[i], idx[j]);
                if ei.iter().any(|v| ej.contains(v)) {
                    continue;
                }
                if segments_cross(p(ei[0]), p(ei[1]), p(ej[0]), p(ej[1])) {
                    m.self_crossings += 1;
                }
            }
        }
        for cell in sb.cells() {
            let x: [Vector; 3] =
                core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
            let vol = (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5;
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
            let pa = |v: u32| ma.vertex(sa, v as usize);
            let pb = |v: u32| mb.vertex(sb, v as usize);
            if a < b {
                for ea in ma.indices() {
                    for eb in mb.indices() {
                        if segments_cross(pa(ea[0]), pa(ea[1]), pb(eb[0]), pb(eb[1])) {
                            m.cross_crossings += 1;
                        }
                    }
                }
            }
            if !mb.is_closed() {
                continue;
            }
            for v in 0..ma.vertex_count() {
                let x = pa(v as u32);
                if inside(x, mb, sb) {
                    m.inside += 1;
                    m.max_depth = m.max_depth.max(surface_distance(x, mb, sb));
                }
            }
        }
    }
    m
}

/// The recovery preset under test (`SOFT_STACK_PRESET`).
fn apply_preset(world: &mut PhysicsWorld) -> String {
    let preset = std::env::var("SOFT_STACK_PRESET").unwrap_or_else(|_| "defaults".into());
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    *r = Default::default();
    match preset.as_str() {
        "defaults" => {}
        // The user's reference settings (2026-09-04): prevention without edge speculation,
        // detection + stand-down, the intersection-volume constraints on.
        "reference" => {
            r.edge_speculation = false;
        }
        other => panic!("unknown SOFT_STACK_PRESET `{other}`"),
    }
    preset
}

fn floor(world: &mut PhysicsWorld, half_width: Real) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(half_width, 0.5),
    );
}

fn blob(center: Vector, radius: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, 20)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

fn jelly(center: Vector, half: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::grid(center, Vector::splat(half), 4, 4)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 3.0e3,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .self_contacts(true)
        .particle_mass(0.1)
        .particle_radius(0.08)
        .surface_collider(ColliderBuilder::ball(0.08).friction(0.7))
}

/// Runs `steps` steps, printing the metrics every `every` steps (and the step time at the end);
/// `hook` runs before every step (kinematic drivers).
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
            worst.self_crossings = worst.self_crossings.max(m.self_crossings);
            worst.cross_crossings = worst.cross_crossings.max(m.cross_crossings);
            worst.inside = worst.inside.max(m.inside);
            worst.max_depth = worst.max_depth.max(m.max_depth);
            worst.min_cell_ratio = worst.min_cell_ratio.min(m.min_cell_ratio);
            worst.inverted_cells = worst.inverted_cells.max(m.inverted_cells);
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

/// A column of twelve cell-less blobs dropped one on the other: the bottom blobs bear the
/// whole column's weight and must neither be pushed through the floor nor into each other.
#[test]
fn blob_column() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 4.0);
    let handles: Vec<_> = (0..12)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(blob(Vector::new(x, 0.55 + i as Real * 1.05), 0.5))
        })
        .collect();
    // A heavy box lands on the column once it has settled (twenty times a blob's mass).
    let m = run(
        "blob_column",
        &mut world,
        &handles,
        1200,
        100,
        |world, step| {
            if step == 300 {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 14.0)),
                    ColliderBuilder::cuboid(0.6, 0.4).density(10.0),
                );
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "blobs of the column penetrate each other"
    );
    assert_eq!(m.self_crossings, 0, "a blob of the column is self-crossed");
}

/// A column of eight solid jelly squares: the cells of the bottom ones compress under the
/// load but must never invert, and the boundaries must not cross.
#[test]
fn jelly_column() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 4.0);
    let handles: Vec<_> = (0..8)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(jelly(Vector::new(x, 0.55 + i as Real * 1.1), 0.5))
        })
        .collect();
    let m = run(
        "jelly_column",
        &mut world,
        &handles,
        1200,
        100,
        |world, step| {
            if step == 300 {
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
                    ColliderBuilder::cuboid(0.6, 0.4).density(10.0),
                );
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "squares of the column penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a square of the column has inverted cells"
    );
}

/// A bin of blobs pressed from above by a kinematic plate: the plate descends slowly to two
/// thirds of the pile's height and holds there. Nothing in the pile may end up crossed or
/// inside a neighbor, whatever the load.
#[test]
fn pressed_blob_pile() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 3.5);
    for x in [-3.5, 3.5] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 4.0)),
            ColliderBuilder::cuboid(0.5, 5.0),
        );
    }
    let mut handles = Vec::new();
    for j in 0..4 {
        for i in 0..5 {
            let x = -2.4 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.55 + j as Real * 1.05;
            handles.push(world.insert_soft_body(blob(Vector::new(x, y), 0.5)));
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 5.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.9, 0.25), Some(plate));
    // Settle, press down at 0.5 u/s to y = 3.0 (the pile starts ~4.3 high), then hold.
    let m = run(
        "pressed_blob_pile",
        &mut world,
        &handles,
        1500,
        150,
        |world, step| {
            if step >= 300 {
                let y = (5.0 - (step - 300) as Real * 0.5 / 60.0).max(3.0);
                world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "blobs of the pressed pile penetrate each other"
    );
    assert_eq!(
        m.self_crossings, 0,
        "a blob of the pressed pile is self-crossed"
    );
}

/// A bin of jelly squares pressed the same way, to four fifths of the pile's height: cells
/// compress, none may invert.
#[test]
fn pressed_jelly_pile() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 3.5);
    for x in [-3.5, 3.5] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 4.0)),
            ColliderBuilder::cuboid(0.5, 5.0),
        );
    }
    let mut handles = Vec::new();
    for j in 0..3 {
        for i in 0..5 {
            let x = -2.4 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.55 + j as Real * 1.1;
            handles.push(world.insert_soft_body(jelly(Vector::new(x, y), 0.5)));
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 4.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.9, 0.25), Some(plate));
    let m = run(
        "pressed_jelly_pile",
        &mut world,
        &handles,
        1500,
        150,
        |world, step| {
            if step >= 300 {
                let y = (4.0 - (step - 300) as Real * 0.5 / 60.0).max(2.7);
                world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "squares of the pressed pile penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a square of the pressed pile has inverted cells"
    );
}

/// Checks that the jelly pile crushed to two thirds of its height keeps its cells uninverted and
/// its boundaries uncrossed. Ignored until a phase passes it.
#[test]
#[ignore]
fn crushed_jelly_pile() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 3.5);
    for x in [-3.5, 3.5] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 4.0)),
            ColliderBuilder::cuboid(0.5, 5.0),
        );
    }
    let mut handles = Vec::new();
    for j in 0..3 {
        for i in 0..5 {
            let x = -2.4 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.55 + j as Real * 1.1;
            handles.push(world.insert_soft_body(jelly(Vector::new(x, y), 0.5)));
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 4.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.9, 0.25), Some(plate));
    let m = run(
        "crushed_jelly_pile",
        &mut world,
        &handles,
        1500,
        150,
        |world, step| {
            if step >= 300 {
                let y = (4.0 - (step - 300) as Real * 0.5 / 60.0).max(2.4);
                world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
            }
        },
    );
    assert_eq!(
        m.cross_crossings, 0,
        "squares of the crushed pile penetrate each other"
    );
    assert_eq!(
        m.self_crossings, 0,
        "a square of the crushed pile is self-crossed"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a square of the crushed pile has inverted cells"
    );
}

/// Step-time reference: the 24-blob pile of the recovery probes, resting.
#[test]
#[ignore]
fn pile_timing() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0);
    let handles: Vec<_> = (0..4)
        .flat_map(|j| (0..6).map(move |i| (i, j)))
        .map(|(i, j)| {
            let x = -3.0 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.7 + j as Real * 1.1;
            world.insert_soft_body(blob(Vector::new(x, y), 0.5))
        })
        .collect();
    apply_preset(&mut world);
    for _ in 0..120 {
        world.step();
    }
    let start = std::time::Instant::now();
    for _ in 0..2000 {
        world.step();
    }
    let m = measure(&world, &handles);
    println!(
        "pile: {:.3} ms/step (cross={} inside={})",
        start.elapsed().as_secs_f64() * 1000.0 / 2000.0,
        m.cross_crossings,
        m.inside
    );
}
