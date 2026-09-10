//! The intersection-volume constraints as contacts (`SoftRecoverySettings::overlap_constraints`)
//! under the reference settings (`SOFT_VOLUME_MONO`: no multi-volume grid, `SOFT_NORMAL_PUSH`:
//! push along the normal): patch policies, self-guided repulsion, self-region overlaps.

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

fn overlap(world: &PhysicsWorld, a: SoftBodyHandle, b: SoftBodyHandle) -> (usize, usize, usize) {
    let (sa, sb) = (&world.soft_bodies[a], &world.soft_bodies[b]);
    let (ma, mb) = (sa.meshes().next().unwrap(), sb.meshes().next().unwrap());
    let pa = |v: u32| ma.vertex(sa, v as usize);
    let pb = |v: u32| mb.vertex(sb, v as usize);
    let mut crossings = 0;
    for ea in ma.indices() {
        for eb in mb.indices() {
            if segments_cross(pa(ea[0]), pa(ea[1]), pb(eb[0]), pb(eb[1])) {
                crossings += 1;
            }
        }
    }
    let a_in_b = (0..ma.vertex_count())
        .filter(|&v| inside(pa(v as u32), mb, sb))
        .count();
    let b_in_a = (0..mb.vertex_count())
        .filter(|&v| inside(pb(v as u32), ma, sa))
        .count();
    (crossings, a_in_b, b_in_a)
}

fn max_speed(world: &PhysicsWorld, h: SoftBodyHandle) -> Real {
    world.soft_bodies[h]
        .particle_velocities()
        .map(|v| v.length())
        .fold(0.0, Real::max)
}

/// The reference settings with the crossing repulsion guides and the volume constraints on (see
/// the module docs).
fn configure(world: &mut PhysicsWorld) {
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    r.overlap_constraints = true;
    r.edge_speculation = false;
    // The mono-volume variant (a single multiplier per pair: blocks tip on slopes).
    if std::env::var("SOFT_VOLUME_MONO").is_ok() {
        r.overlap_multi_volume = false;
    }
    if std::env::var("SOFT_NORMAL_PUSH").is_ok() {
        r.overlap_normal_push = true;
    }
}

fn floor(world: &mut PhysicsWorld) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(6.0, 0.5).friction(0.6),
    );
}

/// Steps to run (`SOFT_STEPS` shortens a scene for tracing).
fn steps() -> usize {
    std::env::var("SOFT_STEPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(900)
}

/// The stack suite's blob: a cell-less loop stiff enough to support another.
fn blob(center: Vector, radius: Real, n: usize) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, n)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

/// The stack suite's jelly square: a solid corotational grid (its flat faces neither roll
/// nor tip, unlike a blob).
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

/// A blob whose top vertex was pushed through its own bottom (two crossings and a mirrored tip),
/// with the crossing repulsion and no volume constraint: guided by the fold's volume normal, the
/// tip is pulled back onto the pierced surface until the crossings annihilate.
#[test]
fn self_guided_repulsion_recovers_pierced_tip() {
    let run = |guided: bool| -> usize {
        let mut world = PhysicsWorld::new();
        floor(&mut world);
        let a = world.insert_soft_body(blob(Vector::new(0.0, 0.56), 0.5, 24));
        // Vertex 6 is the top of the loop (angle 90 degrees): pushed below the bottom.
        world.soft_bodies[a].set_particle_position(6, Vector::new(0.0, 0.56 - 0.5 - 0.15));
        configure(&mut world);
        let r = &mut world.integration_parameters.soft_bodies.recovery;
        r.overlap_constraints = false;
        r.crossing_repulsion = true;
        r.crossing_repulsion_self_guide = guided;
        let self_crossings = |world: &PhysicsWorld| {
            let sb = &world.soft_bodies[a];
            let mesh = sb.meshes().next().unwrap();
            let idx = mesh.indices();
            let p = |v: u32| mesh.vertex(sb, v as usize);
            let mut crossings = 0;
            for i in 0..idx.len() {
                for j in i + 1..idx.len() {
                    let (ei, ej) = (idx[i], idx[j]);
                    if ei.iter().any(|v| ej.contains(v)) {
                        continue;
                    }
                    if segments_cross(p(ei[0]), p(ei[1]), p(ej[0]), p(ej[1])) {
                        crossings += 1;
                    }
                }
            }
            crossings
        };
        for step in 0..=steps() {
            if step % 100 == 0 {
                println!(
                    "guided {guided} step {step:4}: crossings={} area={:+.3}",
                    self_crossings(&world),
                    world.soft_bodies[a].volume()
                );
            }
            world.step();
        }
        self_crossings(&world)
    };
    let unguided = run(false);
    let guided = run(true);
    println!("end crossings: unguided {unguided}, guided {guided}");
    assert_eq!(guided, 0, "the guided repulsion never pulled the tip back");
}

/// A blob on the floor with two bottom vertices tunneled through it, manifold constraints kept:
/// the volume constraint pulls the vertices back while the manifold constraints of the same
/// features may fight it. Every `overlap_patch_constraints` policy must recover the vertices.
#[test]
fn patch_constraints_tunneled_vertices_recover() {
    let run = |policy: SoftPatchConstraints| -> (usize, Real) {
        let mut world = PhysicsWorld::new();
        floor(&mut world);
        let a = world.insert_soft_body(blob(Vector::new(0.0, 0.56), 0.5, 24));
        // Vertices 17, 18, 19 are the bottom of the loop (angles around 270 degrees).
        for v in [17usize, 18, 19] {
            let p = world.soft_bodies[a].particle_position(v);
            world.soft_bodies[a].set_particle_position(v, Vector::new(p.x, p.y - 0.2));
        }
        configure(&mut world);
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .overlap_patch_constraints = policy;
        let below = |world: &PhysicsWorld| {
            world.soft_bodies[a]
                .particle_positions()
                .filter(|p| p.y < 0.0)
                .count()
        };
        for step in 0..=steps() {
            if step % 100 == 0 {
                println!(
                    "{policy:?} step {step:4}: below floor={} speed={:.4}",
                    below(&world),
                    max_speed(&world, a)
                );
            }
            world.step();
        }
        (below(&world), max_speed(&world, a))
    };
    let mut failed = Vec::new();
    for policy in [
        SoftPatchConstraints::Keep,
        SoftPatchConstraints::StandDown,
        SoftPatchConstraints::AlongNormal,
    ] {
        let (below, speed) = run(policy);
        println!("{policy:?}: below floor {below}, speed {speed:.4}");
        if below != 0 {
            failed.push(policy);
        }
    }
    assert!(
        failed.is_empty(),
        "{failed:?} left vertices under the floor"
    );
}

/// A concave "U" whose legs are pressed into each other (a self-contact between distinct regions
/// of one surface): the self-region constraints de-overlap the legs like a pair of bodies.
#[test]
fn self_regions_deoverlap_legs() {
    let run = |regions: bool| -> usize {
        let mut world = PhysicsWorld::new();
        floor(&mut world);
        // A U, counter-clockwise, densified to 0.2 segments.
        let corners = [
            Vector::new(0.0, 0.0),
            Vector::new(1.4, 0.0),
            Vector::new(1.4, 1.2),
            Vector::new(1.0, 1.2),
            Vector::new(1.0, 0.4),
            Vector::new(0.4, 0.4),
            Vector::new(0.4, 1.2),
            Vector::new(0.0, 1.2),
        ];
        let mut points = Vec::new();
        for i in 0..corners.len() {
            let (a, b) = (corners[i], corners[(i + 1) % corners.len()]);
            let n = ((b - a).length() / 0.2).round().max(1.0) as usize;
            for k in 0..n {
                points.push(a + (b - a) * (k as Real / n as Real));
            }
        }
        let lift = Vector::new(-0.7, 0.1);
        let points: Vec<Vector> = points.iter().map(|p| *p + lift).collect();
        let h = world.insert_soft_body(
            SoftBodyBuilder::polygon(points.clone())
                .softness(SpringCoefficients::new(20.0, 1.0))
                .self_contacts(true)
                .particle_mass(0.05)
                .particle_radius(0.06)
                .surface_collider(ColliderBuilder::ball(0.06).friction(0.6)),
        );
        // The right leg pushed into the left one (their gap is 0.6): by 0.15 by default,
        // `SOFT_LEG_PUSH` sets the shift.
        let shift: Real = std::env::var("SOFT_LEG_PUSH")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.75);
        for (i, p) in points.iter().enumerate() {
            if p.x - lift.x > 0.9 && p.y - lift.y > 0.4 {
                world.soft_bodies[h].set_particle_position(i, *p - Vector::new(shift, 0.0));
            }
        }
        configure(&mut world);
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .overlap_self_regions = regions;
        world.step();
        let constraints = world.soft_bodies[h].volume_contacts().count();
        println!("regions {regions}: {constraints} volume constraints after the first step");
        assert_eq!(
            constraints > 0,
            regions,
            "the self-region constraints are {}",
            if regions { "missing" } else { "unexpected" }
        );
        let self_crossings = |world: &PhysicsWorld| {
            let sb = &world.soft_bodies[h];
            let mesh = sb.meshes().next().unwrap();
            let idx = mesh.indices();
            let p = |v: u32| mesh.vertex(sb, v as usize);
            let mut crossings = 0;
            for i in 0..idx.len() {
                for j in i + 1..idx.len() {
                    let (ei, ej) = (idx[i], idx[j]);
                    if ei.iter().any(|v| ej.contains(v)) {
                        continue;
                    }
                    if segments_cross(p(ei[0]), p(ei[1]), p(ej[0]), p(ej[1])) {
                        crossings += 1;
                    }
                }
            }
            crossings
        };
        for step in 0..=steps() {
            if step % 100 == 0 {
                println!(
                    "regions {regions} step {step:4}: crossings={} area={:+.3} speed={:.3}",
                    self_crossings(&world),
                    world.soft_bodies[h].volume(),
                    max_speed(&world, h)
                );
            }
            world.step();
        }
        self_crossings(&world)
    };
    let plain = run(false);
    let regions = run(true);
    println!("end crossings: plain {plain}, regions {regions}");
    assert_eq!(
        regions, 0,
        "the self-region constraints never de-overlapped the legs"
    );
}
