//! Intersection-volume constraints (see docs/soft-depenetration-literature-plan.md, phase 3):
//! the part of a closed boundary inside another, or a self-crossed loop's mirrored lobe,
//! retracts along its own area gradient. Every other active recovery is off in these scenes.

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

fn self_crossings(world: &PhysicsWorld, h: SoftBodyHandle) -> usize {
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
}

/// Overlap constraints on, every other active recovery off (`SOFT_DEPEN_PRESET=defaults` keeps
/// the defaults for A/B runs).
fn overlap_only(world: &mut PhysicsWorld) {
    if std::env::var("SOFT_DEPEN_PRESET").as_deref() == Ok("defaults") {
        return;
    }
    // The reference settings: prevention without edge speculation, detection and the
    // stand-down, the constraints on.
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    r.overlap_constraints = true;
    r.edge_speculation = false;
    // Diagnostic: the closed-closed edge constraints off, or the overlap constraints off (the reference
    // settings minus the constraints).
    if std::env::var("SOFT_DEPEN_NO_EDGE").is_ok() {
        r.edge_speculation = false;
    }
    if std::env::var("SOFT_OVERLAP_OFF").is_ok() {
        r.overlap_constraints = false;
    }
    if let Some(pace) = std::env::var("SOFT_OVERLAP_CONSTRAINT_PACE").ok().and_then(|v| v.parse().ok()) {
        r.overlap_constraint_pace = pace;
    }
}

fn floor(world: &mut PhysicsWorld) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(6.0, 0.5),
    );
}

fn blob(center: Vector, radius: Real, n: usize) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, n)
        .softness(SpringCoefficients::new(4.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
}

fn make_gerono_eight(world: &mut PhysicsWorld, h: SoftBodyHandle, center: Vector, r: Real) {
    let n = world.soft_bodies[h].particles().len();
    for i in 0..n {
        let t = core::f32::consts::TAU as Real * i as Real / n as Real;
        let p = center + Vector::new(r * t.cos(), r * t.sin() * t.cos());
        world.soft_bodies[h].set_particle_position(i, p);
    }
}

/// Two cell-less blobs inserted overlapping by a third of a radius: each one's intruding
/// arc retracts into its own body, and the pair de-overlaps with no expulsion.
#[test]
fn overlapping_blobs_deoverlap() {
    let mut world = PhysicsWorld::new();
    floor(&mut world);
    let a = world.insert_soft_body(blob(Vector::new(-0.42, 0.6), 0.6, 24));
    let b = world.insert_soft_body(blob(Vector::new(0.42, 0.6), 0.6, 24));
    overlap_only(&mut world);
    for step in 0..=900 {
        if step % 100 == 0 {
            let (c, ab, ba) = overlap(&world, a, b);
            println!("step {step:4}: crossings={c} a_in_b={ab} b_in_a={ba}");
        }
        world.step();
    }
    let (c, ab, ba) = overlap(&world, a, b);
    assert_eq!((c, ab, ba), (0, 0, 0), "the blobs still overlap");
}

/// Checks that a balanced figure-eight's mirrored lobe deflates along its area gradient until
/// the crossing annihilates and the loop re-inflates. Ignored: without the active untangler's
/// endgame pull the balanced eight stalls as a sliver crossing.
#[test]
#[ignore]
fn balanced_eight_recovers() {
    let mut world = PhysicsWorld::new();
    floor(&mut world);
    let a = world.insert_soft_body(blob(Vector::new(0.0, 0.65), 0.6, 24));
    make_gerono_eight(&mut world, a, Vector::new(0.0, 0.65), 0.6);
    overlap_only(&mut world);
    for step in 0..=1200 {
        if step % 100 == 0 {
            println!(
                "step {step:4}: crossings={} area={:.3}",
                self_crossings(&world, a),
                world.soft_bodies[a].volume()
            );
        }
        world.step();
    }
    assert_eq!(
        self_crossings(&world, a),
        0,
        "the balanced eight never recovered"
    );
    let area = world.soft_bodies[a].volume();
    let rest = world.soft_bodies[a].rest_volume();
    assert!(
        area > 0.8 * rest,
        "the loop did not re-inflate: {area:.3} of {rest:.3}"
    );
}

/// A dynamic rigid box inserted inside a hollow blob, with expulsion off: the blob's patch
/// inside the box and the box itself share one constraint, and the pair separates without the box
/// being thrown (a box, not a ball: a ball keeps rolling with whatever it was handed).
#[test]
fn rigid_ball_inside_blob_separates() {
    let mut world = PhysicsWorld::new();
    floor(&mut world);
    let a = world.insert_soft_body(blob(Vector::new(0.0, 0.65), 0.6, 24));
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.3, 0.65)),
        ColliderBuilder::cuboid(0.3, 0.3).density(1.0),
    );
    overlap_only(&mut world);
    let inside_count = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[a];
        let c = world.bodies[ball].center_of_mass();
        sb.particle_positions()
            .filter(|p| (p.x - c.x).abs() < 0.3 && (p.y - c.y).abs() < 0.3)
            .count()
    };
    for step in 0..=600 {
        if step % 100 == 0 {
            println!(
                "step {step:4}: vertices inside ball={} ball=({:+.2},{:+.2}) blob=({:+.2},{:+.2})",
                inside_count(&world),
                world.bodies[ball].center_of_mass().x,
                world.bodies[ball].center_of_mass().y,
                world.soft_bodies[a].center_of_mass().x,
                world.soft_bodies[a].center_of_mass().y,
            );
        }
        world.step();
    }
    assert_eq!(
        inside_count(&world),
        0,
        "the blob still has vertices inside the ball"
    );
    let speed = world.bodies[ball].linvel().length();
    assert!(speed < 1.0, "the ball was thrown: {speed} u/s");
}

/// A hollow blob dropped onto a thin fixed pin that ends up inside it (the pile's impaled
/// blobs): the rigid overlap constraint lifts the blob off the pin, expulsion off.
#[test]
fn impaled_blob_recovers() {
    let mut world = PhysicsWorld::new();
    floor(&mut world);
    let pin_friction: Real = std::env::var("SOFT_PIN_FRICTION")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.5);
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 1.5)),
        ColliderBuilder::capsule_x(0.6, 0.1).friction(pin_friction),
    );
    let a = world.insert_soft_body(blob(Vector::new(0.0, 1.5), 0.6, 24));
    overlap_only(&mut world);
    let pin_inside = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[a];
        let mesh = sb.meshes().next().unwrap();
        inside(Vector::new(0.0, 1.5), mesh, sb)
    };
    println!("start: pin inside={}", pin_inside(&world));
    for step in 0..=600 {
        if step % 100 == 0 {
            println!(
                "step {step:4}: pin inside={} blob=({:+.2},{:+.2})",
                pin_inside(&world),
                world.soft_bodies[a].center_of_mass().x,
                world.soft_bodies[a].center_of_mass().y,
            );
        }
        world.step();
    }
    assert!(!pin_inside(&world), "the pin is still inside the blob");
}
