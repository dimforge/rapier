//! Headless probes of the `debug_self_intersect2` demo family: surfaces that start
//! self-intersecting must recover instead of being held tangled by their own self-contact
//! constraints (docs/self-intersection-recovery-plan.md); the 8-shaped blobs probe open problems.

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

fn probe(world: &PhysicsWorld, h: SoftBodyHandle) -> (usize, usize, Real) {
    let sb = &world.soft_bodies[h];
    // Boundary self-crossings among non-adjacent surface segments.
    let mesh = sb.meshes().next().unwrap();
    let indices = mesh.indices();
    let mut crossings = 0;
    for i in 0..indices.len() {
        for j in i + 1..indices.len() {
            let (ei, ej) = (indices[i], indices[j]);
            if ei.iter().any(|v| ej.contains(v)) {
                continue;
            }
            let p = |v: u32| mesh.vertex(sb, v as usize);
            if segments_cross(p(ei[0]), p(ei[1]), p(ej[0]), p(ej[1])) {
                crossings += 1;
            }
        }
    }
    // Inverted cells.
    let mut inverted = 0;
    let mut min_ratio: Real = Real::MAX;
    for cell in sb.cells() {
        let x: [Vector; 3] = core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
        let vol = (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5;
        let ratio = vol / cell.rest_volume;
        min_ratio = min_ratio.min(ratio);
        if ratio <= 0.0 {
            inverted += 1;
        }
    }
    (crossings, inverted, min_ratio)
}

const SELF_CONTACTS: bool = option_env!("NO_SELF_CONTACTS").is_none();
const BLOB_HZ: Real = if option_env!("STIFF_BLOB").is_some() { 20.0 } else { 4.0 };

/// A closed blob (no cells: edges + boundary-volume) whose top vertex starts pushed down
/// through the bottom wall: does the cell-less capture recover?
#[test]
fn blob_vertex_capture() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0)),
        ColliderBuilder::cuboid(4.0, 0.5),
    );
    let h = world.insert_soft_body(
        SoftBodyBuilder::disk(Vector::new(0.0, 0.0), 0.5, 16)
            .softness(SpringCoefficients::new(BLOB_HZ, 1.0))
            .self_contacts(SELF_CONTACTS)
            .particle_mass(0.05),
    );
    // Vertex 4 sits at the top of the disk (angle 90°); park it below the bottom wall.
    world.soft_bodies[h].set_particle_position(4, Vector::new(0.0, -0.8));

    for step in 0..=600 {
        if step % 60 == 0 {
            let (crossings, _, _) = probe(&world, h);
            let p4 = world.soft_bodies[h].particle_position(4);
            println!(
                "step {step:4}: crossings={crossings} p4=({:+.3},{:+.3})",
                p4.x, p4.y
            );
        }
        world.step();
    }
    let (crossings, _, _) = probe(&world, h);
    println!("final: crossings={crossings}");
    assert_eq!(crossings, 0, "the blob's boundary is still self-crossed");
}

#[test]
fn self_intersect_recovery() {
    let mut world = PhysicsWorld::new();
    let strip = SoftBodyBuilder::grid(Vector::ZERO, Vector::new(3.0, 0.15), 3, 2)
        .pinned_particles([0, 1, 4, 5])
        .softness(SpringCoefficients::new(3.0, 1.0))
        .self_contacts(SELF_CONTACTS);
    let h = world.insert_soft_body(strip);
    world.soft_bodies[h].set_particle_position(3, Vector::new(1.0, -0.4));

    for step in 0..=600 {
        if step % 60 == 0 {
            let (crossings, inverted, min_ratio) = probe(&world, h);
            let p2 = world.soft_bodies[h].particle_position(2);
            let p3 = world.soft_bodies[h].particle_position(3);
            println!(
                "step {step:4}: crossings={crossings} inverted_cells={inverted} \
                 min_vol_ratio={min_ratio:+.3} p2=({:+.3},{:+.3}) p3=({:+.3},{:+.3})",
                p2.x, p2.y, p3.x, p3.y
            );
        }
        world.step();
    }
    let (crossings, inverted, _) = probe(&world, h);
    println!("final: crossings={crossings} inverted_cells={inverted}");
    assert_eq!(crossings, 0, "the strip's boundary is still self-crossed");
    assert_eq!(inverted, 0, "the strip still has inverted cells");
}


/// Boundary crossings between two bodies, and how many of each body's vertices lie inside
/// the other (even-odd parity along +x).
fn pair_overlap(
    world: &PhysicsWorld,
    a: SoftBodyHandle,
    b: SoftBodyHandle,
) -> (usize, usize, usize) {
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
    let inside = |point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody| {
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
    };
    let a_in_b = (0..ma.vertex_count())
        .filter(|&v| inside(pa(v as u32), mb, sb))
        .count();
    let b_in_a = (0..mb.vertex_count())
        .filter(|&v| inside(pb(v as u32), ma, sa))
        .count();
    (crossings, a_in_b, b_in_a)
}

fn eight_blob(center: Vector, radius: Real, n: usize) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, n)
        .softness(SpringCoefficients::new(4.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
}

/// Reshapes a disk blob into an asymmetric 8: a big CCW lobe and a small mirrored (CW) lobe,
/// two circles joined near `center`. The signed area stays clearly positive, but the small
/// lobe's segments are wound backward.
fn make_asym_eight(
    world: &mut PhysicsWorld,
    h: SoftBodyHandle,
    center: Vector,
    rb: Real,
    rs: Real,
) {
    let n = world.soft_bodies[h].particles().len();
    let n_big = (n as Real * rb / (rb + rs)) as usize;
    let tau = core::f32::consts::TAU as Real;
    for i in 0..n {
        let p = if i < n_big {
            let t = tau * i as Real / n_big as Real;
            center + Vector::new(-rb + rb * t.cos(), rb * t.sin())
        } else {
            let t = tau * (i - n_big) as Real / (n - n_big) as Real;
            center + Vector::new(rs - rs * t.cos(), -rs * t.sin())
        };
        world.soft_bodies[h].set_particle_position(i, p);
    }
}

fn eight_floor(world: &mut PhysicsWorld) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(6.0, 0.5),
    );
}

/// An 8 with a clearly smaller mirrored lobe recovers to an O: the area deficit gives the
/// volume constraint a strong gradient, and the stand-down lets the lobe flip back through.
#[test]
fn asymmetric_eight_recovers() {
    let mut world = PhysicsWorld::new();
    eight_floor(&mut world);
    let a = world.insert_soft_body(eight_blob(Vector::new(0.0, 0.65), 0.6, 24));
    make_asym_eight(&mut world, a, Vector::new(0.1, 0.65), 0.55, 0.3);
    for _ in 0..600 {
        world.step();
    }
    let (crossings, _, _) = probe(&world, a);
    assert_eq!(crossings, 0, "the asymmetric 8 never recovered");
}

/// An 8 whose mirrored lobe starts swallowed inside a bigger blob still recovers, and the
/// pair de-overlaps.
#[test]
fn swallowed_lobe_recovers() {
    let mut world = PhysicsWorld::new();
    eight_floor(&mut world);
    let a = world.insert_soft_body(eight_blob(Vector::new(-0.5, 0.65), 0.6, 24));
    make_asym_eight(&mut world, a, Vector::new(-0.7, 0.65), 0.55, 0.3);
    let b = world.insert_soft_body(eight_blob(Vector::new(0.15, 0.65), 0.55, 20));
    for _ in 0..600 {
        world.step();
    }
    let (sa, _, _) = probe(&world, a);
    let (sb, _, _) = probe(&world, b);
    let (cross, a_in_b, b_in_a) = pair_overlap(&world, a, b);
    assert_eq!((sa, sb), (0, 0), "a blob is still self-crossed");
    assert_eq!((cross, a_in_b, b_in_a), (0, 0, 0), "the blobs still overlap");
}

/// The demo capture with a heavy ball parked on top of the tangled region: the load opposes
/// the elastic recovery, and the stand-down must still resolve the crossing (the probed
/// candidate trigger for active untangling; the elasticity won).
#[test]
fn loaded_tangle_recovers() {
    let mut world = PhysicsWorld::new();
    let strip = SoftBodyBuilder::grid(Vector::ZERO, Vector::new(3.0, 0.15), 3, 2)
        .pinned_particles([0, 1, 4, 5])
        .softness(SpringCoefficients::new(3.0, 1.0))
        .self_contacts(true);
    let h = world.insert_soft_body(strip);
    world.soft_bodies[h].set_particle_position(3, Vector::new(1.0, -0.4));
    // A ball ~20x the strip's mass resting right above the tangle.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.3, 0.8)),
        ColliderBuilder::ball(0.45).density(20.0),
    );

    for step in 0..=900 {
        if step % 60 == 0 {
            let (crossings, inverted, min_ratio) = probe(&world, h);
            let p3 = world.soft_bodies[h].particle_position(3);
            println!(
                "step {step:4}: crossings={crossings} inverted={inverted} \
                 min_ratio={min_ratio:+.3} p3=({:+.3},{:+.3})",
                p3.x, p3.y
            );
        }
        world.step();
    }
    let (crossings, inverted, _) = probe(&world, h);
    assert_eq!(crossings, 0, "the loaded tangle never resolved");
    assert_eq!(inverted, 0, "cells stayed inverted under the load");
}

/// Rough step-time probe: a pile of resting self-colliding blobs (no tangles), to price the
/// per-step crossing sweep.
#[test]
#[ignore]
fn pile_timing() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0)),
        ColliderBuilder::cuboid(6.0, 0.5),
    );
    for j in 0..4 {
        for i in 0..6 {
            let x = -3.0 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.2 + j as Real * 1.1;
            world.insert_soft_body(
                SoftBodyBuilder::disk(Vector::new(x, y), 0.5, 24)
                    .softness(SpringCoefficients::new(20.0, 1.0))
                    .self_contacts(true)
                    .particle_mass(0.05),
            );
        }
    }
    for _ in 0..120 {
        world.step();
    }
    let start = std::time::Instant::now();
    for _ in 0..2000 {
        world.step();
    }
    println!("pile: {:.3} ms/step", start.elapsed().as_secs_f64() * 1000.0 / 2000.0);
}

/// A rope threaded through a pinned rope slides free under gravity instead of hanging frozen:
/// edge constraints touching the mutual crossing stand down.
#[test]
fn rope_through_rope_slides_free() {
    let mut world = PhysicsWorld::new();
    let horizontal = world.insert_soft_body(
        SoftBodyBuilder::rope(Vector::new(-1.5, 0.0), Vector::new(1.5, 0.0), 16)
            .pinned_particles([0, 15])
            .particle_mass(0.05)
            .particle_radius(0.05),
    );
    let vertical = world.insert_soft_body(
        SoftBodyBuilder::rope(Vector::new(0.02, -1.0), Vector::new(0.02, 1.0), 12)
            .particle_mass(0.05)
            .particle_radius(0.05),
    );
    let crossings = |world: &PhysicsWorld| {
        let (cross, _, _) = pair_overlap(world, horizontal, vertical);
        cross
    };
    assert!(crossings(&world) > 0, "the ropes were authored crossed");
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[vertical].center_of_mass();
    println!("crossings {} com_y {:+.3}", crossings(&world), com.y);
    assert_eq!(crossings(&world), 0, "the ropes are still crossed");
    assert!(com.y < -1.5, "the vertical rope hangs frozen at {:+.3}", com.y);
}

/// With the whole `soft_bodies.recovery` stack disabled a self-intersecting strip stays
/// tangled, and with the defaults it recovers: every toggle is read.
#[test]
fn recovery_toggles_are_wired() {
    let build = || {
        let mut world = PhysicsWorld::new();
        let strip = SoftBodyBuilder::grid(Vector::ZERO, Vector::new(3.0, 0.15), 3, 2)
            .pinned_particles([0, 1, 4, 5])
            .softness(SpringCoefficients::new(3.0, 1.0))
            .self_contacts(true);
        let h = world.insert_soft_body(strip);
        world.soft_bodies[h].set_particle_position(3, Vector::new(1.0, -0.4));
        (world, h)
    };

    // Defaults: recovers.
    let (mut world, h) = build();
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(probe(&world, h).0, 0, "defaults did not recover the strip");

    // Everything off: stays tangled, like the pre-recovery baseline.
    let (mut world, h) = build();
    world.integration_parameters.soft_bodies.recovery = SoftRecoverySettings {
        authored_velocity_margin: false,
        edge_speculation: false,
        inverted_cell_detection: false,
        self_crossing_detection: false,
        detection_motion_gating: false,
        cross_body_detection: false,
        self_stand_down: false,
        cross_body_expel_gate: false,
        edge_stand_down: false,
        crossing_repulsion: false,
        crossing_repulsion_guide: false,
        recovery_pace: 0.5,
        overlap_constraints: false,
        overlap_rigid: true,
        overlap_skip_self_tangled: true,
        overlap_edge_stand_down: true,
        overlap_constraint_pace: 1.0,
        overlap_patch_constraints: SoftPatchConstraints::Keep,
        overlap_skin_volume: false,
        overlap_kept_depth: 0.0,
        overlap_normal_push: false,
        overlap_self_regions: false,
        overlap_multi_volume: false,
        overlap_split: 3,
        overlap_patience: 240,
        overlap_progress_margin: 0.02,
    };
    for _ in 0..600 {
        world.step();
    }
    assert!(
        probe(&world, h).0 > 0,
        "disabling every recovery toggle still recovered the strip: a toggle is unwired"
    );
}

/// Crossing repulsion versus the freeze: with the self stand-down off, the self contacts
/// of a strip vertex flicked through its bottom edge hold the crossing forever; with
/// repulsion the same constraints push the vertex back to the side its neighbors are on.
#[test]
fn crossing_repulsion_recovers_flick() {
    for repulsion in [true, false] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let r = &mut world.integration_parameters.soft_bodies.recovery;
        r.authored_velocity_margin = false;
        r.self_stand_down = false;
        r.crossing_repulsion = repulsion;
        let strip = SoftBodyBuilder::grid(Vector::ZERO, Vector::new(3.0, 0.15), 3, 2)
            .pinned_particles([0, 1, 4, 5])
            .softness(SpringCoefficients::new(3.0, 1.0))
            .particle_radius(0.02)
            .self_contacts(true);
        let h = world.insert_soft_body(strip);
        for _ in 0..5 {
            world.step();
        }
        world.soft_bodies[h].set_particle_velocity(3, Vector::new(0.0, -600.0));
        for _ in 0..120 {
            world.step();
        }
        let end = probe(&world, h).0;
        println!("repulsion={repulsion}: end={end}");
        if repulsion {
            assert_eq!(end, 0, "crossing repulsion did not push the vertex back");
        } else {
            assert!(end > 0, "the frozen constraints did not keep the strip crossed");
        }
    }
}
