//! Soft-body recovery scenes: stacks and pressed piles that must stay uncrossed and uninverted,
//! self-intersecting surfaces that must untangle, and overlapping pairs that must de-overlap
//! through the intersection-volume constraints. Every scene runs the default recovery settings
//! unless it tests an opt-in toggle.

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

/// Boundary crossings among the non-adjacent surface segments of a body.
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

/// The body's cells with a non-positive area.
fn inverted_cells(world: &PhysicsWorld, h: SoftBodyHandle) -> usize {
    let sb = &world.soft_bodies[h];
    sb.cells()
        .iter()
        .filter(|cell| {
            let x: [Vector; 3] =
                core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
            (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5 / cell.rest_volume <= 0.0
        })
        .count()
}

/// Boundary crossings between two bodies, and how many of each body's vertices lie inside
/// the other.
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
    let a_in_b = (0..ma.vertex_count())
        .filter(|&v| inside(pa(v as u32), mb, sb))
        .count();
    let b_in_a = (0..mb.vertex_count())
        .filter(|&v| inside(pb(v as u32), ma, sa))
        .count();
    (crossings, a_in_b, b_in_a)
}

/// The crossings within and between the bodies of a stack, and their inverted cells.
#[derive(Clone, Copy, Debug, Default)]
struct Metrics {
    self_crossings: usize,
    cross_crossings: usize,
    inverted_cells: usize,
}

fn measure(world: &PhysicsWorld, handles: &[SoftBodyHandle]) -> Metrics {
    let mut m = Metrics::default();
    for (i, &a) in handles.iter().enumerate() {
        m.self_crossings += self_crossings(world, a);
        m.inverted_cells += inverted_cells(world, a);
        for &b in &handles[i + 1..] {
            m.cross_crossings += pair_overlap(world, a, b).0;
        }
    }
    m
}

/// Runs `steps` steps, `hook` before each of them (kinematic drivers), and measures the end state.
fn run(
    world: &mut PhysicsWorld,
    handles: &[SoftBodyHandle],
    steps: usize,
    mut hook: impl FnMut(&mut PhysicsWorld, usize),
) -> Metrics {
    for step in 0..steps {
        hook(world, step);
        world.step();
    }
    measure(world, handles)
}

fn floor(world: &mut PhysicsWorld, half_width: Real, friction: Real) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(half_width, 0.5).friction(friction),
    );
}

/// A floor with two walls: the bin of the pressed piles.
fn bin(world: &mut PhysicsWorld) {
    floor(world, 3.5, 0.5);
    for x in [-3.5, 3.5] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 4.0)),
            ColliderBuilder::cuboid(0.5, 5.0),
        );
    }
}

/// A cell-less loop stiff enough to bear another body.
fn stiff_blob(center: Vector, radius: Real, n: usize) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, n)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

/// A soft cell-less loop that folds easily.
fn soft_blob(center: Vector, radius: Real, n: usize) -> SoftBodyBuilder {
    SoftBodyBuilder::disk(center, radius, n)
        .softness(SpringCoefficients::new(4.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
}

/// A solid corotational square (its flat faces neither roll nor tip, unlike a blob).
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

/// A soft strip pinned at both ends, from the `debug_self_intersect2` demo.
fn strip() -> SoftBodyBuilder {
    SoftBodyBuilder::grid(Vector::ZERO, Vector::new(3.0, 0.15), 3, 2)
        .pinned_particles([0, 1, 4, 5])
        .softness(SpringCoefficients::new(3.0, 1.0))
        .self_contacts(true)
}

/// The strip with its bottom-middle vertex pushed through its bottom edge.
fn tangled_strip(world: &mut PhysicsWorld) -> SoftBodyHandle {
    let h = world.insert_soft_body(strip());
    world.soft_bodies[h].set_particle_position(3, Vector::new(1.0, -0.4));
    h
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

/// A column of twelve cell-less blobs dropped one on the other: the bottom blobs bear the
/// whole column's weight and must neither be pushed through the floor nor into each other.
#[test]
fn blob_column() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 4.0, 0.5);
    let handles: Vec<_> = (0..12)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(stiff_blob(Vector::new(x, 0.55 + i as Real * 1.05), 0.5, 20))
        })
        .collect();
    // A heavy box lands on the column once it has settled (twenty times a blob's mass).
    let m = run(&mut world, &handles, 1200, |world, step| {
        if step == 300 {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 14.0)),
                ColliderBuilder::cuboid(0.6, 0.4).density(10.0),
            );
        }
    });
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
    floor(&mut world, 4.0, 0.5);
    let handles: Vec<_> = (0..8)
        .map(|i| {
            let x = (i % 2) as Real * 0.05;
            world.insert_soft_body(jelly(Vector::new(x, 0.55 + i as Real * 1.1), 0.5))
        })
        .collect();
    let m = run(&mut world, &handles, 1200, |world, step| {
        if step == 300 {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
                ColliderBuilder::cuboid(0.6, 0.4).density(10.0),
            );
        }
    });
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
    bin(&mut world);
    let mut handles = Vec::new();
    for j in 0..4 {
        for i in 0..5 {
            let x = -2.4 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.55 + j as Real * 1.05;
            handles.push(world.insert_soft_body(stiff_blob(Vector::new(x, y), 0.5, 20)));
        }
    }
    let plate = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 5.0)),
    );
    world.insert_collider(ColliderBuilder::cuboid(2.9, 0.25), Some(plate));
    // Settle, press down at 0.5 u/s to y = 3.0 (the pile starts ~4.3 high), then hold.
    let m = run(&mut world, &handles, 1500, |world, step| {
        if step >= 300 {
            let y = (5.0 - (step - 300) as Real * 0.5 / 60.0).max(3.0);
            world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
        }
    });
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
    bin(&mut world);
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
    let m = run(&mut world, &handles, 1500, |world, step| {
        if step >= 300 {
            let y = (4.0 - (step - 300) as Real * 0.5 / 60.0).max(2.7);
            world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
        }
    });
    assert_eq!(
        m.cross_crossings, 0,
        "squares of the pressed pile penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a square of the pressed pile has inverted cells"
    );
}

/// A closed blob (no cells: edges + boundary-volume) whose top vertex starts pushed down
/// through the bottom wall recovers.
#[test]
fn blob_vertex_capture() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0)),
        ColliderBuilder::cuboid(4.0, 0.5),
    );
    let h = world.insert_soft_body(soft_blob(Vector::new(0.0, 0.0), 0.5, 16));
    // Vertex 4 sits at the top of the disk (angle 90°); park it below the bottom wall.
    world.soft_bodies[h].set_particle_position(4, Vector::new(0.0, -0.8));
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, h),
        0,
        "the blob's boundary is still self-crossed"
    );
}

/// An 8 with a clearly smaller mirrored lobe recovers to an O: the area deficit gives the
/// volume constraint a strong gradient, and the stand-down lets the lobe flip back through.
#[test]
fn asymmetric_eight_recovers() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.5);
    let a = world.insert_soft_body(soft_blob(Vector::new(0.0, 0.65), 0.6, 24));
    make_asym_eight(&mut world, a, Vector::new(0.1, 0.65), 0.55, 0.3);
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, a),
        0,
        "the asymmetric 8 never recovered"
    );
}

/// An 8 whose mirrored lobe starts swallowed inside a bigger blob still recovers, and the
/// pair de-overlaps.
#[test]
fn swallowed_lobe_recovers() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.5);
    let a = world.insert_soft_body(soft_blob(Vector::new(-0.5, 0.65), 0.6, 24));
    make_asym_eight(&mut world, a, Vector::new(-0.7, 0.65), 0.55, 0.3);
    let b = world.insert_soft_body(soft_blob(Vector::new(0.15, 0.65), 0.55, 20));
    for _ in 0..600 {
        world.step();
    }
    let (sa, sb) = (self_crossings(&world, a), self_crossings(&world, b));
    assert_eq!((sa, sb), (0, 0), "a blob is still self-crossed");
    assert_eq!(
        pair_overlap(&world, a, b),
        (0, 0, 0),
        "the blobs still overlap"
    );
}

/// The tangled strip with a heavy ball parked on top of the tangled region: the load opposes
/// the elastic recovery, and the stand-down must still resolve the crossing.
#[test]
fn loaded_tangle_recovers() {
    let mut world = PhysicsWorld::new();
    let h = tangled_strip(&mut world);
    // A ball ~20x the strip's mass resting right above the tangle.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.3, 0.8)),
        ColliderBuilder::ball(0.45).density(20.0),
    );
    for _ in 0..900 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, h),
        0,
        "the loaded tangle never resolved"
    );
    assert_eq!(
        inverted_cells(&world, h),
        0,
        "cells stayed inverted under the load"
    );
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
    let crossings = |world: &PhysicsWorld| pair_overlap(world, horizontal, vertical).0;
    assert!(crossings(&world) > 0, "the ropes were authored crossed");
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[vertical].center_of_mass();
    assert_eq!(crossings(&world), 0, "the ropes are still crossed");
    assert!(
        com.y < -1.5,
        "the vertical rope hangs frozen at {:+.3}",
        com.y
    );
}

/// With the whole `soft_bodies.recovery` stack disabled the tangled strip stays tangled, and
/// with the defaults it recovers: every toggle is read.
#[test]
fn recovery_toggles_are_wired() {
    let mut world = PhysicsWorld::new();
    let h = tangled_strip(&mut world);
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, h),
        0,
        "defaults did not recover the strip"
    );

    let mut world = PhysicsWorld::new();
    let h = tangled_strip(&mut world);
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
        crossing_repulsion_self_guide: false,
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
        self_crossings(&world, h) > 0,
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
        let h = world.insert_soft_body(strip().particle_radius(0.02));
        for _ in 0..5 {
            world.step();
        }
        world.soft_bodies[h].set_particle_velocity(3, Vector::new(0.0, -600.0));
        for _ in 0..120 {
            world.step();
        }
        let end = self_crossings(&world, h);
        if repulsion {
            assert_eq!(end, 0, "crossing repulsion did not push the vertex back");
        } else {
            assert!(
                end > 0,
                "the frozen constraints did not keep the strip crossed"
            );
        }
    }
}

/// A blob whose top vertex was pushed through its own bottom (two crossings and a mirrored tip),
/// with the crossing repulsion and no volume constraint: guided by the fold's volume normal, the
/// tip is pulled back onto the pierced surface until the crossings annihilate.
#[test]
fn self_guided_repulsion_recovers_pierced_tip() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.6);
    let a = world.insert_soft_body(stiff_blob(Vector::new(0.0, 0.56), 0.5, 24));
    // Vertex 6 is the top of the loop (angle 90 degrees): pushed below the bottom.
    world.soft_bodies[a].set_particle_position(6, Vector::new(0.0, 0.56 - 0.5 - 0.15));
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    r.overlap_constraints = false;
    r.crossing_repulsion = true;
    r.crossing_repulsion_self_guide = true;
    for _ in 0..900 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, a),
        0,
        "the guided repulsion never pulled the tip back"
    );
}

/// A blob on the floor with two bottom vertices tunneled through it, manifold constraints kept:
/// the volume constraint pulls the vertices back while the manifold constraints of the same
/// features may fight it. Every `overlap_patch_constraints` policy must recover the vertices.
#[test]
fn patch_constraints_tunneled_vertices_recover() {
    let mut failed = Vec::new();
    for policy in [
        SoftPatchConstraints::Keep,
        SoftPatchConstraints::StandDown,
        SoftPatchConstraints::AlongNormal,
    ] {
        let mut world = PhysicsWorld::new();
        floor(&mut world, 6.0, 0.6);
        let a = world.insert_soft_body(stiff_blob(Vector::new(0.0, 0.56), 0.5, 24));
        // Vertices 17, 18, 19 are the bottom of the loop (angles around 270 degrees).
        for v in [17usize, 18, 19] {
            let p = world.soft_bodies[a].particle_position(v);
            world.soft_bodies[a].set_particle_position(v, Vector::new(p.x, p.y - 0.2));
        }
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .overlap_patch_constraints = policy;
        for _ in 0..900 {
            world.step();
        }
        let below = world.soft_bodies[a]
            .particle_positions()
            .filter(|p| p.y < 0.0)
            .count();
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
/// of one surface): the self-region constraints de-overlap the legs like a pair of bodies, and
/// without them no volume constraint appears.
#[test]
fn self_regions_deoverlap_legs() {
    for regions in [false, true] {
        let mut world = PhysicsWorld::new();
        floor(&mut world, 6.0, 0.6);
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
        // The right leg pushed into the left one (their gap is 0.6).
        for (i, p) in points.iter().enumerate() {
            if p.x - lift.x > 0.9 && p.y - lift.y > 0.4 {
                world.soft_bodies[h].set_particle_position(i, *p - Vector::new(0.75, 0.0));
            }
        }
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .overlap_self_regions = regions;
        world.step();
        let constraints = world.soft_bodies[h].volume_contacts().count();
        assert_eq!(
            constraints > 0,
            regions,
            "the self-region constraints are {}",
            if regions { "missing" } else { "unexpected" }
        );
        if !regions {
            continue;
        }
        for _ in 0..900 {
            world.step();
        }
        assert_eq!(
            self_crossings(&world, h),
            0,
            "the self-region constraints never de-overlapped the legs"
        );
    }
}

/// Two cell-less blobs inserted overlapping by a third of a radius: each one's intruding
/// arc retracts into its own body, and the pair de-overlaps with no expulsion.
#[test]
fn overlapping_blobs_deoverlap() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.5);
    let a = world.insert_soft_body(soft_blob(Vector::new(-0.42, 0.6), 0.6, 24));
    let b = world.insert_soft_body(soft_blob(Vector::new(0.42, 0.6), 0.6, 24));
    for _ in 0..900 {
        world.step();
    }
    assert_eq!(
        pair_overlap(&world, a, b),
        (0, 0, 0),
        "the blobs still overlap"
    );
}

/// A dynamic rigid box inserted inside a hollow blob: the blob's patch inside the box and the
/// box itself share one constraint, and the pair separates without the box being thrown (a box,
/// not a ball: a ball keeps rolling with whatever it was handed).
#[test]
fn rigid_box_inside_blob_separates() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.5);
    let a = world.insert_soft_body(soft_blob(Vector::new(0.0, 0.65), 0.6, 24));
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.3, 0.65)),
        ColliderBuilder::cuboid(0.3, 0.3).density(1.0),
    );
    for _ in 0..600 {
        world.step();
    }
    let c = world.bodies[rb].center_of_mass();
    let inside_box = world.soft_bodies[a]
        .particle_positions()
        .filter(|p| (p.x - c.x).abs() < 0.3 && (p.y - c.y).abs() < 0.3)
        .count();
    assert_eq!(inside_box, 0, "the blob still has vertices inside the box");
    let speed = world.bodies[rb].linvel().length();
    assert!(speed < 1.0, "the box was thrown: {speed} u/s");
}

/// A hollow blob dropped onto a thin fixed pin that ends up inside it (the pile's impaled
/// blobs): the rigid overlap constraint lifts the blob off the pin.
#[test]
fn impaled_blob_recovers() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0, 0.5);
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 1.5)),
        ColliderBuilder::capsule_x(0.6, 0.1).friction(0.5),
    );
    let a = world.insert_soft_body(soft_blob(Vector::new(0.0, 1.5), 0.6, 24));
    let pin_inside = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[a];
        inside(Vector::new(0.0, 1.5), sb.meshes().next().unwrap(), sb)
    };
    assert!(pin_inside(&world), "the pin was authored inside the blob");
    for _ in 0..600 {
        world.step();
    }
    assert!(!pin_inside(&world), "the pin is still inside the blob");
}
