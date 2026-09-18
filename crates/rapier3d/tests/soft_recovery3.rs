//! 3D soft-body recovery scenes (mechanisms in the 2D `soft_recovery`): balloon and jelly
//! columns under a heavy weight, a pressed cube pile, self-crossed surfaces that must untangle,
//! and overlapping balloons that must de-overlap. Every scene runs the default recovery
//! settings unless it tests an opt-in toggle.

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

/// The world-space triangles of a body's surface mesh.
fn triangles(body: &SoftBody, mesh: &SoftCollisionMesh) -> Vec<[Vector; 3]> {
    mesh.indices()
        .iter()
        .map(|e| core::array::from_fn(|k| mesh.vertex(body, e[k] as usize)))
        .collect()
}

/// Crossings among the non-adjacent surface triangles of a body.
fn self_crossings(world: &PhysicsWorld, h: SoftBodyHandle) -> usize {
    let sb = &world.soft_bodies[h];
    let mesh = sb.meshes().next().unwrap();
    let idx = mesh.indices();
    let tris = triangles(sb, mesh);
    let mut crossings = 0;
    for i in 0..idx.len() {
        for j in i + 1..idx.len() {
            if idx[i].iter().any(|v| idx[j].contains(v)) {
                continue;
            }
            if triangles_cross(&tris[i], &tris[j]) {
                crossings += 1;
            }
        }
    }
    crossings
}

/// The body's cells with a non-positive volume.
fn inverted_cells(world: &PhysicsWorld, h: SoftBodyHandle) -> usize {
    let sb = &world.soft_bodies[h];
    sb.cells()
        .iter()
        .filter(|cell| {
            let x: [Vector; 4] =
                core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
            let vol = (x[1] - x[0]).cross(x[2] - x[0]).dot(x[3] - x[0]);
            vol * cell.rest_volume <= 0.0
        })
        .count()
}

/// Surface crossings between two bodies, and how many of each body's vertices lie inside
/// the other (0 for an open surface).
fn pair_overlap(
    world: &PhysicsWorld,
    a: SoftBodyHandle,
    b: SoftBodyHandle,
) -> (usize, usize, usize) {
    let (sa, sb) = (&world.soft_bodies[a], &world.soft_bodies[b]);
    let (ma, mb) = (sa.meshes().next().unwrap(), sb.meshes().next().unwrap());
    let (ta, tb) = (triangles(sa, ma), triangles(sb, mb));
    let mut crossings = 0;
    for x in &ta {
        for y in &tb {
            if triangles_cross(x, y) {
                crossings += 1;
            }
        }
    }
    let count_inside = |m1: &SoftCollisionMesh, s1: &SoftBody, m2: &SoftCollisionMesh, s2| {
        if !m2.is_closed() {
            return 0;
        }
        (0..m1.vertex_count())
            .filter(|&v| inside(m1.vertex(s1, v), m2, s2))
            .count()
    };
    (
        crossings,
        count_inside(ma, sa, mb, sb),
        count_inside(mb, sb, ma, sa),
    )
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

fn floor(world: &mut PhysicsWorld, half: Real) {
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half, 0.5, half),
    );
}

/// A closed cell-less sphere stiff enough to bear another body.
fn balloon(center: Vector, radius: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::sphere(center, radius, 1)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

/// A solid corotational cube.
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

/// A stiff cloth strip of twelve columns and two rows.
fn strip(origin: Vector, du: Vector, dv: Vector, radius: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::cloth(origin, du, dv, 12, 2)
        .material(SoftBodyMaterial::uniform(SpringCoefficients::new(
            40.0, 1.0,
        )))
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(radius)
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
    let m = run(&mut world, &handles, 900, |world, step| {
        if step == 300 {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 8.0, 0.0)),
                ColliderBuilder::cuboid(0.5, 0.3, 0.5).density(10.0),
            );
        }
    });
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
    let m = run(&mut world, &handles, 900, |world, step| {
        if step == 300 {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 7.0, 0.0)),
                ColliderBuilder::cuboid(0.5, 0.3, 0.5).density(10.0),
            );
        }
    });
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
    let m = run(&mut world, &handles, 1200, |world, step| {
        if step >= 300 {
            let y = (3.0 - (step - 300) as Real * 0.5 / 60.0).max(1.95);
            world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y, 0.0));
        }
    });
    assert_eq!(
        m.cross_crossings, 0,
        "cubes of the pressed pile penetrate each other"
    );
    assert_eq!(
        m.inverted_cells, 0,
        "a cube of the pressed pile has inverted cells"
    );
}

/// A thin volumetric slab pinned at both end faces, with a top vertex parked below the bottom
/// face: cells invert and the boundary trimesh self-crosses. The stand-down (inverted cells
/// and edge-vs-triangle crossings) must let the elasticity pull it back out.
#[test]
fn slab_vertex_capture() {
    let mut world = PhysicsWorld::new();
    let builder = SoftBodyBuilder::cuboid(
        Vector::new(0.0, 0.15, 0.0),
        Vector::new(1.5, 0.15, 0.15),
        3,
        2,
        2,
    )
    .softness(SpringCoefficients::new(5.0, 1.0));
    let pinned: Vec<u32> = builder
        .particle_positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.x.abs() > 1.4)
        .map(|(i, _)| i as u32)
        .collect();
    let target = Vector::new(0.0, 0.3, 0.15);
    let captured = builder
        .particle_positions()
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (**a - target)
                .length()
                .partial_cmp(&(**b - target).length())
                .unwrap()
        })
        .unwrap()
        .0;
    let h = world.insert_soft_body(builder.pinned_particles(pinned).self_contacts(true));
    world.soft_bodies[h].set_particle_position(captured, Vector::new(0.3, -0.4, 0.0));
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, h),
        0,
        "the slab's boundary is still self-crossed"
    );
    assert_eq!(
        inverted_cells(&world, h),
        0,
        "the slab still has inverted cells"
    );
}

/// A cloth strip folded into two layers (no cells at all), with one top-layer vertex parked
/// below the bottom layer: its incident triangles pierce the bottom layer. Only the crossing
/// signal can see this; the stand-down must let the elasticity pull the vertex back through.
#[test]
fn folded_cloth_vertex_capture() {
    let mut world = PhysicsWorld::new();
    let (nu, nv) = (8usize, 3usize);
    let builder = SoftBodyBuilder::cloth(
        Vector::ZERO,
        Vector::new(0.25, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.25),
        nu,
        nv,
    )
    .material(SoftBodyMaterial::uniform(SpringCoefficients::new(
        40.0, 1.0,
    )))
    .softness(SpringCoefficients::new(40.0, 1.0))
    .particle_mass(0.02)
    .particle_radius(0.05)
    // Both ends of the strip pinned: the outer edge of the bottom layer at its rest place,
    // the free edge of the folded-back top layer above it (positions set below).
    .pinned_particles((0..nv as u32).chain((nu as u32 - 1) * nv as u32..(nu * nv) as u32))
    .self_contacts(true);
    let h = world.insert_soft_body(builder);
    // Fold columns 4..8 back over the first four, one gap above them.
    let idx = |i: usize, j: usize| i * nv + j;
    for i in 4..nu {
        for j in 0..nv {
            let x = 0.25 * (7 - i) as Real;
            world.soft_bodies[h]
                .set_particle_position(idx(i, j), Vector::new(x, 0.15, 0.25 * j as Real));
        }
    }
    // Park a middle vertex of the top layer below the bottom layer.
    let captured = idx(5, 1);
    world.soft_bodies[h].set_particle_position(captured, Vector::new(0.5, -0.15, 0.25));
    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        self_crossings(&world, h),
        0,
        "the cloth is still self-crossed"
    );
}

/// A cloth strip threaded through a pinned strip slides free under gravity instead of
/// hanging frozen on it: edge constraints touching the mutual crossing stand down (thin open
/// bodies meet through their edge constraints, so the vertex-pass gate alone cannot free them).
#[test]
fn ribbon_through_ribbon_slides_free() {
    let mut world = PhysicsWorld::new();
    let horizontal = world.insert_soft_body(
        strip(
            Vector::new(-1.1, 0.0, -0.1),
            Vector::new(0.2, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.2),
            0.04,
        )
        .pinned_particles([0, 1, 22, 23]),
    );
    // Vertical, in the xz-plane of the pinned strip's middle, crossing its surface.
    let vertical = world.insert_soft_body(strip(
        Vector::new(0.03, -1.1, -0.1),
        Vector::new(0.0, 0.2, 0.0),
        Vector::new(0.0, 0.0, 0.2),
        0.04,
    ));
    let crossings = |world: &PhysicsWorld| pair_overlap(world, horizontal, vertical).0;
    assert!(crossings(&world) > 0, "the strips were authored crossed");
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[vertical].center_of_mass();
    assert_eq!(crossings(&world), 0, "the strips are still crossed");
    assert!(
        com.y < -1.5,
        "the vertical strip hangs frozen at {:+.3}",
        com.y
    );
}

/// Two closed slim bars crossing edge-first collide instead of passing through: closed-vs-closed
/// pairs emit edge-vs-edge constraints in 3D with the vertex pass's speculative reach.
#[test]
fn edge_leading_bars_collide() {
    for speed in [5.0, 20.0] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        // The edge constraints under test are behind the (default-off) edge speculation.
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .edge_speculation = true;
        let bar = |center: Vector, along_x: bool| {
            let he = if along_x {
                Vector::new(1.5, 0.15, 0.15)
            } else {
                Vector::new(0.15, 0.15, 1.5)
            };
            SoftBodyBuilder::cuboid(
                center,
                he,
                if along_x { 6 } else { 2 },
                2,
                if along_x { 2 } else { 6 },
            )
            .softness(SpringCoefficients::new(20.0, 1.0))
            .particle_mass(0.1)
            .particle_radius(0.02)
        };
        let c = core::f32::consts::FRAC_1_SQRT_2;
        let a = world.insert_soft_body(bar(Vector::new(0.0, 0.0, 0.0), true));
        for i in 0..world.soft_bodies[a].particles().len() {
            let p = world.soft_bodies[a].particle_position(i);
            let (y, z) = (p.y, p.z);
            world.soft_bodies[a]
                .set_particle_position(i, Vector::new(p.x, c * (y - z), c * (y + z)));
        }
        let b = world.insert_soft_body(bar(Vector::new(0.0, 1.2, 0.0), false));
        for i in 0..world.soft_bodies[b].particles().len() {
            let p = world.soft_bodies[b].particle_position(i);
            let (x, y) = (p.x, p.y - 1.2);
            world.soft_bodies[b]
                .set_particle_position(i, Vector::new(c * (x + y), 1.2 + c * (y - x), p.z));
        }
        for i in 0..world.soft_bodies[b].particles().len() {
            world.soft_bodies[b].set_particle_velocity(i, Vector::new(0.0, -speed, 0.0));
        }
        let mut max_cross = 0;
        for _ in 0..90 {
            world.step();
            max_cross = max_cross.max(pair_overlap(&world, a, b).0);
        }
        assert_eq!(max_cross, 0, "speed {speed}: the bars crossed");
    }
}

/// A frictionless rope threaded through a closed soft ball slides out under gravity: the crossing
/// stand-down frees the piercing, so nothing wrong-sided holds the thread.
#[test]
fn rope_through_ball_slides_out() {
    let mut world = PhysicsWorld::new();
    world.insert_soft_body(
        SoftBodyBuilder::sphere(Vector::new(0.0, 0.0, 0.0), 0.4, 1)
            .softness(SpringCoefficients::new(20.0, 1.0))
            .particle_mass(0.1)
            .surface_collider(ColliderBuilder::ball(0.05).friction(0.0))
            .pinned_particles([0, 3, 11]),
    );
    let rope = world.insert_soft_body(
        SoftBodyBuilder::rope(
            Vector::new(0.0, -1.0, 0.02),
            Vector::new(0.0, 1.0, 0.02),
            16,
        )
        .particle_mass(0.02)
        .particle_radius(0.03)
        .surface_collider(ColliderBuilder::ball(0.03).friction(0.0)),
    );
    for _ in 0..400 {
        world.step();
    }
    let com = world.soft_bodies[rope].center_of_mass();
    let inside = (0..16)
        .filter(|&i| world.soft_bodies[rope].particle_position(i).length() < 0.4)
        .count();
    assert_eq!(inside, 0, "the rope still threads the ball");
    assert!(com.y < -1.0, "the rope hangs frozen at {:+.3}", com.y);
}

/// Crossing repulsion clears a static crossing nothing else resolves: an unstrained horizontal
/// strip whose free end pokes through a vertical strip. With the crossing constraints dropped
/// nothing moves; with repulsion the end vertices are pushed back to their neighbors' side.
#[test]
fn crossing_repulsion_clears_static_poke() {
    for repulsion in [true, false] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world
            .integration_parameters
            .soft_bodies
            .recovery
            .crossing_repulsion = repulsion;
        // Horizontal, pinned at its left end, its right end (x = 1.2) 0.05 past the vertical
        // strip's plane (x = 1.15).
        let horizontal = world.insert_soft_body(
            strip(
                Vector::new(-1.0, 0.0, -0.1),
                Vector::new(0.2, 0.0, 0.0),
                Vector::new(0.0, 0.0, 0.2),
                0.02,
            )
            .pinned_particles([0, 1]),
        );
        let vertical = world.insert_soft_body(
            strip(
                Vector::new(1.15, -1.1, -0.1),
                Vector::new(0.0, 0.2, 0.0),
                Vector::new(0.0, 0.0, 0.2),
                0.02,
            )
            .pinned_particles([0, 1, 22, 23]),
        );
        let start = pair_overlap(&world, horizontal, vertical).0;
        assert!(start > 0, "the strips were not authored crossed");
        for _ in 0..120 {
            world.step();
        }
        let end = pair_overlap(&world, horizontal, vertical).0;
        if repulsion {
            assert_eq!(end, 0, "crossing repulsion did not clear the poke");
        } else {
            assert!(
                end > 0,
                "the dropped constraints resolved the poke (the scene proves nothing)"
            );
        }
    }
}

/// Two closed cell-less balloons inserted overlapping de-overlap through the intersection-volume
/// constraints, each retracting its own intruding patch.
#[test]
fn overlapping_balloons_deoverlap() {
    let mut world = PhysicsWorld::new();
    floor(&mut world, 6.0);
    let a = world.insert_soft_body(balloon(Vector::new(-0.35, 0.55, 0.0), 0.5));
    let b = world.insert_soft_body(balloon(Vector::new(0.35, 0.55, 0.0), 0.5));
    for _ in 0..900 {
        world.step();
    }
    assert_eq!(
        pair_overlap(&world, a, b),
        (0, 0, 0),
        "the balloons still overlap"
    );
}
