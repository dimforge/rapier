//! Headless probes of 3D self-intersection recovery: surfaces that start in a self-crossed state
//! must recover instead of being held tangled by their own self-contact constraints (mechanism in
//! the 2D `self_intersect_probe`). Env toggle `NO_SELF_CONTACTS` runs the baseline without them.

use rapier3d::prelude::*;

const SELF_CONTACTS: bool = option_env!("NO_SELF_CONTACTS").is_none();

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

/// The number of edge-vs-triangle self-crossings of the body's surface, and its inverted cells.
fn probe(world: &PhysicsWorld, h: SoftBodyHandle) -> (usize, usize) {
    let sb = &world.soft_bodies[h];
    let mesh = sb.meshes().next().unwrap();
    let indices = mesh.indices();
    let p = |v: u32| mesh.vertex(sb, v as usize);
    let mut crossings = 0;
    for i in 0..indices.len() {
        for j in 0..indices.len() {
            let (ei, ej) = (indices[i], indices[j]);
            if i == j || ei.iter().any(|v| ej.contains(v)) {
                continue;
            }
            let tri = [p(ej[0]), p(ej[1]), p(ej[2])];
            for k in 0..3 {
                let (a, b) = (ei[k], ei[(k + 1) % 3]);
                if a < b && segment_crosses_triangle(p(a), p(b), &tri) {
                    crossings += 1;
                }
            }
        }
    }
    let mut inverted = 0;
    for cell in sb.cells() {
        let x: [Vector; 4] =
            core::array::from_fn(|k| sb.particle_position(cell.vertices[k] as usize));
        let vol = (x[1] - x[0]).cross(x[2] - x[0]).dot(x[3] - x[0]);
        if vol * cell.rest_volume <= 0.0 {
            inverted += 1;
        }
    }
    (crossings, inverted)
}

/// Rough step-time probe: a resting self-colliding cloth (no tangles), to price the per-step
/// edge-vs-triangle crossing sweep.
#[test]
#[ignore]
fn cloth_timing() {
    let mut world = PhysicsWorld::new();
    let n = 20;
    let edge = |i: usize, j: usize| i == 0 || j == 0 || i == n - 1 || j == n - 1;
    let pinned = (0..n * n)
        .filter(|&id| edge(id / n, id % n))
        .map(|id| id as u32);
    let extent = 0.12 * (n - 1) as Real / 2.0;
    world.insert_soft_body(
        SoftBodyBuilder::cloth(
            Vector::new(-extent, 2.0, -extent),
            Vector::new(0.12, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.12),
            n,
            n,
        )
        .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(0.05)
        .pinned_particles(pinned)
        .self_contacts(true)
    );
    for _ in 0..120 {
        world.step();
    }
    let start = std::time::Instant::now();
    for _ in 0..1000 {
        world.step();
    }
    println!(
        "cloth: {:.3} ms/step",
        start.elapsed().as_secs_f64() * 1000.0 / 1000.0
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
    let h = world.insert_soft_body(builder.pinned_particles(pinned).self_contacts(SELF_CONTACTS));
    world.soft_bodies[h].set_particle_position(captured, Vector::new(0.3, -0.4, 0.0));

    for step in 0..=600 {
        if step % 60 == 0 {
            let (crossings, inverted) = probe(&world, h);
            let p = world.soft_bodies[h].particle_position(captured);
            println!(
                "step {step:4}: crossings={crossings} inverted_cells={inverted} \
                 captured=({:+.3},{:+.3},{:+.3})",
                p.x, p.y, p.z
            );
        }
        world.step();
    }
    let (crossings, inverted) = probe(&world, h);
    println!("final: crossings={crossings} inverted_cells={inverted}");
    assert_eq!(crossings, 0, "the slab's boundary is still self-crossed");
    assert_eq!(inverted, 0, "the slab still has inverted cells");
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
    .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
    .softness(SpringCoefficients::new(40.0, 1.0))
    .particle_mass(0.02)
    .particle_radius(0.05)
    // Both ends of the strip pinned: the outer edge of the bottom layer at its rest place,
    // the free edge of the folded-back top layer above it (positions set below).
    .pinned_particles((0..nv as u32).chain((nu as u32 - 1) * nv as u32..(nu * nv) as u32))
    .self_contacts(SELF_CONTACTS);
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

    for step in 0..=600 {
        if step % 60 == 0 {
            let (crossings, _) = probe(&world, h);
            let p = world.soft_bodies[h].particle_position(captured);
            println!(
                "step {step:4}: crossings={crossings} captured=({:+.3},{:+.3},{:+.3})",
                p.x, p.y, p.z
            );
        }
        world.step();
    }
    let (crossings, _) = probe(&world, h);
    println!("final: crossings={crossings}");
    assert_eq!(crossings, 0, "the cloth is still self-crossed");
}

/// The number of edge-vs-triangle crossings between two bodies' surfaces (both directions).
fn cross_crossings(world: &PhysicsWorld, a: SoftBodyHandle, b: SoftBodyHandle) -> usize {
    let (sa, sb) = (&world.soft_bodies[a], &world.soft_bodies[b]);
    let (ma, mb) = (sa.meshes().next().unwrap(), sb.meshes().next().unwrap());
    let pa = |v: u32| ma.vertex(sa, v as usize);
    let pb = |v: u32| mb.vertex(sb, v as usize);
    let mut crossings = 0;
    let mut count = |ea: &[u32; 3], tri: &[Vector; 3], p: &dyn Fn(u32) -> Vector| {
        for k in 0..3 {
            let (u, v) = (ea[k], ea[(k + 1) % 3]);
            if u < v && segment_crosses_triangle(p(u), p(v), tri) {
                crossings += 1;
            }
        }
    };
    for ea in ma.indices() {
        for eb in mb.indices() {
            let tb = [pb(eb[0]), pb(eb[1]), pb(eb[2])];
            count(ea, &tb, &pa);
            let ta = [pa(ea[0]), pa(ea[1]), pa(ea[2])];
            count(eb, &ta, &pb);
        }
    }
    crossings
}

/// A cloth strip threaded through a pinned strip slides free under gravity instead of
/// hanging frozen on it: edge constraints touching the mutual crossing stand down (thin open
/// bodies meet through their edge constraints, so the vertex-pass gate alone cannot free them).
#[test]
fn ribbon_through_ribbon_slides_free() {
    let mut world = PhysicsWorld::new();
    let strip = |origin: Vector, du: Vector, dv: Vector| {
        SoftBodyBuilder::cloth(origin, du, dv, 12, 2)
            .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
            .softness(SpringCoefficients::new(40.0, 1.0))
            .particle_mass(0.02)
            .particle_radius(0.04)
    };
    let horizontal = world.insert_soft_body(
        strip(
            Vector::new(-1.1, 0.0, -0.1),
            Vector::new(0.2, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.2),
        )
        .pinned_particles([0, 1, 22, 23]),
    );
    // Vertical, in the xz-plane of the pinned strip's middle, crossing its surface.
    let vertical = world.insert_soft_body(strip(
        Vector::new(0.03, -1.1, -0.1),
        Vector::new(0.0, 0.2, 0.0),
        Vector::new(0.0, 0.0, 0.2),
    ));
    let crossings = |world: &PhysicsWorld| cross_crossings(world, horizontal, vertical);
    assert!(crossings(&world) > 0, "the strips were authored crossed");
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[vertical].center_of_mass();
    println!("crossings {} com_y {:+.3}", crossings(&world), com.y);
    assert_eq!(crossings(&world), 0, "the strips are still crossed");
    assert!(com.y < -1.5, "the vertical strip hangs frozen at {:+.3}", com.y);
}

/// Two closed slim bars crossing edge-first collide instead of passing through: closed-vs-closed
/// pairs emit edge-vs-edge constraints in 3D with the vertex pass's speculative reach.
#[test]
fn edge_leading_bars_collide() {
    for speed in [5.0, 20.0] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        // The edge constraints under test are behind the (default-off) edge speculation.
        world.integration_parameters.soft_bodies.recovery.edge_speculation = true;
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
            world.soft_bodies[b].set_particle_position(
                i,
                Vector::new(c * (x + y), 1.2 + c * (y - x), p.z),
            );
        }
        for i in 0..world.soft_bodies[b].particles().len() {
            world.soft_bodies[b].set_particle_velocity(i, Vector::new(0.0, -speed, 0.0));
        }
        let mut max_cross = 0;
        for _ in 0..90 {
            world.step();
            max_cross = max_cross.max(cross_crossings(&world, a, b));
        }
        let com = world.soft_bodies[b].center_of_mass();
        println!("speed {speed:5}: max_cross {max_cross} b_com_y {:+.3}", com.y);
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
        SoftBodyBuilder::rope(Vector::new(0.0, -1.0, 0.02), Vector::new(0.0, 1.0, 0.02), 16)
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
    println!("rope com_y {:+.3}, particles inside ball: {inside}", com.y);
    assert_eq!(inside, 0, "the rope still threads the ball");
    assert!(com.y < -1.0, "the rope hangs frozen at {:+.3}", com.y);
}

