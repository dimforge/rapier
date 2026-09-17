//! Wire collision meshes: a soft body whose collision geometry is a polyline (a rope in 3D)
//! rather than a surface.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

fn rope(world: &mut PhysicsWorld, from: Vector, to: Vector, n: usize) -> SoftBodyHandle {
    world.insert_soft_body(SoftBodyBuilder::rope(from, to, n).particle_mass(0.05))
}

fn lowest(world: &PhysicsWorld, handle: SoftBodyHandle) -> Real {
    world.soft_bodies[handle]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min)
}

#[test]
fn a_rope_collides_through_a_polyline() {
    let mut world = world_with_ground();
    let handle = rope(
        &mut world,
        Vector::new(-1.0, 2.0, 0.0),
        Vector::new(1.0, 2.0, 0.0),
        12,
    );

    let sb = &world.soft_bodies[handle];
    let mesh = sb.collision_mesh().expect("the rope has a collision mesh");
    assert!(mesh.is_wire(), "the rope's mesh is not a wire");
    assert_eq!(mesh.arity(), 2);
    assert!(!mesh.is_closed(), "a wire encloses nothing");
    assert_eq!(
        world.colliders[mesh.collider()].shape().shape_type(),
        ShapeType::Polyline
    );

    for _ in 0..300 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    assert!(sb.particle_positions().all(|p| p.is_finite()));
    let low = lowest(&world, handle);
    assert!(
        low > -0.05 && low < 0.2,
        "the rope did not come to rest on the ground: {low}"
    );
}

#[test]
fn two_ropes_collide_with_each_other() {
    let mut world = world_with_ground();
    // A rope resting on the ground, and another dropped across it.
    let lower = rope(
        &mut world,
        Vector::new(-1.0, 0.1, 0.0),
        Vector::new(1.0, 0.1, 0.0),
        12,
    );
    let upper = rope(
        &mut world,
        Vector::new(0.0, 1.0, -1.0),
        Vector::new(0.0, 1.0, 1.0),
        12,
    );

    for _ in 0..400 {
        world.step();
    }

    for handle in [lower, upper] {
        assert!(
            world.soft_bodies[handle]
                .particle_positions()
                .all(|p| p.is_finite())
        );
    }
    // The upper rope rests *on* the lower one instead of passing through it: at the crossing,
    // it stays above the lower rope's own height.
    let upper_middle = world.soft_bodies[upper].particle_position(6).y;
    let lower_top = lowest(&world, lower);
    assert!(
        upper_middle > lower_top + 0.05,
        "the ropes passed through each other: {upper_middle} vs {lower_top}"
    );
}

#[test]
fn a_rope_rests_on_a_cloth() {
    let mut world = world_with_ground();
    // A cloth pinned by its corners, with a rope dropped on it.
    let cloth = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            Vector::new(-0.6, 1.0, -0.6),
            Vector::X * 0.3,
            Vector::Z * 0.3,
            5,
            5,
        )
        .pinned_particles([0, 4, 20, 24])
        .particle_mass(0.05)
        .self_contacts(true),
    );
    let rope = rope(
        &mut world,
        Vector::new(-0.4, 1.6, 0.0),
        Vector::new(0.4, 1.6, 0.0),
        10,
    );

    for _ in 0..400 {
        world.step();
    }

    assert!(
        world.soft_bodies[rope]
            .particle_positions()
            .all(|p| p.is_finite())
    );
    let rope_low = lowest(&world, rope);
    let cloth_low = lowest(&world, cloth);
    assert!(
        rope_low > cloth_low - 0.05,
        "the rope fell through the cloth: rope {rope_low}, cloth {cloth_low}"
    );
    assert!(rope_low < 1.6, "the rope never landed: {rope_low}");
}

/// The wire's collider holds the *deformed* geometry, not a frozen copy moved rigidly by
/// the cluster frame: a rope pinned at both ends sags, and its shape sags with it.
#[test]
fn a_wire_collider_follows_the_particles() {
    let mut world = world_with_ground();
    let handle = world.insert_soft_body(
        SoftBodyBuilder::rope(Vector::new(-1.0, 2.0, 0.0), Vector::new(1.0, 2.0, 0.0), 12)
            .pinned_particles([0, 11])
            .particle_mass(0.05),
    );
    let collider = world.soft_bodies[handle]
        .collision_mesh()
        .unwrap()
        .collider();

    for _ in 0..200 {
        world.step();
    }

    let sb = &world.soft_bodies[handle];
    // It actually sagged, or the test would pass on a rigid rope.
    let middle = sb.particle_position(6).y;
    assert!(middle < 1.9, "the rope did not sag: {middle}");

    let co = &world.colliders[collider];
    let polyline = co.shape().as_polyline().expect("a wire is a polyline");
    assert_eq!(polyline.vertices().len(), sb.num_particles());
    for (i, vertex) in polyline.vertices().iter().enumerate() {
        let world_vertex = *co.position() * *vertex;
        let particle = sb.particle_position(i);
        assert!(
            (world_vertex - particle).length() < 1.0e-4,
            "vertex {i} sits at {world_vertex:?} instead of its particle {particle:?}"
        );
    }
}

/// A rope released already hanging over a fixed rod rests on it. A segment across a round rod
/// touches it at one point: with a single contact there the chord rocked about it until an
/// endpoint sank into the rod and was kicked back, and the rope jittered on the rod forever.
#[test]
fn a_rope_draped_over_a_rod_rests() {
    let mut world = world_with_ground();
    let rod_radius = 0.12;
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 4.0, 0.0)),
        ColliderBuilder::capsule_x(5.0, rod_radius),
    );
    // The rope's path: down the -z side, over the rod, down the +z side.
    let n = 40;
    let length = 3.5;
    let segment = length / (n - 1) as Real;
    let radius = segment * 0.5;
    let r = rod_radius + radius + 0.01;
    let arc = core::f32::consts::PI * r;
    let side = (length - arc) * 0.5;
    let positions: Vec<Vector> = (0..n)
        .map(|i| {
            let t = i as Real * segment;
            if t < side {
                Vector::new(0.0, 4.0 - (side - t), -r)
            } else if t < side + arc {
                let a = (t - side) / r;
                Vector::new(0.0, 4.0 + a.sin() * r, -a.cos() * r)
            } else {
                Vector::new(0.0, 4.0 - (t - side - arc), r)
            }
        })
        .collect();
    let edges: Vec<[u32; 2]> = (0..n as u32 - 1).map(|i| [i, i + 1]).collect();
    let bend_edges: Vec<[u32; 2]> = (0..n as u32 - 2).map(|i| [i, i + 2]).collect();
    let handle = world.insert_soft_body(
        SoftBodyBuilder::new(positions)
            .particle_radius(radius)
            .wire(edges.clone())
            .edges(edges)
            .bend_edges(bend_edges)
            .softness(SpringCoefficients::new(30.0, 1.0))
            .particle_mass(0.03)
            .surface_collider(ColliderBuilder::ball(radius).friction(0.6)),
    );
    let mut max_speed: Real = 0.0;
    let mut on_rod = 0;
    for step in 0..400 {
        world.step();
        if step < 200 {
            continue;
        }
        for p in world.soft_bodies[handle].particles() {
            if p.position().y > 3.9 {
                on_rod += 1;
                max_speed = max_speed.max(p.velocity().length());
            }
        }
    }
    assert!(on_rod > 0, "the rope slid off the rod");
    let low = lowest(&world, handle);
    assert!(low > 2.0 && low < 3.0, "the rope is not hanging from the rod: {low}");
    assert!(
        max_speed < 0.02,
        "the rope jitters on the rod: max speed {max_speed}"
    );
}

