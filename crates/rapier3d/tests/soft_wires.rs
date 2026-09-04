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
