//! Collision meshes owned by a soft body's clusters: binding a mesh to a cluster
//! (`ColliderSet::insert_deformable`), what is rejected, and how several meshes of one body
//! interact.

use rapier3d::parry::shape::{Ball, Cuboid, TriMeshFlags};
use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

/// A soft cube of 3x3x3 particles, above the ground.
fn cube(world: &mut PhysicsWorld) -> SoftBodyHandle {
    world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 1.5, 0.0), Vector::splat(0.5), 3, 3, 3)
            .particle_mass(0.2),
    )
}

/// A deformable mesh authored in world space, to be attached to `parent`: placed by the
/// inverse of the parent's pose.
fn world_mesh(
    world: &PhysicsWorld,
    vertices: Vec<Vector>,
    indices: Vec<[u32; 3]>,
    parent: RigidBodyHandle,
) -> ColliderBuilder {
    ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)
        .expect("a valid mesh")
        .position(world.bodies[parent].position().inverse())
}

/// The body's own boundary, in world space: a mesh that sits exactly on its particles, to be
/// attached to `parent`.
fn boundary_mesh(
    world: &PhysicsWorld,
    handle: SoftBodyHandle,
    parent: RigidBodyHandle,
) -> ColliderBuilder {
    let sb = &world.soft_bodies[handle];
    let vertices: Vec<Vector> = sb.particle_positions().collect();
    let indices: Vec<[u32; 3]> = sb.boundary().to_vec();
    world_mesh(world, vertices, indices, parent)
}

#[test]
fn a_second_mesh_on_the_whole_body_cluster_follows_the_particles() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    let proxy = world.soft_bodies[handle].root_body();
    let collider = world
        .insert_deformable(
            boundary_mesh(&world, handle, proxy),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .expect("the mesh binds to the whole-body cluster");

    assert_eq!(world.soft_bodies[handle].meshes().count(), 2);
    let before = world.soft_bodies[handle]
        .mesh_of(collider)
        .unwrap()
        .vertex_positions(&world.soft_bodies[handle])
        .map(|v| v.y)
        .fold(Real::MAX, Real::min);

    for _ in 0..120 {
        world.step();
    }

    let sb = &world.soft_bodies[handle];
    let mesh = sb.mesh_of(collider).expect("the mesh is still there");
    let after = mesh
        .vertex_positions(sb)
        .map(|v| v.y)
        .fold(Real::MAX, Real::min);
    assert!(
        mesh.vertex_positions(sb).all(|v| v.is_finite()),
        "the mesh went non-finite"
    );
    assert!(
        after < before - 0.1,
        "the mesh did not come down with the particles: {before} -> {after}"
    );
    // It rests on the ground with its particles rather than sinking.
    assert!(after > -0.2, "the mesh sank through the ground: {after}");
}

#[test]
fn two_meshes_of_one_cluster_never_collide_with_each_other() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    let proxy = world.soft_bodies[handle].root_body();
    let first = world.soft_bodies[handle]
        .collision_mesh()
        .unwrap()
        .collider();
    let second = world
        .insert_deformable(
            boundary_mesh(&world, handle, proxy),
            // Even asking for self contacts: two meshes of one cluster describe the same
            // particles, so they must not fight each other.
            SoftMeshBinding::direct_by_position(1.0e-4).self_contacts(true),
            proxy,
        )
        .unwrap();

    for _ in 0..60 {
        world.step();
    }

    assert!(
        world.narrow_phase.contact_pair(first, second).is_none(),
        "two meshes of one cluster reported a contact pair"
    );
    // Both still meet the ground.
    for collider in [first, second] {
        assert!(
            world
                .narrow_phase
                .contact_pairs_with(collider)
                .any(|p| p.has_any_active_contact()),
            "a mesh lost its contacts with the world"
        );
    }
}

#[test]
fn a_mesh_bound_outside_its_cluster_is_rejected() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    // A cluster covering a single particle: every other particle is outside it.
    let cluster = world
        .soft_bodies
        .add_cluster(handle, &[0], &mut world.bodies, &mut world.colliders)
        .expect("the cluster is created");
    let proxy = world.soft_bodies[handle].cluster_proxy(cluster).unwrap();

    // Particle 1 is not in the cluster, so no vertex may ride it.
    let err = world
        .insert_deformable(
            boundary_mesh(&world, handle, proxy),
            SoftMeshBinding::direct(vec![1; world.soft_bodies[handle].num_particles()]),
            proxy,
        )
        .unwrap_err();
    assert_eq!(
        err,
        SoftBindingError::VertexOutsideCluster {
            vertex: 0,
            particle: 1
        },
        "{err}"
    );

    // A binding by position finds no particle of that cluster near most vertices.
    let err = world
        .insert_deformable(
            boundary_mesh(&world, handle, proxy),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .unwrap_err();
    assert!(
        matches!(err, SoftBindingError::UnmatchedVertex { .. }),
        "{err}"
    );
    assert_eq!(world.soft_bodies[handle].cluster(cluster).unwrap().meshes().count(), 0);
}

#[test]
fn a_bad_parent_or_shape_is_rejected() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    let (rigid, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(5.0, 5.0, 5.0)),
        ColliderBuilder::ball(0.2),
    );

    // Not a cluster proxy.
    let err = world
        .insert_deformable(
            boundary_mesh(&world, handle, rigid),
            SoftMeshBinding::direct_by_position(1.0e-4),
            rigid,
        )
        .unwrap_err();
    assert_eq!(err, SoftBindingError::NotAClusterProxy);

    // Not a mesh.
    let proxy = world.soft_bodies[handle].root_body();
    let err = world
        .insert_deformable(
            ColliderBuilder::ball(0.5),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .unwrap_err();
    assert_eq!(err, SoftBindingError::UnsupportedShape);

    // A mesh without the deformable flag: its vertices could not be moved in place.
    let sb = &world.soft_bodies[handle];
    let err = world
        .insert_deformable(
            ColliderBuilder::trimesh(sb.particle_positions().collect(), sb.boundary().to_vec())
                .unwrap()
                .position(world.bodies[proxy].position().inverse()),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .unwrap_err();
    assert_eq!(err, SoftBindingError::NotDeformable);

    // Both sets are untouched by a rejected insertion.
    assert_eq!(world.soft_bodies[handle].meshes().count(), 1);
    assert_eq!(world.bodies[proxy].colliders().len(), 1);
}

/// The collider is attached like any other: its shape object, flags and pose relative to the
/// proxy are kept, and its vertices are rewritten in its own frame.
#[test]
fn a_deformable_collider_keeps_its_shape_and_pose() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    let proxy = world.soft_bodies[handle].root_body();
    let sb = &world.soft_bodies[handle];
    let vertices: Vec<Vector> = sb.particle_positions().collect();
    let indices: Vec<[u32; 3]> = sb.boundary().to_vec();
    let flags = TriMeshFlags::DEFORMABLE | TriMeshFlags::ORIENTED;
    let given = world.bodies[proxy].position().inverse();
    let collider = world
        .insert_deformable(
            ColliderBuilder::trimesh_with_flags(vertices, indices, flags)
                .unwrap()
                .position(given),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .unwrap();

    let check = |world: &PhysicsWorld| {
        let co = &world.colliders[collider];
        let wrt_parent = co.position_wrt_parent().unwrap();
        assert!(
            (wrt_parent.translation - given.translation).length() < 1.0e-6,
            "the pose relative to the proxy was changed: {wrt_parent:?} vs {given:?}"
        );
        let trimesh = co.shape().as_trimesh().expect("still a trimesh");
        assert_eq!(trimesh.flags(), flags, "the shape's flags were changed");
        // The shape, through the collider's pose, is where the mesh's vertices are.
        let sb = &world.soft_bodies[handle];
        let mesh = sb.mesh_of(collider).unwrap();
        for (local, expected) in trimesh.vertices().iter().zip(mesh.vertex_positions(sb)) {
            let world_vertex = *co.position() * *local;
            assert!(
                (world_vertex - expected).length() < 1.0e-4,
                "the shape drifted from its mesh: {world_vertex:?} vs {expected:?}"
            );
        }
    };
    check(&world);
    for _ in 0..60 {
        world.step();
    }
    check(&world);
}

#[test]
fn removing_a_deformable_collider_removes_its_mesh() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    let proxy = world.soft_bodies[handle].root_body();
    let collider = world
        .insert_deformable(
            boundary_mesh(&world, handle, proxy),
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .unwrap();
    assert_eq!(world.soft_bodies[handle].meshes().count(), 2);

    world.remove_collider(collider);
    assert_eq!(world.soft_bodies[handle].meshes().count(), 1);
    assert!(world.soft_bodies[handle].mesh_of(collider).is_none());

    for _ in 0..30 {
        world.step();
    }
    // The body keeps simulating through the mesh it has left.
    assert!(
        world.soft_bodies[handle]
            .particle_positions()
            .all(|p| p.is_finite())
    );
}

#[test]
fn a_mesh_on_a_sub_cluster_binds_to_its_own_particles() {
    let mut world = world_with_ground();
    let handle = cube(&mut world);
    // Three particles of the cube's lower face.
    let lower: Vec<u32> = world.soft_bodies[handle]
        .particles()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.position().y < 1.4)
        .map(|(i, _)| i as u32)
        .take(3)
        .collect();
    assert_eq!(lower.len(), 3);
    let cluster = world
        .soft_bodies
        .add_cluster(handle, &lower, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[handle].cluster_proxy(cluster).unwrap();

    // A quad over those four particles.
    let sb = &world.soft_bodies[handle];
    let vertices: Vec<Vector> = lower.iter().map(|&i| sb.particle_position(i as usize)).collect();
    let patch = world_mesh(&world, vertices, vec![[0, 1, 2]], proxy);
    let collider = world
        .insert_deformable(
            patch,
            SoftMeshBinding::direct_by_position(1.0e-4),
            proxy,
        )
        .expect("the patch binds to the lower-face cluster");

    let mesh = world.soft_bodies[handle].mesh_of(collider).unwrap();
    assert_eq!(mesh.id().cluster, cluster);
    assert_eq!(mesh.vertex_count(), 3);

    for _ in 0..120 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    let mesh = sb.mesh_of(collider).unwrap();
    assert!(mesh.vertex_positions(sb).all(|v| v.is_finite()));
    // The patch stayed on the particles it was bound to.
    for (vertex, &particle) in mesh.vertex_positions(sb).zip(&lower) {
        let expected = sb.particle_position(particle as usize);
        assert!(
            (vertex - expected).length() < 1.0e-4,
            "the patch left its particle: {vertex:?} vs {expected:?}"
        );
    }
}

/// A tear rewires the cells to the split particles; a skinned mesh must follow its cells instead
/// of freezing where it was (its bindings used to be left untouched, so every vertex went
/// dangling).
#[test]
fn a_skinned_mesh_follows_its_cells_through_a_tear() {
    let (vertices, indices) = Ball::new(1.0).to_trimesh(16, 16);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(0.0, 3.0, 0.0))
        .collect();
    let mut world = world_with_ground();
    let handle = world.insert_soft_body(
        SoftBodyBuilder::volumetric_skinned(&vertices, &indices, 0.4)
            .expect("the ball is filled")
            .cell_model(SoftBodyCellModel::Corotational)
            .skin_collision(true)
            .particle_mass(0.05),
    );

    // Where every skin vertex sits just before the tear.
    world.step();
    let before: Vec<Vector> = {
        let sb = &world.soft_bodies[handle];
        sb.collision_mesh().unwrap().vertex_positions(sb).collect()
    };

    // Tear a cell of the ball: a crack opens through it, under the skin.
    let cells_before = world.soft_bodies[handle].cells().len();
    world.soft_bodies[handle].tear_cell(0);
    world.step();
    assert!(
        world.soft_bodies[handle].cells().len() < cells_before,
        "nothing was torn"
    );

    let sb = &world.soft_bodies[handle];
    let mesh = sb.collision_mesh().expect("the skin is still the collision mesh");
    // Every vertex still rides a live cell.
    match mesh.binding() {
        SoftMeshMapping::Skinned { bindings } => {
            for (vertex, binding) in bindings.iter().enumerate() {
                assert!(
                    (binding.cell as usize) < sb.cells().len(),
                    "skin vertex {vertex} lost its cell"
                );
            }
        }
        other => panic!("the collision mesh is not skinned: {other:?}"),
    }
    assert!(mesh.vertex_positions(sb).all(|v| v.is_finite()));
    // A vertex left on a renumbered cell jumps across the body; every one of them must still be
    // where it was, one step later.
    assert_eq!(mesh.vertex_count(), before.len());
    let moved = mesh
        .vertex_positions(sb)
        .zip(&before)
        .map(|(now, then)| (now - *then).length())
        .fold(0.0, Real::max);
    assert!(moved < 0.1, "a skin vertex jumped {moved} through the tear");
}

/// `insert_deformable` takes the mesh in world space: a skinned mesh must stay where it was
/// given, not be displaced by the body's rest frame.
#[test]
fn a_skinned_mesh_binds_where_it_is_given() {
    let mut world = world_with_ground();
    // At rest and far from the origin: binding against the rest positions (which are relative
    // to the rest center of mass) lands the mesh a whole `center` away as soon as it is
    // updated from the particles.
    world.gravity = Vector::ZERO;
    let center = Vector::new(-3.0, 1.5, 2.0);
    let handle = world.insert_soft_body(
        SoftBodyBuilder::cuboid(center, Vector::splat(0.5), 3, 3, 3).particle_mass(0.2),
    );
    let (vertices, indices) = Cuboid::new(Vector::splat(0.7)).to_trimesh();
    let given: Vec<Vector> = vertices.iter().map(|v| *v + center).collect();
    let proxy = world.soft_bodies[handle].root_body();
    let collider = world
        .insert_deformable(
            world_mesh(&world, given.clone(), indices, proxy),
            SoftMeshBinding::skinned(),
            proxy,
        )
        .expect("the mesh binds to the whole-body cluster");

    // The cached vertices are the input until the first update: step first.
    for _ in 0..5 {
        world.step();
    }

    let sb = &world.soft_bodies[handle];
    let mesh = sb.mesh_of(collider).unwrap();
    for (bound, given) in mesh.vertex_positions(sb).zip(&given) {
        assert!(
            (bound - *given).length() < 1.0e-4,
            "the mesh was bound at {bound:?} instead of {given:?}"
        );
    }
}
