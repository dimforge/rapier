//! Collision meshes owned by clusters: a jelly wearing a fine skinned sensor over its coarse
//! cells, a jelly whose top cluster carries a rigid plate, and a jelly split into two clusters
//! whose own deformable meshes collide with each other.

use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::parry::shape::Ball;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-6.0, 4.0, 8.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// A corotational jelly cube of 4x4x4 particles.
fn jelly(young_modulus: f32) -> (SoftBody, SoftBodyMaterial, Friction) {
    (
        SoftBody::cuboid(Vec3::splat(0.6), 4, 4, 4).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .particle_mass(0.1)
                .particle_radius(0.05)
        }),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            young_modulus,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..default()
        }),
        Friction::coefficient(0.7),
    )
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let jelly_color = materials.add(Color::srgb(0.55, 0.75, 0.95));
    let rigid_color = materials.add(Color::srgb(0.4, 0.4, 0.45));

    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(12.0, 0.1, 12.0),
    ));

    /*
     * A jelly wearing a second, skinned ball-shaped sensor, bound to the cells it lies in.
     */
    let center = Vec3::new(-3.0, 1.2, 0.0);
    let skinned = commands
        .spawn((
            Transform::from_translation(center),
            jelly(2.0e2),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(jelly_color.clone()),
        ))
        .id();
    // The vertices of a deformable collider are placed by its entity's transform.
    let (vertices, indices) = Ball::new(1.2).to_trimesh(20, 20);
    commands.spawn((
        Transform::from_translation(center),
        Collider::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)
            .expect("a valid ball mesh"),
        Sensor,
        DeformableCollider::new(skinned, SoftMeshBinding::skinned()),
    ));

    /*
     * A jelly whose top cluster carries a rigid plate: the load on the plate reaches the
     * particles through the cluster's frame.
     */
    let center = Vec3::new(0.0, 1.2, 0.0);
    let (plated, material, friction) = jelly(4.0e2);
    let top: Vec<u32> = (0..)
        .zip(plated.builder.particle_positions())
        .filter(|(_, p)| p.y > 0.3)
        .map(|(i, _)| i)
        .collect();
    let plated = commands
        .spawn((
            Transform::from_translation(center),
            plated,
            material,
            friction,
            SoftBodyMeshSync::default(),
            MeshMaterial3d(jelly_color.clone()),
        ))
        .id();
    // A child of a cluster entity is attached to the cluster's proxy like to any rigid-body.
    commands
        .spawn(SoftBodyCluster::new(plated, top))
        .with_child((
            Transform::from_xyz(0.0, 0.65, 0.0),
            Collider::cuboid(0.7, 0.05, 0.7),
            Mesh3d(meshes.add(Cuboid::new(1.4, 0.1, 1.4))),
            MeshMaterial3d(rigid_color.clone()),
        ));

    /*
     * A jelly split into two clusters, each with a mesh of its own: they collide with each other
     * (both opted into self contacts) because their clusters share no particle.
     */
    let center = Vec3::new(3.0, 1.2, 0.0);
    let (split, material, friction) = jelly(4.0e2);
    let split = split.map(|b| b.no_surface_collider());
    let (left, right): (Vec<u32>, Vec<u32>) = (0..split.builder.particle_positions().len() as u32)
        .partition(|i| split.builder.particle_positions()[*i as usize].x < 0.0);
    let halves = [left, right].map(|half| {
        let (vertices, indices) = cluster_surface(&split, &half);
        (half, vertices, indices)
    });
    let split = commands
        .spawn((
            Transform::from_translation(center),
            split,
            material,
            friction,
            SoftBodyMeshSync::default(),
            MeshMaterial3d(jelly_color),
        ))
        .id();
    for (half, vertices, indices) in halves {
        let cluster = commands
            .spawn((SoftBodyCluster::new(split, half.iter().copied()),))
            .id();
        commands.spawn((
            Transform::from_translation(center),
            Collider::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)
                .expect("a valid half surface"),
            Friction::coefficient(0.7),
            // Both halves opt in: their clusters share no particle, so their meshes may meet.
            DeformableCollider::new(cluster, SoftMeshBinding::direct(half).self_contacts(true)),
        ));
    }

    /*
     * Something to drop on all three.
     */
    let box_mesh = meshes.add(Cuboid::new(0.6, 0.6, 0.6));
    for (i, x) in [-3.0, 0.0, 3.0].into_iter().enumerate() {
        commands.spawn((
            Transform::from_xyz(x, 4.0 + i as f32, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(0.3, 0.3, 0.3),
            ColliderMassProperties::Density(4.0),
            Mesh3d(box_mesh.clone()),
            MeshMaterial3d(rigid_color.clone()),
        ));
    }
}

/// The closed surface of the cells a set of particles owns, in the soft body's local frame: every
/// face no second owned cell shares, oriented outward. The vertex `i` is the particle
/// `particles[i]`, so a half's surface caps the cut with particles from inside the body.
fn cluster_surface(body: &SoftBody, particles: &[u32]) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let positions = body.builder.particle_positions();
    let mut vertex_of = vec![u32::MAX; positions.len()];
    for (vertex, particle) in particles.iter().enumerate() {
        vertex_of[*particle as usize] = vertex as u32;
    }
    let vertices = particles.iter().map(|i| positions[*i as usize]).collect();

    // Every face of every owned cell, keyed by its sorted vertices: a face shared by two of them
    // is interior to the half.
    let mut faces: HashMap<[u32; 3], ([u32; 3], Vec3, usize)> = HashMap::new();
    for cell in &body.builder.cells {
        if cell.iter().any(|v| vertex_of[*v as usize] == u32::MAX) {
            continue;
        }
        let centroid = cell.iter().map(|v| positions[*v as usize]).sum::<Vec3>() / 4.0;
        for k in 0..4 {
            let face = [cell[(k + 1) % 4], cell[(k + 2) % 4], cell[(k + 3) % 4]];
            let mut key = face;
            key.sort_unstable();
            faces.entry(key).or_insert((face, centroid, 0)).2 += 1;
        }
    }

    let indices = faces
        .values()
        .filter(|(_, _, count)| *count == 1)
        .map(|(face, centroid, _)| {
            let p = face.map(|v| positions[v as usize]);
            // Outward winding: the face's normal points away from its cell.
            let normal = (p[1] - p[0]).cross(p[2] - p[0]);
            let face = if normal.dot((p[0] + p[1] + p[2]) / 3.0 - *centroid) < 0.0 {
                [face[0], face[2], face[1]]
            } else {
                *face
            };
            face.map(|v| vertex_of[v as usize])
        })
        .collect();

    (vertices, indices)
}
