//! Cloth: a sheet draped over a rigid ball and a rigid box, and a curtain pinned along its top
//! edge pushed by a rolling ball. The cloths are rendered with meshes kept in sync with their
//! particles.

use bevy::prelude::*;
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
        Transform::from_xyz(8.0, 6.0, 14.0).looking_at(Vec3::new(0.0, 1.5, 2.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// A double-sided material, for cloths.
fn cloth_material(color: Color) -> StandardMaterial {
    StandardMaterial {
        base_color: color,
        double_sided: true,
        cull_mode: None,
        ..default()
    }
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let obstacle_color = materials.add(Color::srgb(0.4, 0.4, 0.45));

    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(12.0, 0.1, 12.0),
    ));

    /*
     * A sheet dropped over a rigid ball and a rigid box. Particle `(i, j)` of a cloth is at
     * `origin + i * du + j * dv`.
     */
    commands.spawn((
        Transform::from_xyz(-1.0, 1.0, 0.0),
        Collider::ball(1.0),
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(obstacle_color.clone()),
    ));
    commands.spawn((
        Transform::from_xyz(1.5, 0.6, 0.0),
        Collider::cuboid(0.6, 0.6, 0.6),
        Mesh3d(meshes.add(Cuboid::new(1.2, 1.2, 1.2))),
        MeshMaterial3d(obstacle_color.clone()),
    ));
    commands.spawn((
        Transform::from_xyz(-3.0, 3.0, -2.0),
        SoftBody::cloth(Vec3::ZERO, Vec3::X * 0.1, Vec3::Z * 0.1, 60, 40)
            .map(|b| b.particle_mass(0.02)),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            bend_softness: SpringCoefficients::new(3.0, 1.0),
            ..RapierSoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
        }),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(cloth_material(Color::srgb(0.8, 0.2, 0.2)))),
    ));

    /*
     * A curtain pinned along its top edge (the particles `(i, 0)`, of index `i * 30`), with a
     * heavy ball rolling into it.
     */
    commands.spawn((
        Transform::from_xyz(-2.0, 4.0, 5.0),
        SoftBody::cloth(Vec3::ZERO, Vec3::X * 0.1, Vec3::NEG_Y * 0.1, 40, 30).map(|b| {
            b.pinned_particles((0..40).map(|i| i * 30))
                .particle_mass(0.02)
        }),
        SoftBodyMaterial::uniform(30.0, 1.0),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(cloth_material(Color::srgb(0.2, 0.5, 0.8)))),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 0.5, 9.0),
        RigidBody::Dynamic,
        Velocity::linear(Vec3::new(0.0, 0.0, -6.0)),
        Collider::ball(0.5),
        ColliderMassProperties::Density(3.0),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(obstacle_color),
    ));
}
