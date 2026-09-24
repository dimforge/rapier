//! Soft bodies: soft disks and a soft square falling on the ground, and a rope hanging from a
//! fixed point and holding a rigid ball.
//!
//! The square is rendered with a mesh kept in sync with its particles; the other soft bodies
//! are shown by the debug-renderer.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands, mut debug_render: ResMut<DebugRenderContext>) {
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 150.0, 0.0)));
    // Draw the elements of the soft bodies.
    debug_render.mode.soft_bodies = true;
}

pub fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -20.0, 0.0),
        Collider::cuboid(500.0, 20.0),
    ));

    /*
     * Soft disks with an inflated area, colliding with their own boundary.
     */
    for i in 0..3 {
        let radius = 30.0 + 10.0 * i as f32;
        commands.spawn((
            Transform::from_xyz(-250.0 + 90.0 * i as f32, 150.0 + 80.0 * i as f32, 0.0),
            SoftBody::disk(radius, 24).map(|b| b.particle_mass(0.05).self_contacts(true)),
            SoftBodyMaterial::uniform(20.0, 1.0),
            SoftBodyVolumeFactor(1.1),
        ));
    }

    /*
     * A jelly square, rendered with a synchronized mesh.
     */
    commands.spawn((
        Transform::from_xyz(50.0, 200.0, 0.0).with_rotation(Quat::from_rotation_z(0.3)),
        SoftBody::grid(Vec2::splat(50.0), 6, 6).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .particle_mass(0.2)
        }),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            young_modulus: 3.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        }),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(materials.add(Color::srgb(0.2, 0.6, 0.3))),
    ));

    /*
     * A rope pinned at its first particle, its last particle attached to a rigid ball.
     */
    let ball = commands
        .spawn((
            Transform::from_xyz(400.0, 300.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(20.0),
            ColliderMassProperties::Density(0.002),
            Mesh2d(meshes.add(Circle::new(20.0))),
            MeshMaterial2d(materials.add(Color::srgb(0.2, 0.4, 0.8))),
        ))
        .id();
    commands.spawn((
        Transform::from_xyz(200.0, 300.0, 0.0),
        SoftBody::rope(Vec2::ZERO, Vec2::new(180.0, 0.0), 20)
            .map(|b| b.pinned_particles([0]).particle_mass(0.05)),
        SoftBodyAttachments(vec![SoftBodyAttachment {
            particle: 19,
            body: ball,
        }]),
    ));
}
