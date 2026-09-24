//! Soft bodies: a cloth pinned at two corners draping over a box, and a soft cube and a soft
//! ball falling on the ground, rendered with meshes kept in sync with their particles.
//!
//! Press space to cut the cloth in two.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Marks the cloth entity.
#[derive(Component)]
pub struct Cloth;

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
        .add_systems(Update, (cut_cloth, log_tears))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-4.0, 5.0, 9.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    /*
     * Ground and a box for the cloth to drape over.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(10.0, 0.1, 10.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 0.75, 0.0),
        Collider::cuboid(0.75, 0.75, 0.75),
        Mesh3d(meshes.add(Cuboid::new(1.5, 1.5, 1.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.4, 0.4, 0.45))),
    ));

    /*
     * A cloth, pinned at the two corners of its back edge. Particle `(i, j)` has the index
     * `i * n + j`, at `origin + i * du + j * dv`.
     */
    let n = 30;
    let spacing = 0.1;
    let origin = Vec3::new(-1.5, 0.0, -1.5);
    let pinned = [0, (n - 1) * n];
    commands.spawn((
        Cloth,
        Transform::from_xyz(0.0, 2.2, 0.0),
        SoftBody::cloth(
            origin,
            Vec3::X * spacing,
            Vec3::Z * spacing,
            n as usize,
            n as usize,
        )
        .map(|b| {
            b.particle_mass(0.02)
                .material(RapierSoftBodyMaterial {
                    bend_softness: SpringCoefficients::new(3.0, 1.0),
                    ..RapierSoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
                })
                .pinned_particles(pinned)
        }),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
    ));

    /*
     * A soft cube and a soft ball, falling next to the box.
     */
    commands.spawn((
        Transform::from_xyz(2.5, 3.0, 0.0).with_rotation(Quat::from_rotation_z(0.4)),
        SoftBody::cuboid(Vec3::splat(0.5), 5, 5, 5).map(|b| b.particle_mass(0.05)),
        SoftBodyMaterial::uniform(20.0, 1.0),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.3))),
    ));
    commands.spawn((
        Transform::from_xyz(-2.5, 3.0, 0.0),
        SoftBody::sphere(0.6, 2).map(|b| b.particle_mass(0.05)),
        SoftBodyVolumeFactor(1.1),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.4, 0.8))),
    ));
}

/// Cuts the cloth along the plane `x = 0` when space is pressed.
pub fn cut_cloth(
    keys: Res<ButtonInput<KeyCode>>,
    mut context: WriteRapierContext,
    mut commands: Commands,
    cloths: Query<Entity, With<Cloth>>,
) {
    if !keys.just_pressed(KeyCode::Space) {
        return;
    }
    let Ok(mut context) = context.single_mut() else {
        return;
    };
    for cloth in &cloths {
        // A large triangle in the plane `x = 0`, crossing the whole cloth.
        let blade = [
            Vec3::new(0.0, -10.0, -10.0),
            Vec3::new(0.0, 20.0, -10.0),
            Vec3::new(0.0, -10.0, 20.0),
        ];
        if let Some(tear) = context.cut_soft_body(&mut commands, cloth, &blade) {
            info!("The cut split the cloth into {:?}", tear.pieces);
        }
    }
}

/// Logs the pieces soft bodies tear into.
pub fn log_tears(mut tears: MessageReader<SoftBodyTearEvent>) {
    for tear in tears.read() {
        let pieces: Vec<_> = tear.pieces.iter().map(|piece| piece.soft_body).collect();
        info!("Soft body {} tore into {pieces:?}", tear.soft_body);
    }
}
