//! Jelly: a stack of tetrahedral cubes of increasing softness, balloons piled in a box, a
//! shape-matched cube driven toward an animated pose, and a ball and a capsule filled with
//! tetrahedral cells from their boundary meshes.

use bevy::prelude::*;
use bevy_rapier3d::parry::shape::{Ball, Capsule};
use bevy_rapier3d::prelude::*;

/// Marks the shape-matched cube driven toward an animated pose.
#[derive(Component)]
struct Driven;

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
        .add_systems(Update, drive_cube)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(8.0, 7.0, 14.0).looking_at(Vec3::new(0.0, 1.5, 1.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// A corotational jelly material.
fn jelly(young_modulus: f32, poisson_ratio: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio,
        elastic_damping_ratio: 0.5,
        ..default()
    })
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let rigid_color = materials.add(Color::srgb(0.4, 0.4, 0.45));

    /*
     * Ground and a container for the balloons.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(12.0, 0.1, 12.0),
    ));
    for (dx, dz, hx, hz) in [
        (-2.5, 0.0, 0.1, 2.5),
        (2.5, 0.0, 0.1, 2.5),
        (0.0, -2.5, 2.5, 0.1),
        (0.0, 2.5, 2.5, 0.1),
    ] {
        commands.spawn((
            Transform::from_xyz(dx + 5.0, 1.0, dz),
            Collider::cuboid(hx, 1.0, hz),
        ));
    }

    /*
     * A stack of elastic cubes, softer at the top.
     */
    for (i, young_modulus) in [1.0e4, 4.0e3, 1.5e3].into_iter().enumerate() {
        commands.spawn((
            Transform::from_xyz(-4.0, 0.7 + 1.5 * i as f32, 0.0),
            SoftBody::cuboid(Vec3::splat(0.6), 5, 5, 5).map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.1)
            }),
            jelly(young_modulus, 0.4),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5 + 0.15 * i as f32, 0.4))),
        ));
    }

    /*
     * Balloons dropped in the container.
     */
    let balloon_color = materials.add(Color::srgb(0.9, 0.35, 0.35));
    for i in 0..6 {
        let x = 5.0 + ((i % 3) as f32 - 1.0) * 1.2;
        let z = ((i / 3) as f32 - 0.5) * 1.2;
        commands.spawn((
            Transform::from_xyz(x, 2.0 + i as f32 * 1.5, z),
            SoftBody::sphere(0.6, 2).map(|b| b.particle_mass(0.03)),
            SoftBodyMaterial::uniform(15.0, 1.0),
            SoftBodyVolumeFactor(1.1),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(balloon_color.clone()),
        ));
    }

    /*
     * A shape-matched particle cube driven toward an animated pose: it behaves like a squishy
     * kinematic body.
     */
    commands.spawn((
        Driven,
        Transform::from_xyz(0.45, 0.45, 0.45),
        SoftBody::cuboid(Vec3::splat(0.45), 4, 4, 4).map(|b| {
            b.shape_matching(true)
                .particle_radius(0.1)
                .particle_mass(0.2)
                .gravity_scale(0.0)
                .can_sleep(false)
        }),
        SoftBodyMaterial::uniform(15.0, 1.0),
        // Shape-matches the whole-body cluster toward a target pose, animated by `drive_cube`.
        SoftBodyClusterShapeMatching::default(),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.95, 0.75, 0.2))),
    ));

    /*
     * Volumetric jelly: a ball and a capsule filled with tetrahedral cells from their boundary
     * meshes, dropped behind the stack.
     */
    let volumetric_color = materials.add(Color::srgb(0.55, 0.4, 0.85));
    let (vertices, indices) = Ball::new(0.7).to_trimesh(16, 16);
    if let Some(ball) = SoftBody::volumetric(&vertices, &indices, 0.2) {
        commands.spawn((
            Transform::from_xyz(-4.0, 3.0, -4.0),
            ball.map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.05)
            }),
            jelly(1.0e4, 0.35),
            Friction::coefficient(0.7),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(volumetric_color.clone()),
        ));
    }
    let (vertices, indices) = Capsule::new_y(0.6, 0.45).to_trimesh(12, 12);
    if let Some(capsule) = SoftBody::volumetric(&vertices, &indices, 0.2) {
        commands.spawn((
            // The capsule is tilted by the transform of its entity.
            Transform::from_xyz(-4.0, 6.0, -4.0).with_rotation(Quat::from_rotation_z(1.2)),
            capsule.map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.05)
            }),
            jelly(3.0e4, 0.35),
            Friction::coefficient(0.7),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(volumetric_color),
        ));
    }

    /*
     * Rigid boxes to be shoved around by the driven cube.
     */
    let box_mesh = meshes.add(Cuboid::new(0.4, 0.4, 0.4));
    for i in 0..8 {
        commands.spawn((
            Transform::from_xyz(
                -0.5 + (i % 4) as f32 * 0.5,
                0.25,
                4.0 + (i / 4) as f32 * 0.5,
            ),
            RigidBody::Dynamic,
            Collider::cuboid(0.2, 0.2, 0.2),
            Mesh3d(box_mesh.clone()),
            MeshMaterial3d(rigid_color.clone()),
        ));
    }
}

/// Drives the shape-matched cube around the rigid boxes at height 0.6, spinning.
fn drive_cube(time: Res<Time>, mut driven: Query<&mut SoftBodyClusterShapeMatching, With<Driven>>) {
    let t = time.elapsed_secs();
    let target = Transform::from_xyz(2.0 * t.cos(), 0.6, 4.0 + 2.0 * t.sin())
        .with_rotation(Quat::from_rotation_y(2.0 * t));
    for mut shape_matching in &mut driven {
        shape_matching.target = Some(target);
    }
}
