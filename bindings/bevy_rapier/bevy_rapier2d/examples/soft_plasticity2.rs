//! Plasticity: elastic cells whose rest shape flows once their strain exceeds a yield
//! (`plastic_yield` and `plastic_creep` of the soft-body material), shown on squares hit by disks,
//! a clay slab stamped by a kinematic press, a creeping column, and clay disks splatting on a wall.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// Marks the kinematic press.
#[derive(Component)]
struct Press;

/// The position of the press above its first spot.
const PRESS_REST: Vec2 = Vec2::new(-2.0, 2.4);

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
        .add_systems(Update, move_press)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 38.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 3.0, 0.0),
    ));
}

/// A clay material: elastic up to `plastic_yield`, flowing at the rate `plastic_creep` beyond.
fn clay(young_modulus: f32, plastic_yield: f32, plastic_creep: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio: 0.35,
        elastic_damping_ratio: 1.0,
        plastic_yield,
        plastic_creep,
        deformation_damping: 4.0,
        ..default()
    })
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let clay_color = materials.add(Color::srgb(0.8, 0.5, 0.35));
    let elastic_color = materials.add(Color::srgb(0.4, 0.7, 0.9));
    let disk_color = materials.add(Color::srgb(0.3, 0.3, 0.4));
    let clay_color = |plastic_yield: f32| {
        if plastic_yield > 0.0 {
            clay_color.clone()
        } else {
            elastic_color.clone()
        }
    };

    /*
     * Ground, and a wall for the clay disks.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider::cuboid(16.0, 0.5),
    ));
    commands.spawn((
        Transform::from_xyz(13.0, 2.0, 0.0),
        Collider::cuboid(0.2, 2.0),
        Friction::coefficient(0.8),
    ));

    /*
     * The yield ladder: identical squares, from purely elastic (left) to very plastic (right),
     * each hit by the same heavy disk.
     */
    for (i, plastic_yield) in [0.0, 0.2, 0.08, 0.02].into_iter().enumerate() {
        let x = -13.0 + i as f32 * 2.4;
        commands.spawn((
            Transform::from_xyz(x, 0.75, 0.0),
            SoftBody::grid(Vec2::splat(0.75), 6, 6).map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.1)
            }),
            clay(1.0e4, plastic_yield, 20.0),
            Friction::coefficient(0.8),
            SoftBodyMeshSync::default(),
            MeshMaterial2d(clay_color(plastic_yield)),
        ));
        commands.spawn((
            Transform::from_xyz(x, 6.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.4),
            ColliderMassProperties::Density(5.0),
            Mesh2d(meshes.add(Circle::new(0.4))),
            MeshMaterial2d(disk_color.clone()),
        ));
    }

    /*
     * A clay slab stamped by a kinematic press: the imprints stay after the press lifts.
     */
    commands.spawn((
        Transform::from_xyz(0.0, 0.5, 0.0),
        SoftBody::grid(Vec2::new(3.0, 0.5), 25, 5).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .particle_mass(0.1)
        }),
        clay(3.0e4, 0.02, 50.0),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(clay_color(0.02)),
    ));
    commands.spawn((
        Press,
        Transform::from_translation(PRESS_REST.extend(0.0))
            .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_4)),
        RigidBody::KinematicPositionBased,
        Collider::cuboid(0.4, 0.4),
        Friction::coefficient(0.5),
        Mesh2d(meshes.add(Rectangle::new(0.8, 0.8))),
        MeshMaterial2d(disk_color.clone()),
    ));

    /*
     * Two soft columns under their own weight: the elastic one stands, the plastic one creeps,
     * leans and collapses into a heap (a body whose cells keep flowing is kept awake).
     */
    for (i, plastic_yield) in [0.0, 0.04].into_iter().enumerate() {
        let x = 5.5 + i as f32 * 1.5;
        commands.spawn((
            Transform::from_xyz(x, 1.2, 0.0),
            SoftBody::grid(Vec2::new(0.3, 1.2), 3, 12).map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.2)
            }),
            SoftBodyMaterial(RapierSoftBodyMaterial {
                young_modulus: 2.0e3,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 1.0,
                plastic_yield,
                plastic_creep: 0.5,
                deformation_damping: 4.0,
                ..default()
            }),
            Friction::coefficient(1.0),
            SoftBodyMeshSync::default(),
            MeshMaterial2d(clay_color(plastic_yield)),
        ));
    }

    /*
     * Clay disks thrown at the wall: 24-gons filled with triangular cells.
     */
    let n = 24;
    let vertices: Vec<Vec2> = (0..n)
        .map(|k| Vec2::from_angle(k as f32 / n as f32 * std::f32::consts::TAU) * 0.5)
        .collect();
    let indices: Vec<[u32; 2]> = (0..n).map(|i| [i as u32, ((i + 1) % n) as u32]).collect();
    for i in 0..3 {
        let Some(disk) = SoftBody::volumetric(&vertices, &indices, 0.15) else {
            continue;
        };
        commands.spawn((
            Transform::from_xyz(11.5 - i as f32 * 1.5, 1.0 + i as f32 * 0.5, 0.0),
            disk.map(|b| {
                b.cell_model(SoftBodyCellModel::Corotational)
                    .particle_mass(0.05)
            }),
            clay(1.0e4, 0.03, 60.0),
            Friction::coefficient(0.8),
            // Throw the whole disk toward the wall.
            SoftBodyExternalImpulse {
                velocity_change: Vec2::new(10.0, 1.0),
                ..default()
            },
            SoftBodyMeshSync::default(),
            MeshMaterial2d(clay_color(0.03)),
        ));
    }
}

/// Stamps a new spot of the slab every 3 seconds: down for a second, up for a second, then
/// slides to the next spot.
fn move_press(time: Res<Time>, mut presses: Query<&mut Transform, With<Press>>) {
    let t = time.elapsed_secs();
    let period = 3.0;
    let cycle = (t / period).floor();
    let phase = t - cycle * period;
    let depth = 1.0;
    let x = PRESS_REST.x + (cycle % 5.0);
    let y = PRESS_REST.y
        - if phase < 1.0 {
            depth * phase
        } else if phase < 2.0 {
            depth * (2.0 - phase)
        } else {
            0.0
        };
    for mut transform in &mut presses {
        transform.translation.x = x;
        transform.translation.y = y;
    }
}
