//! Tearing (`tear_strain`): a pinned sheet ripped by a dropped ball, a curtain shot through by a
//! box, and a jelly bar pulled apart by its pinned ends. Cloth springs are stiff (100 Hz) so only
//! impacts pass the tear strain. Torn-off pieces become soft body entities of their own.
//!
//! Every `SoftBodyTearEvent` is logged and its cracks flash red. Press the up/down arrows to set
//! the smallest piece a tear may split off, and `D` to restore the default (10 triangles or 6
//! cells).

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// The smallest piece a tear may split off (`None` for Rapier's default), applied to every soft
/// body.
#[derive(Resource, Default)]
struct MinPiece(Option<u32>);

/// Marks the text showing [`MinPiece`].
#[derive(Component)]
struct MinPieceText;

/// Where recent cracks opened, with their remaining display time.
#[derive(Resource, Default)]
struct TearMarks(Vec<(Vec3, f32)>);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .init_resource::<MinPiece>()
        .init_resource::<TearMarks>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (
                drive_bar,
                (log_tears, draw_tear_marks).chain(),
                update_min_piece,
            ),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(9.0, 8.0, 16.0).looking_at(Vec3::new(0.0, 1.5, 1.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        MinPieceText,
        Text::new(""),
        TextColor(Color::BLACK),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

/// A cloth material with stiff springs, tearing past `tear_strain`.
fn stiff_cloth(tear_strain: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        tear_strain: Some(tear_strain),
        ..RapierSoftBodyMaterial::uniform(SpringCoefficients::new(100.0, 1.0))
    })
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
    let projectile_color = materials.add(Color::srgb(0.3, 0.3, 0.35));

    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(30.0, 0.1, 30.0),
    ));

    /*
     * A sheet pinned along its border: the heavy ball dropped on it rips through. Particle
     * `(i, j)` of a cloth has the index `i * nv + j`.
     */
    let n = 40;
    let border = (0..n * n).filter(move |k| {
        let (i, j) = (k / n, k % n);
        i == 0 || j == 0 || i == n - 1 || j == n - 1
    });
    commands.spawn((
        Transform::from_xyz(-6.0, 3.0, -2.0),
        SoftBody::cloth(
            Vec3::ZERO,
            Vec3::X * 0.1,
            Vec3::Z * 0.1,
            n as usize,
            n as usize,
        )
        .map(|b| b.pinned_particles(border).particle_mass(0.02)),
        stiff_cloth(0.4),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(cloth_material(Color::srgb(0.8, 0.2, 0.2)))),
    ));
    commands.spawn((
        Transform::from_xyz(-4.0, 6.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.6),
        ColliderMassProperties::Density(30.0),
        Mesh3d(meshes.add(Sphere::new(0.6))),
        MeshMaterial3d(projectile_color.clone()),
    ));

    /*
     * A curtain pinned along its top edge, with a heavy box shot through it.
     */
    commands.spawn((
        Transform::from_xyz(0.0, 4.5, 4.0),
        SoftBody::cloth(Vec3::ZERO, Vec3::X * 0.1, Vec3::NEG_Y * 0.1, 50, 40).map(|b| {
            b.pinned_particles((0..50).map(|i| i * 40))
                .particle_mass(0.02)
        }),
        stiff_cloth(0.2),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(cloth_material(Color::srgb(0.2, 0.5, 0.8)))),
    ));
    commands.spawn((
        Transform::from_xyz(2.5, 2.5, 12.0).with_rotation(Quat::from_scaled_axis(Vec3::splat(0.5))),
        RigidBody::Dynamic,
        Velocity::linear(Vec3::new(0.0, 0.0, -25.0)),
        Collider::cuboid(0.3, 0.3, 0.3),
        ColliderMassProperties::Density(20.0),
        Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
        MeshMaterial3d(projectile_color),
    ));

    /*
     * A jelly bar pinned at both ends: the right end is pulled away until the bar snaps.
     */
    let bar_center = Vec3::new(3.0, 1.0, -4.0);
    let bar = SoftBody::cuboid(Vec3::new(2.0, 0.4, 0.4), 21, 5, 5).map(|b| {
        b.cell_model(SoftBodyCellModel::Corotational)
            .particle_mass(0.05)
    });
    let positions = bar.builder.particle_positions().to_vec();
    let left: Vec<u32> = (0..positions.len() as u32)
        .filter(|i| positions[*i as usize].x < -1.99)
        .collect();
    let right: Vec<u32> = (0..positions.len() as u32)
        .filter(|i| positions[*i as usize].x > 1.99)
        .collect();
    let pinned: Vec<u32> = left.iter().chain(&right).copied().collect();
    commands.spawn((
        Transform::from_translation(bar_center),
        bar.map(|b| b.pinned_particles(pinned)),
        // The right end is dragged by moving these targets (see `drive_bar`). A tear moves
        // them to the pieces holding their particles.
        SoftBodyKinematicTargets(
            right
                .iter()
                .map(|i| (*i, bar_center + positions[*i as usize]))
                .collect(),
        ),
        // Stiff enough not to tear from sagging between its pinned ends.
        SoftBodyMaterial(RapierSoftBodyMaterial {
            young_modulus: 5.0e4,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            tear_strain: Some(0.4),
            ..default()
        }),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.55, 0.8, 0.6))),
    ));
}

/// Drags the right end of the bar by moving its kinematic targets.
fn drive_bar(
    time: Res<Time>,
    mut last_shift: Local<f32>,
    mut targets: Query<&mut SoftBodyKinematicTargets>,
) {
    // The right end starts moving after a second, at half a meter per second, and stops once
    // the bar has doubled its length.
    let shift = ((time.elapsed_secs() - 1.0).max(0.0) * 0.5).min(4.0);
    let delta = shift - std::mem::replace(&mut *last_shift, shift);
    if delta == 0.0 {
        return;
    }
    for mut targets in &mut targets {
        for (_, target) in &mut targets.0 {
            *target += Vec3::new(delta, 0.0, 0.0);
        }
    }
}

/// Logs the tears, and remembers where their cracks opened.
fn log_tears(
    context: ReadRapierContext,
    mut tears: MessageReader<SoftBodyTearEvent>,
    mut marks: ResMut<TearMarks>,
) {
    let Ok(context) = context.single() else {
        return;
    };
    for tear in tears.read() {
        let pieces: Vec<_> = tear.pieces.iter().map(|piece| piece.soft_body).collect();
        info!(
            "Soft body {} tore: {} edges and {} cells cracked, pieces: {pieces:?}",
            tear.soft_body,
            tear.raw.torn_edges.len(),
            tear.raw.torn_cells.len(),
        );

        // The current position of a particle of the torn body (a post-tear index).
        let position = |particle: u32| {
            let (handle, i) = tear.raw.particle_destination(particle)?;
            let sb = context.rigidbody_set.soft_bodies.get(handle)?;
            (i < sb.num_particles() as u32).then(|| sb.particle_position(i as usize))
        };
        let edges = tear.raw.torn_edges.iter().map(|e| e.as_slice());
        let cells = tear.raw.torn_cells.iter().map(|c| c.as_slice());
        for element in edges.chain(cells) {
            let points: Option<Vec<Vec3>> = element.iter().map(|v| position(*v)).collect();
            if let Some(points) = points {
                let center = points.iter().sum::<Vec3>() / points.len() as f32;
                marks.0.push((center, 1.0));
            }
        }
    }
}

/// Draws the recent cracks, fading out over a second.
fn draw_tear_marks(time: Res<Time>, mut marks: ResMut<TearMarks>, mut gizmos: Gizmos) {
    for (center, life) in &mut marks.0 {
        gizmos.sphere(*center, 0.05, Color::srgba(0.9, 0.1, 0.1, *life));
        *life -= time.delta_secs();
    }
    marks.0.retain(|(_, life)| *life > 0.0);
}

/// Changes the smallest piece a tear may split off with the keyboard.
fn update_min_piece(
    keys: Res<ButtonInput<KeyCode>>,
    mut min_piece: ResMut<MinPiece>,
    mut soft_bodies: Query<&mut SoftBodyMaterial>,
    mut text: Query<&mut Text, With<MinPieceText>>,
) {
    if keys.just_pressed(KeyCode::ArrowUp) {
        min_piece.0 = Some(min_piece.0.map_or(6, |n| (n + 1).min(40)));
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        min_piece.0 = Some(min_piece.0.map_or(6, |n| n.saturating_sub(1).max(1)));
    }
    if keys.just_pressed(KeyCode::KeyD) {
        min_piece.0 = None;
    }
    if !min_piece.is_changed() {
        return;
    }

    // The pieces split off by a tear are clones of their soft body's entity, material included.
    for mut material in &mut soft_bodies {
        material.min_piece = min_piece.0;
    }
    for mut text in &mut text {
        text.0 = match min_piece.0 {
            Some(n) => format!("Minimum piece: {n} elements (up/down, D for default)"),
            None => {
                "Minimum piece: default, 10 triangles or 6 cells (up/down to change)".to_string()
            }
        };
    }
}
