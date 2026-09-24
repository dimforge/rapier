//! Tearing (`tear_strain`): a pinned jelly bridge broken by a heavy disk, a hanging strip shot
//! through by a fast disk, and a jelly bar pulled apart by its pinned ends. Cracks split particles
//! and shed pieces as new soft body entities.
//!
//! Every `SoftBodyTearEvent` is logged and its cracks flash red. Press the up/down arrows to set
//! the smallest piece a tear may split off, and `D` to restore the default (3 cells).

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// The smallest piece a tear may split off (`None` for Rapier's default), applied to every soft
/// body.
#[derive(Resource, Default)]
struct MinPiece(Option<u32>);

/// Marks the text showing [`MinPiece`].
#[derive(Component)]
struct MinPieceText;

/// Where recent cracks opened, with their remaining display time.
#[derive(Resource, Default)]
struct TearMarks(Vec<(Vec2, f32)>);

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

fn setup_graphics(mut commands: Commands, mut debug_render: ResMut<DebugRenderContext>) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 36.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 4.0, 0.0),
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
    // Color the soft-body elements by their load, up to the tear threshold.
    debug_render.mode.soft_bodies = true;
    debug_render.mode.soft_body_stress = true;
}

/// A jelly material tearing past `tear_strain`.
fn jelly(young_modulus: f32, tear_strain: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        tear_strain: Some(tear_strain),
        ..default()
    })
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let jelly_color = materials.add(Color::srgb(0.55, 0.8, 0.6));
    let disk_color = materials.add(Color::srgb(0.2, 0.4, 0.8));

    /*
     * Ground, and a wall catching the disk shot through the strip.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider::cuboid(30.0, 0.5),
    ));
    commands.spawn((
        Transform::from_xyz(16.0, 3.0, 0.0),
        Collider::cuboid(0.3, 3.0),
    ));

    /*
     * A bridge pinned at both ends: the heavy disk dropped on its middle breaks it. Particle
     * `(i, j)` of a grid has the index `i * ny + j`.
     */
    let (nx, ny) = (31, 6);
    let idx = move |i: usize, j: usize| (i * ny + j) as u32;
    commands.spawn((
        Transform::from_xyz(-8.0, 4.0, 0.0),
        SoftBody::grid(Vec2::new(3.0, 0.5), nx, ny).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .pinned_particles((0..ny).flat_map(|j| [idx(0, j), idx(nx - 1, j)]))
                .particle_mass(0.05)
        }),
        // Stiff enough to hold its own weight under the tear strain.
        jelly(1.0e6, 0.35),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color.clone()),
    ));
    commands.spawn((
        Transform::from_xyz(-8.0, 9.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.6),
        ColliderMassProperties::Density(20.0),
        Mesh2d(meshes.add(Circle::new(0.6))),
        MeshMaterial2d(disk_color.clone()),
    ));

    /*
     * A strip hanging from its top edge, with a fast disk shot through it.
     */
    let (nx, ny) = (7, 41);
    let idx = move |i: usize, j: usize| (i * ny + j) as u32;
    commands.spawn((
        Transform::from_xyz(2.0, 5.0, 0.0),
        SoftBody::grid(Vec2::new(0.45, 3.0), nx, ny).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .pinned_particles((0..nx).map(|i| idx(i, ny - 1)))
                .particle_mass(0.05)
        }),
        jelly(3.0e4, 0.35),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color.clone()),
    ));
    commands.spawn((
        Transform::from_xyz(-4.0, 4.5, 0.0),
        RigidBody::Dynamic,
        Velocity::linear(Vec2::new(20.0, 0.0)),
        Collider::ball(0.4),
        ColliderMassProperties::Density(10.0),
        Mesh2d(meshes.add(Circle::new(0.4))),
        MeshMaterial2d(disk_color),
    ));

    /*
     * A jelly bar pinned at both ends: the right end is pulled away until the bar snaps.
     */
    let bar_center = Vec2::new(-3.0, 1.0);
    let bar = SoftBody::grid(Vec2::new(2.0, 0.4), 21, 5).map(|b| {
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
        Transform::from_translation(bar_center.extend(0.0)),
        bar.map(|b| b.pinned_particles(pinned)),
        // The right end is dragged by moving these targets (see `drive_bar`). A tear moves
        // them to the pieces holding their particles.
        SoftBodyKinematicTargets(
            right
                .iter()
                .map(|i| (*i, bar_center + positions[*i as usize]))
                .collect(),
        ),
        jelly(5.0e4, 0.35),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color),
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
            *target += Vec2::new(delta, 0.0);
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
            let points: Option<Vec<Vec2>> = element.iter().map(|v| position(*v)).collect();
            if let Some(points) = points {
                let center = points.iter().sum::<Vec2>() / points.len() as f32;
                marks.0.push((center, 1.0));
            }
        }
    }
}

/// Draws the recent cracks, fading out over a second.
fn draw_tear_marks(time: Res<Time>, mut marks: ResMut<TearMarks>, mut gizmos: Gizmos) {
    for (center, life) in &mut marks.0 {
        gizmos.circle_2d(*center, 0.12, Color::srgba(0.9, 0.1, 0.1, *life));
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
        min_piece.0 = Some(min_piece.0.map_or(3, |n| (n + 1).min(40)));
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        min_piece.0 = Some(min_piece.0.map_or(3, |n| n.saturating_sub(1).max(1)));
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
            None => "Minimum piece: default, 3 cells (up/down to change)".to_string(),
        };
    }
}
