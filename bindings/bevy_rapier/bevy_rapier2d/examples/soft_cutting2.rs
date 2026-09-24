//! Cutting soft bodies with `RapierContextMut::cut_soft_body`: drag the mouse with its left
//! button (or hold `C` while moving it) to cut every soft body along the line from the press to
//! the release; a saw blade also rises through the suspended slab. Cuts remove no material and
//! the separated pieces become soft body entities of their own.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// The start of the cut being drawn, if any.
#[derive(Resource, Default)]
struct BladeStart(Option<Vec2>);

/// Marks the saw blade.
#[derive(Component)]
struct Saw;

/// The position of the saw before it starts rising.
const SAW_START: Vec2 = Vec2::new(10.0, 1.5);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .init_resource::<BladeStart>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (cut_with_mouse, move_saw))
        .run();
}

fn setup_graphics(mut commands: Commands, mut debug_render: ResMut<DebugRenderContext>) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 40.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(1.0, 3.5, 0.0),
    ));
    // Draw the elements of the soft bodies.
    debug_render.mode.soft_bodies = true;
}

/// A jelly material of the given Young modulus.
fn jelly(young_modulus: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio: 0.35,
        elastic_damping_ratio: 1.0,
        ..default()
    })
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let jelly_color = materials.add(Color::srgb(0.95, 0.65, 0.3));

    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider::cuboid(30.0, 0.5),
    ));

    /*
     * A jelly block to slice by hand.
     */
    commands.spawn((
        Transform::from_xyz(-8.0, 2.0, 0.0),
        SoftBody::grid(Vec2::splat(2.0), 13, 13).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .particle_mass(0.05)
                .particle_radius(0.15)
        }),
        jelly(2.0e4),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color.clone()),
    ));

    /*
     * A blob: a ring of particles holding its area. Cut open, it falls limp.
     */
    commands.spawn((
        Transform::from_xyz(-2.0, 2.0, 0.0),
        SoftBody::disk(1.6, 40).map(|b| {
            b.softness(SpringCoefficients::new(30.0, 1.0))
                .particle_mass(0.05)
        }),
        Friction::coefficient(0.8),
    ));

    /*
     * A curtain pinned along its top edge. Particle `(i, j)` of a grid has the index
     * `i * ny + j`.
     */
    let (nx, ny) = (9, 31);
    let idx = move |i: usize, j: usize| (i * ny + j) as u32;
    commands.spawn((
        Transform::from_xyz(3.0, 4.5, 0.0),
        SoftBody::grid(Vec2::new(0.6, 3.0), nx, ny).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .pinned_particles((0..nx).map(|i| idx(i, ny - 1)))
                .particle_mass(0.05)
                .particle_radius(0.1)
        }),
        jelly(3.0e4),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color.clone()),
    ));

    /*
     * A slab suspended between two posts, with a saw rising through it.
     */
    let (nx, ny) = (25, 5);
    let idx = move |i: usize, j: usize| (i * ny + j) as u32;
    commands.spawn((
        Transform::from_xyz(10.0, 4.0, 0.0),
        SoftBody::grid(Vec2::new(3.0, 0.5), nx, ny).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .pinned_particles((0..ny).flat_map(|j| [idx(0, j), idx(nx - 1, j)]))
                .particle_mass(0.05)
                .particle_radius(0.1)
        }),
        jelly(5.0e4),
        Friction::coefficient(0.8),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(jelly_color),
    ));
    // The saw: a kinematic sensor, so it pushes nothing (the cut does the work).
    commands.spawn((
        Saw,
        Transform::from_translation(SAW_START.extend(1.0)),
        RigidBody::KinematicPositionBased,
        Collider::cuboid(0.05, 1.0),
        Sensor,
        Mesh2d(meshes.add(Rectangle::new(0.1, 2.0))),
        MeshMaterial2d(materials.add(Color::srgb(0.3, 0.3, 0.35))),
    ));
}

/// Cuts every soft body along the line drawn with the mouse.
fn cut_with_mouse(
    (buttons, keys): (Res<ButtonInput<MouseButton>>, Res<ButtonInput<KeyCode>>),
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut blade_start: ResMut<BladeStart>,
    (mut context, mut commands): (WriteRapierContext, Commands),
    soft_bodies: Query<Entity, With<SoftBody>>,
    mut gizmos: Gizmos,
) {
    let (camera, camera_transform) = *camera;
    let cursor = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok());

    if buttons.pressed(MouseButton::Left) || keys.pressed(KeyCode::KeyC) {
        if blade_start.0.is_none() {
            blade_start.0 = cursor;
        }
        if let (Some(a), Some(b)) = (blade_start.0, cursor) {
            gizmos.line_2d(a, b, Color::srgb(1.0, 0.35, 0.25));
        }
    } else if let (Some(a), Some(b)) = (blade_start.0.take(), cursor) {
        let Ok(mut context) = context.single_mut() else {
            return;
        };
        for soft_body in &soft_bodies {
            context.cut_soft_body(&mut commands, soft_body, &[a, b]);
        }
    }
}

/// Raises the saw, cutting every soft body along its blade.
fn move_saw(
    time: Res<Time>,
    mut context: WriteRapierContext,
    mut commands: Commands,
    mut saws: Query<&mut Transform, With<Saw>>,
    soft_bodies: Query<Entity, With<SoftBody>>,
) {
    let Ok(mut context) = context.single_mut() else {
        return;
    };
    // The saw rises at a fifth of a meter per second, from a second in.
    let rise = ((time.elapsed_secs() - 1.0).max(0.0) * 0.2).min(4.5);
    let position = SAW_START + Vec2::new(0.0, rise);
    for mut transform in &mut saws {
        transform.translation = position.extend(transform.translation.z);
    }
    let blade = [position - Vec2::Y, position + Vec2::Y];
    for soft_body in &soft_bodies {
        context.cut_soft_body(&mut commands, soft_body, &blade);
    }
}
