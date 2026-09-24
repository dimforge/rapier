use bevy::prelude::*;
use bevy_rapier2d::geometry::RapierColliderHandle;
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
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, (check_collisions, check_collider_enabled))
        .run();
}

fn setup(mut commands: Commands) {
    // 2D camera
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 20.0, 0.0)));

    let scale = 15f32;

    let nx = 50;
    for i in 0..nx {
        for j in 0..10 {
            let falling_objects = (j + i) % 3;
            let ball_radius = 0.5 * scale;

            let collider = match falling_objects {
                0 => Collider::ball(ball_radius),
                1 => Collider::cuboid(ball_radius, ball_radius),
                2 => Collider::capsule_y(ball_radius, ball_radius),
                _ => unreachable!(),
            };

            commands
                .spawn((
                    Transform::from_xyz(
                        (i as f32 * 2.0 - nx as f32 / 2.0) * scale,
                        (20.0 + j as f32 * 2.0) * scale,
                        0.0,
                    ),
                    RigidBody::Dynamic,
                    AdditionalMassProperties::Mass(1.0),
                ))
                .with_children(|parent| {
                    parent.spawn((collider, ColliderDisabled));
                });
        }
    }

    let polyline = [
        Vec2::new(0.0, 0.0),
        Vec2::new(0.0, 10.0),
        Vec2::new(7.0, 4.0),
        Vec2::new(14.0, 10.0),
        Vec2::new(14.0, 0.0),
        Vec2::new(13.0, 7.0),
        Vec2::new(7.0, 2.0),
        Vec2::new(1.0, 7.0),
    ]
    .iter()
    .map(|p| p * scale)
    .collect::<Vec<_>>();
    let indices: Vec<_> = (0..polyline.len() as u32)
        .map(|i| [i, (i + 1) % polyline.len() as u32])
        .collect();

    commands.spawn((
        Transform::from_xyz(-20.0 * scale, -10.0 * scale, 0.0),
        Collider::voxelized_mesh(&polyline, &indices, 0.2 * scale, FillMode::default()),
        ColliderDisabled,
    ));

    let voxel_size = Vec2::new(scale, scale);
    let voxels: Vec<_> = (0..300)
        .map(|i| {
            let y = (i as f32 / 20.0).sin().clamp(-0.5, 0.5) * 20.0;
            Vec2::new((i as f32 - 125.0) * voxel_size.x / 2.0, y * voxel_size.y)
        })
        .collect();
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        Collider::voxels_from_points(voxel_size, &voxels),
    ));
}

fn check_collisions(mut collision_events: MessageReader<CollisionEvent>) {
    for event in collision_events.read() {
        info!("Collision event: {:?}", event);
    }
}

fn check_collider_enabled(
    rapier_context: ReadRapierContext,
    query: Query<&RapierColliderHandle, With<ColliderDisabled>>,
) {
    let context = match rapier_context.single() {
        Ok(c) => c,
        Err(_) => return,
    };

    for rapier_handle in &query {
        let handle = rapier_handle.0;
        if let Some(col) = context.colliders.colliders.get(handle) {
            if col.is_enabled() {
                info!(
                    "FIX ME: Voxel collider should NOT be enabled: {}",
                    col.is_enabled()
                );
            }
        }
    }
}
