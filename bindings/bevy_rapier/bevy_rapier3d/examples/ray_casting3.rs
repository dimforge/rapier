use bevy::color::palettes::basic;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
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
        .add_systems(Update, cast_ray)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;

                // Build the rigid body.
                commands.spawn((
                    Transform::from_xyz(x, y, z),
                    RigidBody::Dynamic,
                    Collider::cuboid(rad, rad, rad),
                ));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

pub fn cast_ray(
    mut commands: Commands,
    window: Single<&Window, With<PrimaryWindow>>,
    rapier_context: ReadRapierContext,
    cameras: Query<(&Camera, &GlobalTransform)>,
) -> Result<()> {
    let Some(cursor_position) = window.cursor_position() else {
        log::warn!("Cursor is outside the window area");
        return Ok(());
    };

    // We will color in read the colliders hovered by the mouse.
    for (camera, camera_transform) in &cameras {
        // First, compute a ray from the mouse position.
        let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
            log::warn!("Unable to compute ray from mouse position");
            return Ok(());
        };
        let context = rapier_context.single()?;
        context.with_query_pipeline(QueryFilter::only_dynamic(), |query_pipeline| {
            // Then cast the ray.
            let hit = query_pipeline.cast_ray(ray.origin, ray.direction.into(), f32::MAX, true);

            if let Some((entity, _toi)) = hit {
                // Color in blue the entity we just hit.
                // Because of the query filter, only colliders attached to a dynamic body
                // will get an event.
                let color = basic::BLUE.into();
                commands.entity(entity).insert(ColliderDebugColor(color));
            }
        });
    }

    Ok(())
}
