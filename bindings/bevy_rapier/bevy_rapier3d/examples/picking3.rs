//! A simple scene to demonstrate picking events for rapier [`Collider`] entities.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
            RapierPickingPlugin,
        ))
        .insert_resource(RapierPickingSettings {
            // Optional: only needed when you want fine-grained control
            // over which cameras and entities should be used with the rapier picking backend.
            // This is disabled by default, and no marker components are required on cameras or colliders.
            // This resource is inserted by default,
            // you only need to add it if you want to override the default settings.
            require_markers: true,
            ..Default::default()
        })
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
        RapierPickable,
    ));
}

pub fn setup_physics(mut commands: Commands) {
    commands
        .spawn((
            Text::new("Click Me to get a box\nDrag cubes to rotate"),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Percent(12.0),
                left: Val::Percent(12.0),
                ..default()
            },
        ))
        .observe(on_click_spawn_cube)
        .observe(|out: On<PointerOut>, mut texts: Query<&mut TextColor>| {
            let mut text_color = texts.get_mut(out.event().entity).unwrap();
            text_color.0 = Color::WHITE;
        })
        .observe(|over: On<PointerOver>, mut texts: Query<&mut TextColor>| {
            let mut color = texts.get_mut(over.event().entity).unwrap();
            color.0 = bevy::color::palettes::tailwind::CYAN_400.into();
        });
    // Base
    let ground_size = 3.1;
    let ground_height = 0.1;
    commands.spawn((
        Transform::from_xyz(0.0, -ground_height / 2.0, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
        ColliderDebugColor(Hsla::BLACK),
    ));
}

fn on_click_spawn_cube(_click: On<PointerClick>, mut commands: Commands, mut num: Local<usize>) {
    let rad = 0.25;
    let colors = [
        Hsla::hsl(220.0, 1.0, 0.3),
        Hsla::hsl(180.0, 1.0, 0.3),
        Hsla::hsl(260.0, 1.0, 0.7),
    ];
    commands
        .spawn((
            Transform::from_xyz(0.0, 0.25 + 0.55 * *num as f32, 0.0),
            Visibility::default(),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad, rad),
            ColliderDebugColor(colors[*num % 3]),
            RapierPickable,
        ))
        // With the RapierPickingPlugin added, you can add pointer event observers to colliders:
        .observe(on_drag_rotate);
    *num += 1;
}

fn on_drag_rotate(drag: On<PointerDrag>, mut transforms: Query<&mut Transform>) {
    if let Ok(mut transform) = transforms.get_mut(drag.event().entity) {
        transform.rotate_y(drag.delta.x * 0.02);
        transform.rotate_x(drag.delta.y * 0.02);
    }
}
