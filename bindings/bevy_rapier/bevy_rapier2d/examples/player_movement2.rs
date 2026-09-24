use bevy::{prelude::*, window::WindowResolution};
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    resolution: WindowResolution::new(1000, 1000),
                    title: "Player Movement Example".to_string(),
                    ..default()
                }),
                ..default()
            }),
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, spawn_player)
        .add_systems(Update, player_movement)
        .run();
}

// The float value is the player movement speed in 'pixels/second'.
#[derive(Component)]
pub struct Player(f32);

pub fn spawn_player(
    mut commands: Commands,
    mut rapier_config: Query<&mut RapierConfiguration>,
) -> Result<()> {
    let mut rapier_config = rapier_config.single_mut()?;
    // Set gravity to 0.0 and spawn camera.
    rapier_config.gravity = Vec2::ZERO;
    commands.spawn(Camera2d);

    let sprite_size = 100.0;

    // Spawn entity with `Player` struct as a component for access in movement query.
    commands.spawn((
        Sprite {
            color: Color::srgb(0.0, 0.0, 0.0),
            custom_size: Some(Vec2::new(sprite_size, sprite_size)),
            ..Default::default()
        },
        RigidBody::Dynamic,
        Velocity::zero(),
        Collider::ball(sprite_size / 2.0),
        Player(100.0),
    ));

    Ok(())
}

pub fn player_movement(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut player_info: Query<(&Player, &mut Velocity)>,
) {
    for (player, mut rb_vels) in &mut player_info {
        let up = keyboard_input.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]);
        let down = keyboard_input.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]);
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);

        let x_axis = -(left as i8) + right as i8;
        let y_axis = -(down as i8) + up as i8;

        let mut move_delta = Vec2::new(x_axis as f32, y_axis as f32);
        if move_delta != Vec2::ZERO {
            move_delta /= move_delta.length();
        }

        // Update the velocity on the rigid_body_component,
        // the bevy_rapier plugin will update the Sprite transform.
        rb_vels.linear = move_delta * player.0;
    }
}
