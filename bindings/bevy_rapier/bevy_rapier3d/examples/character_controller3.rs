use bevy::{
    input::{mouse::MouseMotion, InputSystems},
    prelude::*,
};
use bevy_rapier3d::{control::KinematicCharacterController, prelude::*};

const MOUSE_SENSITIVITY: f32 = 0.3;
const GROUND_TIMER: f32 = 0.5;
const MOVEMENT_SPEED: f32 = 8.0;
const JUMP_SPEED: f32 = 20.0;
const GRAVITY: f32 = -9.81;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .init_resource::<MovementInput>()
        .init_resource::<LookInput>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_player, setup_map))
        .add_systems(PreUpdate, handle_input.after(InputSystems))
        .add_systems(Update, player_look)
        .add_systems(FixedUpdate, player_movement)
        .run();
}

pub fn setup_player(mut commands: Commands) {
    commands
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            Visibility::default(),
            Collider::round_cylinder(0.9, 0.3, 0.2),
            KinematicCharacterController {
                custom_mass: Some(5.0),
                up: Vec3::Y,
                offset: CharacterLength::Absolute(0.01),
                slide: true,
                autostep: Some(CharacterAutostep {
                    max_height: CharacterLength::Relative(0.3),
                    min_width: CharacterLength::Relative(0.5),
                    include_dynamic_bodies: false,
                }),
                // Don’t allow climbing slopes larger than 45 degrees.
                max_slope_climb_angle: 45.0_f32.to_radians(),
                // Automatically slide down on slopes smaller than 30 degrees.
                min_slope_slide_angle: 30.0_f32.to_radians(),
                apply_impulse_to_dynamic_bodies: true,
                snap_to_ground: None,
                ..default()
            },
        ))
        .with_children(|b| {
            // FPS Camera
            b.spawn((Camera3d::default(), Transform::from_xyz(0.0, 0.2, -0.1)));
        });
}

fn setup_map(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));
    /*
     * Stairs
     */
    let stair_len = 30;
    let stair_step = 0.2;
    for i in 1..=stair_len {
        let step = i as f32;
        let collider = Collider::cuboid(1.0, step * stair_step, 1.0);
        commands.spawn((
            Transform::from_xyz(40.0, step * stair_step, step * 2.0 - 20.0),
            collider.clone(),
        ));
        commands.spawn((
            Transform::from_xyz(-40.0, step * stair_step, step * -2.0 + 20.0),
            collider.clone(),
        ));
        commands.spawn((
            Transform::from_xyz(step * 2.0 - 20.0, step * stair_step, 40.0),
            collider.clone(),
        ));
        commands.spawn((
            Transform::from_xyz(step * -2.0 + 20.0, step * stair_step, -40.0),
            collider.clone(),
        ));
    }
}

/// Keyboard input vector
#[derive(Default, Resource, Deref, DerefMut)]
struct MovementInput(Vec3);

/// Mouse input vector
#[derive(Default, Resource, Deref, DerefMut)]
struct LookInput(Vec2);

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut movement: ResMut<MovementInput>,
    mut look: ResMut<LookInput>,
    mut mouse_events: MessageReader<MouseMotion>,
) {
    if keyboard.pressed(KeyCode::KeyW) {
        movement.z -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        movement.z += 1.0
    }
    if keyboard.pressed(KeyCode::KeyA) {
        movement.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        movement.x += 1.0
    }
    **movement = movement.normalize_or_zero();
    if keyboard.pressed(KeyCode::ShiftLeft) {
        **movement *= 2.0;
    }
    if keyboard.pressed(KeyCode::Space) {
        movement.y = 1.0;
    }

    for event in mouse_events.read() {
        look.x -= event.delta.x * MOUSE_SENSITIVITY;
        look.y -= event.delta.y * MOUSE_SENSITIVITY;
        look.y = look.y.clamp(-89.9, 89.9); // Limit pitch
    }
}

fn player_movement(
    time: Res<Time>,
    mut input: ResMut<MovementInput>,
    mut player: Query<(
        &mut Transform,
        &mut KinematicCharacterController,
        Option<&KinematicCharacterControllerOutput>,
    )>,
    mut vertical_movement: Local<f32>,
    mut grounded_timer: Local<f32>,
) {
    let Ok((transform, mut controller, output)) = player.single_mut() else {
        return;
    };
    let delta_time = time.delta_secs();
    // Retrieve input
    let mut movement = Vec3::new(input.x, 0.0, input.z) * MOVEMENT_SPEED;
    let jump_speed = input.y * JUMP_SPEED;
    // Clear input
    **input = Vec3::ZERO;
    // Check physics ground check
    if output.map(|o| o.grounded).unwrap_or(false) {
        *grounded_timer = GROUND_TIMER;
        *vertical_movement = 0.0;
    }
    // If we are grounded we can jump
    if *grounded_timer > 0.0 {
        *grounded_timer -= delta_time;
        // If we jump we clear the grounded tolerance
        if jump_speed > 0.0 {
            *vertical_movement = jump_speed;
            *grounded_timer = 0.0;
        }
    }
    movement.y = *vertical_movement;
    *vertical_movement += GRAVITY * delta_time * controller.custom_mass.unwrap_or(1.0);
    controller.translation = Some(transform.rotation * (movement * delta_time));
}

fn player_look(
    mut player: Query<&mut Transform, (With<KinematicCharacterController>, Without<Camera>)>,
    mut camera: Query<&mut Transform, With<Camera>>,
    input: Res<LookInput>,
) {
    let Ok(mut transform) = player.single_mut() else {
        return;
    };
    transform.rotation = Quat::from_axis_angle(Vec3::Y, input.x.to_radians());
    let Ok(mut transform) = camera.single_mut() else {
        return;
    };
    transform.rotation = Quat::from_axis_angle(Vec3::X, input.y.to_radians());
}
