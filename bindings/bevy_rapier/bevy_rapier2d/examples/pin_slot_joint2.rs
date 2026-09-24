//! A pin-slot (groove) joint: a cube tethered to a character slides along a slanted slot attached
//! to that character, while staying free to rotate around its pin.
//!
//! Move the character with the arrow keys (or A/D) and jump with Space.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

const CHARACTER_SPEED: f32 = 2.0;
const JUMP_SPEED: f32 = 4.0;
const GRAVITY: f32 = 9.81;

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
        .add_systems(Update, move_character)
        .run();
}

/// The vertical velocity of the manually-controlled character.
#[derive(Component, Default)]
struct Character {
    vertical_velocity: f32,
}

pub fn setup_graphics(mut commands: Commands) {
    // The scene is expressed in meters: zoom in so that one meter spans 100 pixels.
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 0.01,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 1.0, 0.0),
    ));
    commands.spawn((
        Text::new("Arrows/A/D: move the character\nSpace: jump"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        TextColor(Color::BLACK),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground.
     */
    let ground_size = 3.0;
    let ground_height = 0.1;
    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    /*
     * Character we will control manually.
     */
    let character_pos = Vec2::new(0.0, 0.3);
    let character = commands
        .spawn((
            Transform::from_translation(character_pos.extend(0.0)),
            RigidBody::KinematicPositionBased,
            Collider::cuboid(0.15, 0.3),
            KinematicCharacterController::default(),
            Character::default(),
        ))
        .id();

    /*
     * Tethered cube, attached to the character with a pin-slot joint. The slot follows the
     * diagonal axis attached to the character, and the cube can't go below one meter along it.
     */
    let rad = 0.4;
    let axis = Vec2::new(1.0, 1.0).normalize();
    let slot_anchor = Vec2::new(2.0, 2.0);
    let pin_slot = PinSlotJointBuilder::new(axis)
        .local_anchor1(slot_anchor)
        .local_anchor2(Vec2::new(0.0, rad))
        .limits([-1.0, f32::INFINITY]);
    // Start with the pin resting on the lower limit of the slot: a joint starting outside of its
    // limits is corrected almost instantly, which would launch the cube along the slot.
    let pin = character_pos + slot_anchor - axis;
    let cube = commands
        .spawn((
            Transform::from_xyz(pin.x, pin.y - rad, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad),
            ImpulseJoint::new(character, pin_slot),
        ))
        .id();

    /*
     * Small ball welded on top of the cube, to visualize its rotation.
     */
    let fixed = FixedJointBuilder::new().local_anchor2(Vec2::new(0.0, -rad));
    commands.spawn((
        Transform::from_xyz(pin.x, pin.y, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.1),
        ImpulseJoint::new(cube, fixed),
    ));
}

fn move_character(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut characters: Query<(
        &mut KinematicCharacterController,
        Option<&KinematicCharacterControllerOutput>,
        &mut Character,
    )>,
) {
    let dt = time.delta_secs();

    for (mut controller, output, mut character) in &mut characters {
        let mut horizontal = 0.0;
        if keys.any_pressed([KeyCode::ArrowLeft, KeyCode::KeyA]) {
            horizontal -= 1.0;
        }
        if keys.any_pressed([KeyCode::ArrowRight, KeyCode::KeyD]) {
            horizontal += 1.0;
        }

        // Keep a small downward motion while grounded so the controller keeps detecting the ground.
        if output.is_some_and(|output| output.grounded) {
            character.vertical_velocity = 0.0;
            if keys.just_pressed(KeyCode::Space) {
                character.vertical_velocity = JUMP_SPEED;
            }
        }
        character.vertical_velocity -= GRAVITY * dt;

        controller.translation =
            Some(Vec2::new(horizontal * CHARACTER_SPEED, character.vertical_velocity) * dt);
    }
}
