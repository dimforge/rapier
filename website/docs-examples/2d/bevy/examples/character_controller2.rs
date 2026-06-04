use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, setup_physics_more)
        .add_systems(FixedUpdate, update_system)
        .add_systems(FixedUpdate, read_result_system)
        .add_systems(FixedUpdate, modify_character_controller_slopes)
        .add_systems(FixedUpdate, modify_character_controller_autostep)
        .add_systems(FixedUpdate, modify_character_controller_snap_to_ground)
        .add_systems(FixedUpdate, read_character_controller_collisions)
        .add_systems(FixedUpdate, modify_character_controller_impulses)
        .add_systems(
            Update,
            modify_character_controller_up.run_if(input_just_pressed(KeyCode::KeyG)),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics_more(mut commands: Commands) {
    // DOCUSAURUS: Offset start
    /* Configure the character controller when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            // The character offset is set to 0.01.
            offset: CharacterLength::Absolute(0.01),
            ..default()
        });

    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            // The character offset is set to 0.01 multiplied by the collider’s height.
            offset: CharacterLength::Relative(0.01),
            ..default()
        });
    // DOCUSAURUS: Offset stop

    // DOCUSAURUS: UpVector1 start
    /* Character controller with the positive X axis as the up vector. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            up: Vec2::X,
            ..default()
        });
    // DOCUSAURUS: UpVector1 stop

    // DOCUSAURUS: Slopes1 start
    /* Configure the character controller when the collider is created. */
    // Snap to the ground if the vertical distance to the ground is smaller than 0.5.
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            // Don’t allow climbing slopes larger than 45 degrees.
            max_slope_climb_angle: 45_f32.to_radians(),
            // Automatically slide down on slopes smaller than 30 degrees.
            min_slope_slide_angle: 30_f32.to_radians(),
            ..default()
        });
    // DOCUSAURUS: Slopes1 stop

    // DOCUSAURUS: Stairs1 start
    /* Configure the character controller when the collider is created. */
    // Autostep if the step height is smaller than 0.5, and its width larger than 0.2.
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            autostep: Some(CharacterAutostep {
                max_height: CharacterLength::Absolute(0.5),
                min_width: CharacterLength::Absolute(0.2),
                include_dynamic_bodies: true,
            }),
            ..default()
        });

    // Autostep if the step height is smaller than 0.5 multiplied by the character’s height,
    // and its width larger than 0.5 multiplied by the character’s width (i.e. half the character’s
    // width).
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            autostep: Some(CharacterAutostep {
                max_height: CharacterLength::Relative(0.3),
                min_width: CharacterLength::Relative(0.5),
                include_dynamic_bodies: true,
            }),
            ..default()
        });
    // DOCUSAURUS: Stairs1 stop

    // DOCUSAURUS: Snap1 start
    /* Configure the character controller when the collider is created. */
    // Snap to the ground if the vertical distance to the ground is smaller than 0.5.
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            snap_to_ground: Some(CharacterLength::Absolute(0.5)),
            ..default()
        });

    // Snap to the ground if the vertical distance to the ground is smaller than 0.2 times the character’s height
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            snap_to_ground: Some(CharacterLength::Relative(0.2)),
            ..default()
        });
    // DOCUSAURUS: Snap1 stop

    // DOCUSAURUS: Collisions2 start
    /* Configure the character controller when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            // Enable the automatic application of impulses to the dynamic bodies
            // hit by the character along its path.
            apply_impulse_to_dynamic_bodies: true,
            ..default()
        });
    // DOCUSAURUS: Collisions2 stop
}

// DOCUSAURUS: Setup start
fn setup_physics(mut commands: Commands) {
    commands
        .spawn(RigidBody::KinematicPositionBased)
        .insert(Collider::ball(0.5))
        .insert(KinematicCharacterController::default());
}

fn update_system(mut controllers: Query<&mut KinematicCharacterController>) {
    for mut controller in controllers.iter_mut() {
        controller.translation = Some(Vec2::new(1.0, -0.5));
    }
}

fn read_result_system(controllers: Query<(Entity, &KinematicCharacterControllerOutput)>) {
    for (entity, output) in controllers.iter() {
        println!(
            "Entity {:?} moved by {:?} and touches the ground: {:?}",
            entity, output.effective_translation, output.grounded
        );
    }
}
// DOCUSAURUS: Setup stop

// DOCUSAURUS: UpVector2 start
/* Modify the character controller’s up vector inside of a system. */
fn modify_character_controller_up(
    mut character_controllers: Query<&mut KinematicCharacterController>,
) {
    for mut character_controller in character_controllers.iter_mut() {
        character_controller.up = Vec2::X;
    }
}
// DOCUSAURUS: UpVector2 stop

// DOCUSAURUS: Slopes2 start
/* Configure snap-to-ground inside of a system. */
fn modify_character_controller_slopes(
    mut character_controllers: Query<&mut KinematicCharacterController>,
) {
    for mut character_controller in character_controllers.iter_mut() {
        // Don’t allow climbing slopes larger than 45 degrees.
        character_controller.max_slope_climb_angle = 45_f32.to_radians();
        // Automatically slide down on slopes smaller than 30 degrees.
        character_controller.min_slope_slide_angle = 30_f32.to_radians();
    }
}
// DOCUSAURUS: Slopes2 stop

// DOCUSAURUS: Stairs2 start
/* Configure autostep inside of a system. */
fn modify_character_controller_autostep(
    mut character_controllers: Query<&mut KinematicCharacterController>,
) {
    for mut character_controller in character_controllers.iter_mut() {
        character_controller.autostep = Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(0.5),
            min_width: CharacterLength::Absolute(0.2),
            include_dynamic_bodies: true,
        });
    }
}
// DOCUSAURUS: Stairs2 stop

// DOCUSAURUS: Snap2 start
/* Configure snap-to-ground inside of a system. */
fn modify_character_controller_snap_to_ground(
    mut character_controllers: Query<&mut KinematicCharacterController>,
) {
    for mut character_controller in character_controllers.iter_mut() {
        character_controller.snap_to_ground = Some(CharacterLength::Absolute(0.5));
    }
}
// DOCUSAURUS: Snap2 stop

// DOCUSAURUS: Collisions1 start
/* Read the character controller collisions stored in the character controller’s output. */
fn read_character_controller_collisions(
    mut character_controller_outputs: Query<&mut KinematicCharacterControllerOutput>,
) {
    for mut output in character_controller_outputs.iter_mut() {
        for collision in &output.collisions {
            // Do something with that collision information.
        }
    }
}
// DOCUSAURUS: Collisions1 stop

// DOCUSAURUS: Collisions3 start
/* Configure dynamic impulses inside of a system. */
fn modify_character_controller_impulses(
    mut character_controllers: Query<&mut KinematicCharacterController>,
) {
    for mut character_controller in character_controllers.iter_mut() {
        // Enable the automatic application of impulses to the dynamic bodies
        // hit by the character along its path.
        character_controller.apply_impulse_to_dynamic_bodies = true;
    }
}
// DOCUSAURUS: Collisions3 stop
