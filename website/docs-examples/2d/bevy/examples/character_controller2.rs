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
        .add_systems(FixedUpdate, move_character_manually)
        .add_systems(Update, update_doors)
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

    // Autostep if the step height is smaller than 0.3 multiplied by the character’s height,
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

    let platform = commands.spawn(Collider::cuboid(2.0, 0.1)).id();

    // DOCUSAURUS: Filtering start
    /* Configure the character controller filters when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(KinematicCharacterController {
            // Ignore all the sensors and all the colliders attached to dynamic rigid-bodies.
            filter_flags: QueryFilterFlags::EXCLUDE_SENSORS | QueryFilterFlags::EXCLUDE_DYNAMIC,
            // The character is part of the group 1 and only interacts with the group 2.
            filter_groups: Some(CollisionGroups::new(Group::GROUP_1, Group::GROUP_2)),
            // Ignore the collider attached to the `platform` entity.
            exclude_colliders: [platform].into_iter().collect(),
            // Ignore the colliders with a ball shape.
            filter_predicate: Some(ControllerFilterPredicate::new(|_entity, collider| {
                collider.shape().as_ball().is_none()
            })),
            ..default()
        });
    // DOCUSAURUS: Filtering stop

    commands.spawn((Collider::cuboid(0.1, 1.0), Door { open: true }));
    commands.spawn((
        Collider::ball(0.5),
        Transform::from_xyz(0.0, 2.0, 0.0),
        ManualCharacter,
    ));
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

// DOCUSAURUS: MoveShape start
/// Marks a character moved without the `KinematicCharacterController` component.
#[derive(Component)]
struct ManualCharacter;

fn move_character_manually(
    mut context: WriteRapierContext,
    mut characters: Query<(Entity, &Collider, &mut Transform), With<ManualCharacter>>,
) -> Result {
    let mut context = context.single_mut()?;
    for (entity, collider, mut transform) in characters.iter_mut() {
        // The translation we would like to apply if there were no obstacles.
        let desired_translation = Vec2::new(1.0, -0.5);
        // Configure the controller like with the `KinematicCharacterController` component.
        let options = MoveShapeOptions {
            snap_to_ground: Some(CharacterLength::Absolute(0.5)),
            ..default()
        };
        // Make sure the character we are trying to move isn’t considered an obstacle.
        let filter = QueryFilter::default().exclude_collider(entity);
        // Calculate the possible movement.
        let output = context.move_shape(
            desired_translation,
            collider,                                     // The character’s shape.
            transform.translation.truncate(),             // The character’s initial position.
            transform.rotation.to_euler(EulerRot::ZYX).0, // The character’s rotation.
            1.0, // The character’s mass, for the impulses applied to dynamic bodies.
            &options,
            filter,
            |collision| println!("The character hit the entity {:?}.", collision.entity),
        );
        // The movement isn’t applied automatically.
        transform.translation += output.effective_translation.extend(0.0);
    }
    Ok(())
}
// DOCUSAURUS: MoveShape stop

// DOCUSAURUS: ControllerIgnored start
/// A door the characters can only walk through while it is open.
#[derive(Component)]
struct Door {
    open: bool,
}

/* Hide the open doors from every controller inside of a system. */
fn update_doors(mut commands: Commands, doors: Query<(Entity, &Door), Changed<Door>>) {
    for (entity, door) in doors.iter() {
        if door.open {
            commands.entity(entity).insert(ControllerIgnored);
        } else {
            commands.entity(entity).remove::<ControllerIgnored>();
        }
    }
}
// DOCUSAURUS: ControllerIgnored stop

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
/* Configure the slopes inside of a system. */
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
    character_controller_outputs: Query<&KinematicCharacterControllerOutput>,
) {
    for output in character_controller_outputs.iter() {
        for collision in &output.collisions {
            // Do something with that collision information.
            println!(
                "The character hit the entity {:?} after moving by {}.",
                collision.entity, collision.translation_applied
            );
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
