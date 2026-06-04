use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, setup_physics_more)
        .add_systems(FixedUpdate, update_system)
        .add_systems(FixedUpdate, read_result_system)
        .add_systems(
            Update,
            modify_character_controller_up.run_if(input_just_pressed(KeyCode::KeyG)),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics_more(mut commands: Commands) {
    /* Create the ground. */
    commands
        .spawn(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(Transform::from_xyz(0.0, -0.9, 0.0));

    // DOCUSAURUS: UpVector1 start
    /* Character controller with the positive X axis as the up vector. */
    commands
        .spawn(RigidBody::KinematicPositionBased)
        .insert(Collider::ball(0.5))
        .insert(Transform::default().with_translation(Vec3::Z * -10f32))
        .insert(KinematicCharacterController {
            up: Vec3::X,
            ..default()
        });
    // DOCUSAURUS: UpVector1 stop
}

// DOCUSAURUS: Setup start
fn setup_physics(mut commands: Commands) {
    commands
        .spawn(RigidBody::KinematicPositionBased)
        .insert(Collider::ball(0.5))
        .insert(Transform::default())
        .insert(KinematicCharacterController {
            ..KinematicCharacterController::default()
        });
}

fn update_system(time: Res<Time>, mut controllers: Query<&mut KinematicCharacterController>) {
    for mut controller in controllers.iter_mut() {
        controller.translation = Some(Vec3::new(1.0, -5.0, -1.0) * time.delta_secs());
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
        character_controller.up = Vec3::Y;
    }
}
// DOCUSAURUS: UpVector2 stop
