use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // DOCUSAURUS: TimestepMode start
        // Advance the simulation by exactly 1/60 seconds (in two substeps of 1/120 seconds) at
        // each update of the schedule running the physics.
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 2,
        })
        // DOCUSAURUS: TimestepMode stop
        // DOCUSAURUS: ContextInitialization start
        .add_plugins(
            RapierPhysicsPlugin::<NoUserData>::default().with_custom_initialization(
                RapierContextInitialization::InitializeDefaultRapierContext {
                    integration_parameters: IntegrationParameters {
                        num_solver_iterations: 8,
                        ..default()
                    },
                    rapier_configuration: RapierConfiguration {
                        gravity: Vec3::new(0.0, -3.72, 0.0),
                        ..RapierConfiguration::new(1.0)
                    },
                    broad_phase_optimization_strategy:
                        BroadPhaseOptimizationStrategy::SubtreeOptimizer,
                },
            ),
        )
        // DOCUSAURUS: ContextInitialization stop
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (
                modify_integration_parameters.run_if(input_just_pressed(KeyCode::KeyI)),
                modify_configuration.run_if(input_just_pressed(KeyCode::KeyC)),
                toggle_pause.run_if(input_just_pressed(KeyCode::KeyP)),
            ),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -2.0, 0.0),
        Collider::cuboid(100.0, 0.1, 100.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 4.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.5),
    ));
}

// DOCUSAURUS: IntegrationParameters start
fn modify_integration_parameters(
    mut contexts: Query<&mut RapierContextSimulation, With<DefaultRapierContext>>,
) -> Result {
    let mut simulation = contexts.single_mut()?;
    simulation.integration_parameters.num_solver_iterations = 12;
    simulation.integration_parameters.warmstart_joints = true;
    Ok(())
}
// DOCUSAURUS: IntegrationParameters stop

// DOCUSAURUS: RapierConfiguration start
fn modify_configuration(
    mut configurations: Query<&mut RapierConfiguration, With<DefaultRapierContext>>,
) -> Result {
    let mut configuration = configurations.single_mut()?;
    configuration.gravity = Vec3::new(0.0, -9.81, 0.0);
    // Only detect collisions: no forces, joints, or contact responses.
    configuration.simulation_mode = SimulationMode::CollisionOnly;
    // Run the simulation of this context on its own pool of 4 threads (needs the
    // `parallel` feature).
    configuration.num_threads = Some(4);
    Ok(())
}
// DOCUSAURUS: RapierConfiguration stop

// DOCUSAURUS: Pause start
fn toggle_pause(mut configurations: Query<&mut RapierConfiguration>) {
    for mut configuration in configurations.iter_mut() {
        configuration.physics_pipeline_active = !configuration.physics_pipeline_active;
    }
}
// DOCUSAURUS: Pause stop
