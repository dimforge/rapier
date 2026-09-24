use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

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
                        // 100 pixels make one meter.
                        length_unit: 100.0,
                        num_solver_iterations: 8,
                        ..default()
                    },
                    rapier_configuration: RapierConfiguration {
                        gravity: Vec2::new(0.0, -372.0),
                        ..RapierConfiguration::new(100.0)
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
        .add_systems(PostUpdate, print_active_bodies.after(PhysicsSet::Writeback))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2d);
}

fn setup_physics(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -100.0, 0.0),
        Collider::cuboid(500.0, 50.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(50.0),
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
    configuration.gravity = Vec2::new(0.0, -981.0);
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

// DOCUSAURUS: ActiveBodies start
fn print_active_bodies(context: ReadRapierContext, transforms: Query<&Transform>) -> Result {
    let context = context.single()?;
    // Iter on each rigid-body that moved (dynamic and kinematic).
    for handle in context.simulation.islands.active_bodies() {
        let Some(entity) = context.rigidbody_set.rigid_body_entity(handle) else {
            continue;
        };
        if let Ok(transform) = transforms.get(entity) {
            println!("Rigid body {entity} has a new position: {}", transform.translation);
        }
    }
    Ok(())
}
// DOCUSAURUS: ActiveBodies stop
