use bevy::diagnostic::{DiagnosticsStore, LogDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    // DOCUSAURUS: Diagnostics start
    App::new()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            // Measures the simulation of every context after each physics update (the
            // measurements are summed over the contexts), and of each context separately.
            RapierDiagnosticsPlugin::default().with_per_context_diagnostics(true),
            // Prints every registered diagnostic once per second.
            LogDiagnosticsPlugin::default(),
        ))
        // DOCUSAURUS: Diagnostics stop
        .add_systems(Startup, setup_physics)
        .add_systems(
            PostUpdate,
            (
                print_step_stats,
                print_context_step_times,
                handle_quarantine,
            )
                .after(PhysicsSet::Writeback),
        )
        .run();
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

// DOCUSAURUS: StepStats start
fn print_step_stats(
    contexts: Query<&RapierContextSimulation, With<DefaultRapierContext>>,
) -> Result {
    let stats = contexts.single()?.step_stats();
    if stats.num_steps > 0 {
        println!(
            "{} steps in {:.3}ms, {} contact pairs",
            stats.num_steps, stats.step_time_ms, stats.num_contact_pairs
        );
    }
    Ok(())
}
// DOCUSAURUS: StepStats stop

// DOCUSAURUS: PerContextDiagnostics start
fn print_context_step_times(
    diagnostics: Res<DiagnosticsStore>,
    contexts: Query<Entity, With<RapierContextSimulation>>,
) {
    for context in &contexts {
        // The path of the step time of this context only.
        let path =
            RapierDiagnosticsPlugin::context_path(&RapierDiagnosticsPlugin::STEP_TIME, context);
        if let Some(step_time) = diagnostics.get(&path).and_then(|d| d.smoothed()) {
            println!("The context {context} spends {step_time:.3}ms per physics update.");
        }
    }
}
// DOCUSAURUS: PerContextDiagnostics stop

// DOCUSAURUS: Quarantine start
fn handle_quarantine(
    mut commands: Commands,
    mut quarantine_events: MessageReader<PhysicsQuarantineEvent>,
    mut velocities: Query<&mut Velocity>,
) {
    for event in quarantine_events.read() {
        for entity in &event.bodies {
            println!("The rigid-body {entity} went non-finite and was disabled.");
            // Fix the cause, e.g., a non-finite velocity set by our own code.
            if let Ok(mut velocity) = velocities.get_mut(*entity) {
                *velocity = Velocity::zero();
            }
            // Then bring the rigid-body back into the simulation.
            commands.entity(*entity).remove::<RigidBodyDisabled>();
        }
        // The quarantined colliders and soft-bodies are re-enabled the same way.
        for entity in &event.colliders {
            commands.entity(*entity).remove::<ColliderDisabled>();
        }
        for entity in &event.soft_bodies {
            commands.entity(*entity).remove::<SoftBodyDisabled>();
        }
    }
}
// DOCUSAURUS: Quarantine stop
