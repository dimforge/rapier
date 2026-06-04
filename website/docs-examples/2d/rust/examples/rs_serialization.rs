use rapier2d::prelude::*;

// DOCUSAURUS: Serialization start
use serde::{Deserialize, Serialize};

// Use serde derives for our wrapper struct.
#[derive(Serialize, Deserialize)]
struct PhysicsState {
    pub islands: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub integration_parameters: IntegrationParameters,
    pub gravity: Vector<f32>,
}

fn main() {
    let physics_state = setup_physics_scene();
    // Serialize everything.
    let serialized =
        bincode::serde::encode_to_vec(&physics_state, bincode::config::standard()).unwrap();
    // Deserialize everything.
    let deserialized = bincode::serde::decode_from_slice::<PhysicsState, _>(
        &serialized,
        bincode::config::standard(),
    )
    .unwrap();
    // The simulation can continue using the deserialized state.
}
// DOCUSAURUS: Serialization stop

/// Adapted from basic_sim example from rapier user guide.
fn setup_physics_scene() -> PhysicsState {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    collider_set.insert(collider);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        let ball_body = &rigid_body_set[ball_body_handle];
        println!("Ball altitude: {}", ball_body.translation().y);
    }
    PhysicsState {
        islands: IslandManager::new(),
        broad_phase,
        narrow_phase,
        bodies: rigid_body_set,
        colliders: collider_set,
        impulse_joints: impulse_joint_set,
        multibody_joints: multibody_joint_set,
        ccd_solver,
        integration_parameters,
        gravity,
    }
}
