use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

#[derive(serde::Deserialize)]
struct PhysicsState {
    pub gravity: Vector<f32>,
    pub integration_parameters: IntegrationParameters,
    pub islands: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Set up the testbed.
     */
    let path = "state.bin";
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(err) => {
            println!(
                "Failed to open the serialzed scene file {:?}: {}",
                path, err
            );
            return;
        }
    };
    match bincode::deserialize(&bytes) {
        Ok(state) => {
            let state: PhysicsState = state;
            testbed.set_world(
                state.bodies,
                state.colliders,
                state.impulse_joints,
                state.multibody_joints,
            );
            testbed.harness_mut().physics.islands = state.islands;
            testbed.harness_mut().physics.broad_phase = state.broad_phase;
            testbed.harness_mut().physics.narrow_phase = state.narrow_phase;
            testbed.harness_mut().physics.integration_parameters = state.integration_parameters;
            testbed.harness_mut().physics.gravity = state.gravity;

            testbed.set_graphics_shift(vector![-541.0, -6377257.0, -61.0]);
            testbed.look_at(point![10.0, 10.0, 10.0], point![0.0, 0.0, 0.0]);
        }
        Err(err) => println!("Failed to deserialize the world state: {}", err),
    }
}
