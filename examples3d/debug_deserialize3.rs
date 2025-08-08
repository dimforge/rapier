use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

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
    // Deserialize
    let setting = testbed.example_settings_mut();
    let frame_id = setting.get_or_set_u32("frame", 0, 0..=1400);
    let frame_dirs = "/Users/sebcrozet/work/hytopia/sdk/examples/bug-demo";
    let path = format!("{frame_dirs}/snapshot{frame_id}.bincode");
    let bytes = match std::fs::read(&path) {
        Ok(bytes) => bytes,
        Err(err) => {
            println!("Failed to open the serialized scene file {path:?}: {err}");
            return;
        }
    };
    match bincode::deserialize(&bytes) {
        Ok(state) => {
            let state: PhysicsState = state;
            println!("World state deserialized successfully:");
            println!("\tgravity: {:?}", state.gravity);
            println!(
                "\tintegration parameters: {:?}",
                state.integration_parameters
            );
            println!("\tbodies: {:?}", state.bodies.len());
            println!("\tcolliders: {:?}", state.colliders.len());
            println!("\timpulse_joints: {:?}", state.impulse_joints.len());

            for (_, rb) in state.bodies.iter() {
                if rb.linvel().norm() != 0.0 {
                    println!("\tlinvel: {:?}", rb.linvel());
                }
            }

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

            testbed.look_at(point![10.0, 10.0, 10.0], point![0.0, 0.0, 0.0]);
        }
        Err(err) => println!("Failed to deserialize the world state: {err}"),
    }
}
