use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

#[derive(serde::Deserialize)]
struct PhysicsState {
    pub gravity: Vector,
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
                if rb.linvel().length() != 0.0 {
                    println!("\tlinvel: {:?}", rb.linvel());
                }
            }

            let mut world = PhysicsWorld::new();
            world.bodies = state.bodies;
            world.colliders = state.colliders;
            world.impulse_joints = state.impulse_joints;
            world.multibody_joints = state.multibody_joints;
            world.islands = state.islands;
            world.broad_phase = state.broad_phase;
            world.narrow_phase = state.narrow_phase;
            world.integration_parameters = state.integration_parameters;
            world.gravity = state.gravity;
            testbed.set_physics_world(world);

            testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));
        }
        Err(err) => println!("Failed to deserialize the world state: {err}"),
    }
}
