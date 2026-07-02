use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

#[derive(serde::Deserialize)]
struct State {
    pub islands: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let Ok(bytes) = std::fs::read("state.bin") else {
        println!("Failed to load serialized world state.");
        return Ok(());
    };
    let state: State = bincode::deserialize(&bytes).unwrap();

    let mut world = PhysicsWorld::new();
    world.bodies = state.bodies;
    world.colliders = state.colliders;
    world.impulse_joints = state.impulse_joints;
    world.multibody_joints = state.multibody_joints;

    viewer.set_world(&mut world);

    // Preserve the deserialized acceleration structures (set_world resets the
    // broad-phase to the UI-selected default).
    world.islands = state.islands;
    world.broad_phase = state.broad_phase;
    world.narrow_phase = state.narrow_phase;
    world.ccd_solver = state.ccd_solver;

    viewer.set_graphics_shift([-541.0, -6377257.0, -61.0].into());
    viewer.look_at([10.0, 10.0, 10.0].into(), [0.0, 0.0, 0.0].into());

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }

    Ok(())
}
