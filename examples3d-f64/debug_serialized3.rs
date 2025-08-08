use rapier_testbed3d::Testbed;
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

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Set up the testbed.
     */
    let bytes = std::fs::read("state.bin").unwrap();
    let state: State = bincode::deserialize(&bytes).unwrap();

    testbed.set_world(
        state.bodies,
        state.colliders,
        state.impulse_joints,
        state.multibody_joints,
    );
    testbed.harness_mut().physics.islands = state.islands;
    testbed.harness_mut().physics.broad_phase = state.broad_phase;
    testbed.harness_mut().physics.narrow_phase = state.narrow_phase;
    testbed.harness_mut().physics.ccd_solver = state.ccd_solver;

    testbed.set_graphics_shift(vector![-541.0, -6377257.0, -61.0]);
    testbed.look_at(point![10.0, 10.0, 10.0], point![0.0, 0.0, 0.0]);
}
