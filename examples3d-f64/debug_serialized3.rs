use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

#[derive(serde::Deserialize)]
struct State {
    pub islands: IslandManager,
    pub broad_phase: BroadPhase,
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
    let mut state: State = bincode::deserialize(&bytes).unwrap();

    for body in state.bodies.iter_mut() {
        dbg!(body.1.position());
        dbg!(body.1.is_ccd_enabled());
        dbg!(body.1.is_sleeping());
        // dbg!(body.1);
        body.1.clear_forces(false);
    }

    let mut to_remove = vec![];
    for (_, co) in state.colliders.iter() {
        if co.shape().as_ball().is_none() {
            if let Some(parent) = co.parent() {
                let body = &state.bodies[parent];
                if body.is_dynamic() {
                    to_remove.push(parent);
                }
            }
        }
    }

    // for h in to_remove {
    //     state.bodies.remove(
    //         h,
    //         &mut state.islands,
    //         &mut state.colliders,
    //         &mut state.impulse_joints,
    //         &mut state.multibody_joints,
    //     );
    // }

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
    // testbed.harness_mut().physics.integration_parameters.erp = 0.0;
    // testbed
    //     .harness_mut()
    //     .physics
    //     .integration_parameters
    //     .delassus_inv_factor = 1.0;

    testbed.set_graphics_shift(vector![-541.0, -6377257.0, -61.0]);
    testbed.look_at(point![10.0, 10.0, 10.0], point![0.0, 0.0, 0.0]);
}
