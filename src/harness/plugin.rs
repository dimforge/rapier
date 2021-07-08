use crate::harness::RunState;
use crate::physics::{PhysicsEvents, PhysicsState};

/// Crates that want to hook into the harness simulation step runtime, will use this
pub trait HarnessPlugin {
    /// This is called at each step in the simulation, to allow client code to run their own callbacks
    fn run_callbacks(
        &mut self,
        physics: &mut PhysicsState,
        events: &PhysicsEvents,
        run_state: &RunState,
    );

    /// This is called at each step in the simulation, to allow client code to run their own physics etc.
    fn step(&mut self, physics: &mut PhysicsState, run_state: &RunState);

    /// Profiling string
    fn profiling_string(&self) -> String;
}
