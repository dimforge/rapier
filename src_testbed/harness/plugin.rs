use crate::PhysicsState;
use crate::harness::RunState;
use crate::physics::PhysicsEvents;

pub trait HarnessPlugin {
    fn run_callbacks(
        &mut self,
        physics: &mut PhysicsState,
        events: &PhysicsEvents,
        harness_state: &RunState,
    );
    fn step(&mut self, physics: &mut PhysicsState, run_state: &RunState);
    fn profiling_string(&self) -> String;
}
