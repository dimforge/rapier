use crate::harness::HarnessState;
use crate::physics::PhysicsEvents;
use crate::PhysicsState;

pub trait HarnessPlugin {
    fn run_callbacks(
        &mut self,
        physics: &mut PhysicsState,
        events: &PhysicsEvents,
        harness_state: &HarnessState,
        t: f32,
    );
    fn step(&mut self, physics: &mut PhysicsState);
    fn profiling_string(&self) -> String;
}
