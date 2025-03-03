use rapier::prelude::PhysicsContext;

use crate::harness::RunState;
use crate::physics::PhysicsEvents;

pub trait HarnessPlugin {
    fn run_callbacks(
        &mut self,
        physics: &mut PhysicsContext,
        events: &PhysicsEvents,
        harness_state: &RunState,
    );
    fn step(&mut self, physics: &mut PhysicsContext, run_state: &RunState);
    fn profiling_string(&self) -> String;
}
