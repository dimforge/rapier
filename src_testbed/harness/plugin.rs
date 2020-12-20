use crate::harness::HarnessState;
use crate::PhysicsState;
use crate::physics::PhysicsEvents;

pub trait HarnessPlugin {
    //FIXME: is run_callbacks needed?
    fn run_callbacks(&mut self, physics: &mut PhysicsState, events: &PhysicsEvents, harness_state: &HarnessState, t: f32);
    fn step(&mut self, physics: &mut PhysicsState);
    fn profiling_string(&self) -> String;
}