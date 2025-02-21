use serde::{Deserialize, Serialize};
use crate::settings::ExampleSettings;
use crate::testbed::{RapierSolverType, RunMode, TestbedStateFlags};

#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableTestbedState {
    pub running: RunMode,
    pub flags: TestbedStateFlags,
    pub selected_example: usize,
    pub selected_backend: usize,
    pub example_settings: ExampleSettings,
    pub solver_type: RapierSolverType,
    pub physx_use_two_friction_directions: bool,
}