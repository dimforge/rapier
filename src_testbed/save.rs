#[cfg(feature = "dim2")]
use crate::camera2d::OrbitCamera;
#[cfg(feature = "dim3")]
use crate::camera3d::OrbitCamera;
use crate::settings::ExampleSettings;
use crate::testbed::{RunMode, TestbedStateFlags};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableTestbedState {
    pub running: RunMode,
    pub flags: TestbedStateFlags,
    pub selected_example: usize,
    pub selected_backend: usize,
    pub example_settings: ExampleSettings,
    pub physx_use_two_friction_directions: bool,
    pub camera: OrbitCamera,
}
