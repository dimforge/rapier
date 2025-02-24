#[cfg(feature = "dim2")]
use crate::camera2d::OrbitCamera;
#[cfg(feature = "dim3")]
use crate::camera3d::OrbitCamera;
use crate::settings::ExampleSettings;
use crate::testbed::{RapierSolverType, RunMode, TestbedStateFlags};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableTestbedState {
    pub running: RunMode,
    pub flags: TestbedStateFlags,
    pub selected_example: usize,
    pub selected_backend: usize,
    pub example_settings: ExampleSettings,
    pub solver_type: RapierSolverType,
    pub physx_use_two_friction_directions: bool,
    pub camera: OrbitCamera,
}

#[cfg(feature = "dim2")]
#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableCameraState {
    pub zoom: f32,
    pub center: na::Point2<f32>,
}

#[cfg(feature = "dim3")]
#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableCameraState {
    pub distance: f32,
    pub position: na::Point3<f32>,
    pub center: na::Point3<f32>,
}
