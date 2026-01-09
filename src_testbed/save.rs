use crate::settings::ExampleSettings;
use crate::testbed::{RunMode, TestbedStateFlags};
use crate::{Camera, TestbedState};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableTestbedState {
    pub running: RunMode,
    pub flags: TestbedStateFlags,
    pub selected_example: usize,
    pub selected_backend: usize,
    pub example_settings: ExampleSettings,
    pub physx_use_two_friction_directions: bool,
    pub camera: Camera,
}

impl TestbedState {
    pub fn save_data(&self, camera: Camera) -> SerializableTestbedState {
        SerializableTestbedState {
            running: self.running,
            flags: self.flags,
            selected_example: self.selected_display_index,
            selected_backend: self.selected_backend,
            example_settings: self.example_settings.clone(),
            physx_use_two_friction_directions: self.physx_use_two_friction_directions,
            camera,
        }
    }

    pub fn apply_saved_data(&mut self, state: SerializableTestbedState, camera: &mut Camera) {
        self.prev_save_data = state.clone();
        self.running = state.running;
        self.flags = state.flags;
        self.selected_display_index = state.selected_example;
        self.selected_backend = state.selected_backend;
        self.example_settings = state.example_settings;
        self.physx_use_two_friction_directions = state.physx_use_two_friction_directions;
        *camera = state.camera;
    }
}
