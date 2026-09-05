use crate::settings::ExampleSettings;
use crate::testbed::{RunMode, TestbedStateFlags};
use crate::{Camera, TestbedState};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Debug, Default, Clone)]
pub struct SerializableTestbedState {
    pub running: RunMode,
    pub flags: TestbedStateFlags,
    pub selected_example: usize,
    pub example_settings: ExampleSettings,
    pub soft_recovery: rapier::dynamics::SoftRecoverySettings,
    pub camera: Camera,
}

impl TestbedState {
    pub fn save_data(&self, camera: Camera) -> SerializableTestbedState {
        SerializableTestbedState {
            running: self.running,
            flags: self.flags,
            selected_example: self.selected_display_index,
            example_settings: self.example_settings.clone(),
            soft_recovery: self.soft_recovery,
            camera,
        }
    }

    pub fn apply_saved_data(&mut self, state: SerializableTestbedState, camera: &mut Camera) {
        self.prev_save_data = state.clone();
        self.running = state.running;
        self.flags = state.flags;
        self.selected_display_index = state.selected_example;
        self.example_settings = state.example_settings;
        self.soft_recovery = state.soft_recovery;
        *camera = state.camera;
    }
}
