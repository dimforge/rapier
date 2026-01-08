//! Testbed state types and flags.

use bitflags::bitflags;
use na::Point3;

#[cfg(feature = "dim3")]
use rapier::control::DynamicRayCastVehicleController;

use crate::harness::RapierBroadPhaseType;
use crate::physics::PhysicsSnapshot;
use crate::save::SerializableTestbedState;
use crate::settings::ExampleSettings;

/// Run mode for the simulation
#[derive(Default, PartialEq, Copy, Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum RunMode {
    Running,
    #[default]
    Stop,
    Step,
}

bitflags! {
    /// Flags for controlling what is displayed in the testbed
    #[derive(Copy, Clone, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
    pub struct TestbedStateFlags: u32 {
        const SLEEP = 1 << 0;
        const SUB_STEPPING = 1 << 1;
        const SHAPES = 1 << 2;
        const JOINTS = 1 << 3;
        const AABBS = 1 << 4;
        const CONTACT_POINTS = 1 << 5;
        const CONTACT_NORMALS = 1 << 6;
        const CENTER_OF_MASSES = 1 << 7;
        const WIREFRAME = 1 << 8;
        const STATISTICS = 1 << 9;
        const DRAW_SURFACES = 1 << 10;
    }
}

impl Default for TestbedStateFlags {
    fn default() -> Self {
        TestbedStateFlags::DRAW_SURFACES | TestbedStateFlags::SLEEP
    }
}

bitflags! {
    /// Flags for testbed actions that need to be processed
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    pub struct TestbedActionFlags: u32 {
        const RESET_WORLD_GRAPHICS = 1 << 0;
        const EXAMPLE_CHANGED = 1 << 1;
        const RESTART = 1 << 2;
        const BACKEND_CHANGED = 1 << 3;
        const TAKE_SNAPSHOT = 1 << 4;
        const RESTORE_SNAPSHOT = 1 << 5;
        const APP_STARTED = 1 << 6;
    }
}

pub(crate) const RAPIER_BACKEND: usize = 0;
pub(crate) const PHYSX_BACKEND_PATCH_FRICTION: usize = 1;
pub(crate) const PHYSX_BACKEND_TWO_FRICTION_DIR: usize = 2;

/// Which tab is currently selected in the UI
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum UiTab {
    #[default]
    Examples,
    Settings,
    Performance,
}

/// Information about an example for UI display
#[derive(Clone, Debug)]
pub struct ExampleEntry {
    pub name: &'static str,
    pub group: &'static str,
    /// Index in the original builders array
    pub builder_index: usize,
}

/// State for the testbed application
pub struct TestbedState {
    pub running: RunMode,
    pub draw_colls: bool,
    #[cfg(feature = "dim3")]
    pub vehicle_controller: Option<DynamicRayCastVehicleController>,
    pub grabbed_object_plane: (Point3<f32>, na::Vector3<f32>),
    pub can_grab_behind_ground: bool,
    pub drawing_ray: Option<na::Point2<f32>>,
    pub prev_flags: TestbedStateFlags,
    pub flags: TestbedStateFlags,
    pub action_flags: TestbedActionFlags,
    pub backend_names: Vec<&'static str>,
    /// Examples in display order (grouped, then by original order within group)
    pub examples: Vec<ExampleEntry>,
    /// Unique group names in order of first appearance
    pub example_groups: Vec<&'static str>,
    /// Currently selected position in the display order
    pub selected_display_index: usize,
    pub selected_backend: usize,
    pub example_settings: ExampleSettings,
    pub broad_phase_type: RapierBroadPhaseType,
    pub physx_use_two_friction_directions: bool,
    pub snapshot: Option<PhysicsSnapshot>,
    pub nsteps: usize,
    pub camera_locked: bool,
    pub selected_tab: UiTab,
    pub prev_save_data: SerializableTestbedState,
}

impl Default for TestbedState {
    fn default() -> Self {
        #[allow(unused_mut)]
        let mut backend_names = vec!["rapier"];
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (patch friction)");
        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        backend_names.push("physx (two friction dir)");

        let flags = TestbedStateFlags::default();
        Self {
            running: RunMode::Running,
            draw_colls: false,
            #[cfg(feature = "dim3")]
            vehicle_controller: None,
            grabbed_object_plane: (Point3::origin(), na::zero()),
            can_grab_behind_ground: false,
            drawing_ray: None,
            snapshot: None,
            prev_flags: flags,
            flags,
            action_flags: TestbedActionFlags::APP_STARTED | TestbedActionFlags::EXAMPLE_CHANGED,
            backend_names,
            examples: Vec::new(),
            example_groups: Vec::new(),
            example_settings: ExampleSettings::default(),
            selected_display_index: 0,
            selected_backend: RAPIER_BACKEND,
            broad_phase_type: RapierBroadPhaseType::default(),
            physx_use_two_friction_directions: true,
            nsteps: 1,
            camera_locked: false,
            selected_tab: UiTab::default(),
            prev_save_data: SerializableTestbedState::default(),
        }
    }
}

impl TestbedState {
    /// Get the builder index for the currently selected example
    pub fn selected_builder_index(&self) -> usize {
        self.examples
            .get(self.selected_display_index)
            .map(|e| e.builder_index)
            .unwrap_or(0)
    }
}
