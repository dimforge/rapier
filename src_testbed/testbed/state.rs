//! Testbed state types and flags.

use bitflags::bitflags;

use crate::physics::{PhysicsSnapshot, RapierBroadPhaseType};
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

/// A loop transition requested from the UI: stop entirely, or switch to another
/// example (or re-run the current one). The target is carried in
/// [`TestbedState::selected_display_index`]; this only signals the
/// example-owned `while viewer.render_frame()` loop to exit so the outer demo
/// runner can dispatch the next example.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Transition {
    Quit,
    Switch,
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
    /// Flags for in-frame testbed actions applied to the borrowed world.
    ///
    /// Example switching / restart / backend changes are no longer flags — they
    /// are handled by [`Transition`], which makes the example's render loop exit
    /// so the outer demo runner re-dispatches.
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    pub struct TestbedActionFlags: u32 {
        const RESET_WORLD_GRAPHICS = 1 << 0;
        const TAKE_SNAPSHOT = 1 << 4;
        const RESTORE_SNAPSHOT = 1 << 5;
        const APP_STARTED = 1 << 6;
        /// Recenter the camera so the entire scene is visible and fills
        /// the viewport.
        const FRAME_SCENE = 1 << 7;
    }
}

/// Which tab is currently selected in the UI
#[derive(Default, Copy, Clone, PartialEq, Eq, Debug)]
pub enum UiTab {
    #[default]
    Examples,
    Settings,
    Performance,
}

/// Information about an example for UI display.
#[derive(Clone, Debug)]
pub struct ExampleEntry {
    pub name: &'static str,
    pub group: &'static str,
}

impl ExampleEntry {
    pub fn new(group: &'static str, name: &'static str) -> Self {
        Self { name, group }
    }
}

/// State for the testbed application
pub struct TestbedState {
    pub running: RunMode,
    pub can_grab_behind_ground: bool,
    pub prev_flags: TestbedStateFlags,
    pub flags: TestbedStateFlags,
    pub action_flags: TestbedActionFlags,
    /// Pending loop transition (example switch / quit) requested from the UI.
    pub transition: Option<Transition>,
    /// `true` while a restart / solver-parameter change is the reason for a
    /// pending [`Transition::Switch`]; such switches preserve the user's
    /// example-setting edits, whereas selecting a different example clears them.
    pub preserve_settings_on_switch: bool,
    /// Examples in display order (grouped, then by original order within group)
    pub examples: Vec<ExampleEntry>,
    /// Unique group names in order of first appearance
    pub example_groups: Vec<&'static str>,
    /// Currently selected position in the display order
    pub selected_display_index: usize,
    pub example_settings: ExampleSettings,
    pub broad_phase_type: RapierBroadPhaseType,
    pub snapshot: Option<PhysicsSnapshot>,
    pub camera_locked: bool,
    pub selected_tab: UiTab,
    pub prev_save_data: SerializableTestbedState,
    /// Unit up-vector kept in sync with the camera (see
    /// [`crate::TestbedViewer::set_up_axis`]). The gravity slider in the
    /// testbed UI reads this so it can keep gravity aligned with "down"
    /// (`-up_axis`) instead of the hard-coded Y-axis it used to assume.
    /// Defaults to `Vector::Y`.
    pub up_axis: rapier::math::Vector,
}

impl Default for TestbedState {
    fn default() -> Self {
        let flags = TestbedStateFlags::default();
        Self {
            running: RunMode::Running,
            can_grab_behind_ground: false,
            snapshot: None,
            prev_flags: flags,
            flags,
            action_flags: TestbedActionFlags::APP_STARTED,
            transition: None,
            preserve_settings_on_switch: false,
            examples: Vec::new(),
            example_groups: Vec::new(),
            example_settings: ExampleSettings::default(),
            selected_display_index: 0,
            broad_phase_type: RapierBroadPhaseType::default(),
            camera_locked: false,
            selected_tab: UiTab::default(),
            prev_save_data: SerializableTestbedState::default(),
            up_axis: rapier::math::Vector::Y,
        }
    }
}

impl TestbedState {
    /// Builds the grouped display order from a flat list of examples.
    pub fn set_examples(&mut self, examples: Vec<ExampleEntry>) {
        use indexmap::IndexSet;

        let mut groups: IndexSet<&'static str> = IndexSet::new();
        for example in &examples {
            groups.insert(example.group);
        }

        let mut ordered = Vec::new();
        for group in &groups {
            for example in &examples {
                if example.group == *group {
                    ordered.push(example.clone());
                }
            }
        }

        self.example_groups = groups.into_iter().collect();
        self.examples = ordered;
    }
}
