//! Keyboard state tracking.

use kiss3d::event::Key;

/// Keyboard state
#[derive(Default, Clone, Debug)]
pub struct KeysState {
    pub shift: bool,
    pub ctrl: bool,
    pub alt: bool,
    pub pressed_keys: Vec<Key>,
}

impl KeysState {
    /// Check if a specific key is currently pressed
    pub fn pressed(&self, key: Key) -> bool {
        self.pressed_keys.contains(&key)
    }

    /// Get all currently pressed keys
    pub fn get_pressed(&self) -> &[Key] {
        &self.pressed_keys
    }
}
