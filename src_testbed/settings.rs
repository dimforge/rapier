use indexmap::IndexMap;
use std::ops::RangeInclusive;

/// How a [`SettingValue::String`] is presented in the Example Settings
/// panel.
#[derive(Copy, Clone, PartialEq, Debug, Default, serde::Serialize, serde::Deserialize)]
pub enum StringDisplayMode {
    /// Single-line dropdown (default). Right for short option lists.
    #[default]
    ComboBox,
    /// Vertically-scrollable column of selectable labels. Use this when
    /// the option list is long enough that a ComboBox feels heavy.
    List,
}

#[derive(Clone, PartialEq, Debug, serde::Serialize, serde::Deserialize)]
pub enum SettingValue {
    Label(String),
    U32 {
        value: u32,
        range: RangeInclusive<u32>,
    },
    F32 {
        value: f32,
        range: RangeInclusive<f32>,
    },
    Bool {
        value: bool,
    },
    String {
        value: usize,
        range: Vec<String>,
        /// Controls how this setting is rendered in the UI. Defaults to
        /// [`StringDisplayMode::ComboBox`].
        #[serde(default)]
        display_mode: StringDisplayMode,
    },
}

#[derive(Default, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExampleSettings {
    values: IndexMap<String, SettingValue>,
    /// Keys whose change should NOT trigger a simulation restart. The example
    /// is expected to read such settings live (e.g. from a per-step callback)
    /// instead of relying on `init_world` re-running. Re-marked by the example
    /// on each `init_world`; not persisted.
    #[serde(skip)]
    non_restart_keys: std::collections::HashSet<String>,
}

impl ExampleSettings {
    pub fn clear(&mut self) {
        self.values.clear();
        self.non_restart_keys.clear();
    }

    /// Mark (`restart = false`) or unmark (`restart = true`) a setting key so
    /// that changing it in the UI does (not) trigger a simulation restart. A
    /// non-restart setting still updates its value in place — the example is
    /// responsible for reacting to it live.
    pub fn set_restart_on_change(&mut self, key: &str, restart: bool) {
        if restart {
            self.non_restart_keys.remove(key);
        } else {
            self.non_restart_keys.insert(key.to_string());
        }
    }

    /// Whether changing `key` should trigger a simulation restart (the default
    /// for every setting unless [`Self::set_restart_on_change`] said otherwise).
    pub fn changes_require_restart(&self, key: &str) -> bool {
        !self.non_restart_keys.contains(key)
    }

    /// The set of keys marked as non-restart via [`Self::set_restart_on_change`].
    pub fn non_restart_keys(&self) -> &std::collections::HashSet<String> {
        &self.non_restart_keys
    }

    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (&String, &mut SettingValue)> {
        self.values.iter_mut()
    }

    pub fn set_bool(&mut self, key: &str, value: bool) {
        self.values
            .insert(key.to_string(), SettingValue::Bool { value });
    }

    pub fn get_or_set_bool(&mut self, key: &'static str, default: bool) -> bool {
        let to_insert = SettingValue::Bool { value: default };
        let entry = self
            .values
            .entry(key.to_string())
            .or_insert(to_insert.clone());
        match entry {
            SettingValue::Bool { value } => *value,
            _ => {
                // The entry doesn’t have the right type. Overwrite with the new value.
                *entry = to_insert;
                default
            }
        }
    }

    pub fn set_u32(&mut self, key: &str, value: u32, range: RangeInclusive<u32>) {
        self.values
            .insert(key.to_string(), SettingValue::U32 { value, range });
    }

    pub fn get_or_set_u32(
        &mut self,
        key: &'static str,
        default: u32,
        range: RangeInclusive<u32>,
    ) -> u32 {
        let to_insert = SettingValue::U32 {
            value: default,
            range,
        };
        let entry = self
            .values
            .entry(key.to_string())
            .or_insert(to_insert.clone());
        match entry {
            SettingValue::U32 { value, .. } => *value,
            _ => {
                // The entry doesn’t have the right type. Overwrite with the new value.
                *entry = to_insert;
                default
            }
        }
    }

    pub fn set_f32(&mut self, key: &str, value: f32, range: RangeInclusive<f32>) {
        self.values
            .insert(key.to_string(), SettingValue::F32 { value, range });
    }

    pub fn get_or_set_f32(
        &mut self,
        key: &'static str,
        value: f32,
        range: RangeInclusive<f32>,
    ) -> f32 {
        let to_insert = SettingValue::F32 { value, range };
        let entry = self
            .values
            .entry(key.to_string())
            .or_insert(to_insert.clone());
        match entry {
            SettingValue::F32 { value, .. } => *value,
            _ => {
                // The entry doesn’t have the right type. Overwrite with the new value.
                *entry = to_insert;
                value
            }
        }
    }

    pub fn set_string(&mut self, key: &str, selected: usize, range: Vec<String>) {
        self.values.insert(
            key.to_string(),
            SettingValue::String {
                value: selected,
                range,
                display_mode: StringDisplayMode::ComboBox,
            },
        );
    }

    /// Insert a [`SettingValue::String`] if absent, otherwise return the
    /// existing index. The setting is rendered with `display_mode` —
    /// pass [`StringDisplayMode::List`] for option lists long enough
    /// that a ComboBox feels heavy.
    ///
    /// If the entry already exists with a *different* `display_mode`,
    /// the display mode is updated in place but the user's selected
    /// value is preserved.
    pub fn get_or_set_string_with(
        &mut self,
        key: &'static str,
        default: usize,
        range: Vec<String>,
        display_mode: StringDisplayMode,
    ) -> usize {
        let clamped_default = if range.is_empty() {
            0
        } else {
            default.min(range.len() - 1)
        };
        let to_insert = SettingValue::String {
            value: clamped_default,
            range,
            display_mode,
        };
        let entry = self
            .values
            .entry(key.to_string())
            .or_insert(to_insert.clone());
        match entry {
            SettingValue::String {
                value,
                display_mode: existing_mode,
                ..
            } => {
                // Keep the user's prior selection but pick up display-mode
                // changes from the caller.
                if *existing_mode != display_mode {
                    *existing_mode = display_mode;
                }
                *value
            }
            _ => {
                // The entry doesn't have the right type. Overwrite.
                *entry = to_insert;
                clamped_default
            }
        }
    }

    /// Convenience wrapper around [`Self::get_or_set_string_with`] that
    /// uses [`StringDisplayMode::ComboBox`].
    pub fn get_or_set_string(
        &mut self,
        key: &'static str,
        default: usize,
        range: Vec<String>,
    ) -> usize {
        self.get_or_set_string_with(key, default, range, StringDisplayMode::ComboBox)
    }

    pub fn get_bool(&self, key: &'static str) -> Option<bool> {
        match self.values.get(key)? {
            SettingValue::Bool { value } => Some(*value),
            _ => None,
        }
    }

    pub fn get_u32(&self, key: &'static str) -> Option<u32> {
        match self.values.get(key)? {
            SettingValue::U32 { value, .. } => Some(*value),
            _ => None,
        }
    }

    pub fn get_f32(&self, key: &'static str) -> Option<f32> {
        match self.values.get(key)? {
            SettingValue::F32 { value, .. } => Some(*value),
            _ => None,
        }
    }

    pub fn get_string_id(&self, key: &'static str) -> Option<usize> {
        match self.values.get(key)? {
            SettingValue::String { value, .. } => Some(*value),
            _ => None,
        }
    }

    pub fn get_string(&self, key: &'static str) -> Option<&str> {
        match self.values.get(key)? {
            SettingValue::String { value, range, .. } => Some(&range[*value]),
            _ => None,
        }
    }

    pub fn set_label(&mut self, key: &'static str, value: impl Into<String>) {
        self.values
            .insert(key.to_string(), SettingValue::Label(value.into()));
    }
}
