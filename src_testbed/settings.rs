use indexmap::IndexMap;
use std::ops::RangeInclusive;

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
    },
}

#[derive(Default, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExampleSettings {
    values: IndexMap<String, SettingValue>,
}

impl ExampleSettings {
    pub fn clear(&mut self) {
        self.values.clear();
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
            },
        );
    }

    pub fn get_or_set_string(
        &mut self,
        key: &'static str,
        default: usize,
        range: Vec<String>,
    ) -> usize {
        let to_insert = SettingValue::String {
            value: default,
            range,
        };
        let entry = self
            .values
            .entry(key.to_string())
            .or_insert(to_insert.clone());
        match entry {
            SettingValue::String { value, .. } => *value,
            _ => {
                // The entry doesn’t have the right type. Overwrite with the new value.
                *entry = to_insert;
                default
            }
        }
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
            SettingValue::String { value, range } => Some(&range[*value]),
            _ => None,
        }
    }

    pub fn set_label(&mut self, key: &'static str, value: impl Into<String>) {
        self.values
            .insert(key.to_string(), SettingValue::Label(value.into()));
    }
}
