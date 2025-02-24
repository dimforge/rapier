use std::collections::HashMap;
use std::ops::RangeInclusive;

#[derive(Clone, PartialEq, Debug, serde::Serialize, serde::Deserialize)]
pub enum SettingValue {
    U32 {
        value: u32,
        range: RangeInclusive<u32>,
    },
    F32 {
        value: f32,
        range: RangeInclusive<f32>,
    },
}

#[derive(Default, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExampleSettings {
    values: HashMap<String, SettingValue>,
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
}
