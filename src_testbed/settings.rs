use std::collections::HashMap;

#[derive(Copy, Clone, PartialEq, Debug, serde::Serialize, serde::Deserialize)]
pub enum SettingValue {
    U32(u32),
    F32(f32)
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

    pub fn set_u32(&mut self, key: &str, value: u32) {
        self.values.insert(key.to_string(), SettingValue::U32(value));
    }

    pub fn set_f32(&mut self, key: &str, value: f32) {
        self.values.insert(key.to_string(), SettingValue::F32(value));
    }

    pub fn get_u32(&self, key: &'static str) -> Option<u32> {
        match self.values.get(key)? {
            SettingValue::U32(value) => Some(*value),
            _ => None
        }
    }

    pub fn get_f32(&self, key: &'static str) -> Option<f32> {
        match self.values.get(key)? {
            SettingValue::F32(value) => Some(*value),
            _ => None
        }
    }
}