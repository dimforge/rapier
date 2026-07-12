use rapier::prelude::FeatureId;
use wasm_bindgen::prelude::wasm_bindgen;

#[cfg(feature = "dim2")]
#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawFeatureType {
    Vertex,
    Face,
    Unknown,
}

#[cfg(feature = "dim3")]
#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawFeatureType {
    Vertex,
    Edge,
    Face,
    Unknown,
}

pub trait IntoTypeValue {
    fn into_type(self) -> RawFeatureType;
    fn into_value(self) -> Option<u32>;
}

impl IntoTypeValue for FeatureId {
    fn into_type(self) -> RawFeatureType {
        match self {
            FeatureId::Vertex(_) => RawFeatureType::Vertex,
            #[cfg(feature = "dim3")]
            FeatureId::Edge(_) => RawFeatureType::Edge,
            FeatureId::Face(_) => RawFeatureType::Face,
            _ => RawFeatureType::Unknown,
        }
    }

    fn into_value(self) -> Option<u32> {
        match self {
            FeatureId::Vertex(id) | FeatureId::Face(id) => Some(id),
            #[cfg(feature = "dim3")]
            FeatureId::Edge(id) => Some(id),
            _ => None,
        }
    }
}
