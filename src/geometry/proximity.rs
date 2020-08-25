use crate::geometry::proximity_detector::ProximityPhase;
use crate::geometry::{ColliderPair, Proximity};
use std::any::Any;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The description of the proximity of two colliders.
pub struct ProximityPair {
    /// The pair of collider involved.
    pub pair: ColliderPair,
    /// The state of proximity between the two colliders.
    pub proximity: Proximity,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) detector: Option<ProximityPhase>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) detector_workspace: Option<Box<dyn Any + Send + Sync>>,
}

impl ProximityPair {
    pub(crate) fn new(
        pair: ColliderPair,
        detector: ProximityPhase,
        detector_workspace: Option<Box<dyn Any + Send + Sync>>,
    ) -> Self {
        Self {
            pair,
            proximity: Proximity::Disjoint,
            detector: Some(detector),
            detector_workspace,
        }
    }
}
