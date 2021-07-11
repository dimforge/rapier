use crate::geometry::ColliderHandle;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderPair {
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle,
}

impl ColliderPair {
    pub fn new(collider1: ColliderHandle, collider2: ColliderHandle) -> Self {
        ColliderPair {
            collider1,
            collider2,
        }
    }

    pub fn swap(self) -> Self {
        Self::new(self.collider2, self.collider1)
    }

    pub fn zero() -> Self {
        Self {
            collider1: ColliderHandle::from_raw_parts(0, 0),
            collider2: ColliderHandle::from_raw_parts(0, 0),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BroadPhasePairEvent {
    AddPair(ColliderPair),
    DeletePair(ColliderPair),
}
