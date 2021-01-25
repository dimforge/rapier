//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase_multi_sap::BroadPhase;
pub use self::collider::{Collider, ColliderBuilder};
pub use self::collider_set::{ColliderHandle, ColliderSet};
pub use self::contact_pair::{ContactData, ContactManifoldData};
pub use self::contact_pair::{ContactPair, SolverContact, SolverFlags};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::interaction_groups::InteractionGroups;
pub use self::narrow_phase::NarrowPhase;
pub use self::pair_filter::{ContactPairFilter, IntersectionPairFilter, PairFilterContext};

pub use parry::query::TrackedContact;

pub type Contact = parry::query::TrackedContact<ContactData>;
pub type ContactManifold = parry::query::ContactManifold<ContactManifoldData, ContactData>;
/// A segment shape.
pub type Segment = parry::shape::Segment;
/// A cuboid shape.
pub type Cuboid = parry::shape::Cuboid;
/// A triangle shape.
pub type Triangle = parry::shape::Triangle;
/// A ball shape.
pub type Ball = parry::shape::Ball;
/// A capsule shape.
pub type Capsule = parry::shape::Capsule;
/// A heightfield shape.
pub type HeightField = parry::shape::HeightField;
/// A cylindrical shape.
#[cfg(feature = "dim3")]
pub type Cylinder = parry::shape::Cylinder;
/// A cone shape.
#[cfg(feature = "dim3")]
pub type Cone = parry::shape::Cone;
/// An axis-aligned bounding box.
pub type AABB = parry::bounding_volume::AABB;
/// A ray that can be cast against colliders.
pub type Ray = parry::query::Ray;
/// The intersection between a ray and a  collider.
pub type RayIntersection = parry::query::RayIntersection;
/// The the projection of a point on a collider.
pub type PointProjection = parry::query::PointProjection;
/// The the time of impact between two shapes.
pub type TOI = parry::query::TOI;
pub use parry::shape::SharedShape;

#[derive(Copy, Clone, Hash, Debug)]
/// Events occurring when two collision objects start or stop being in contact (or penetration).
pub enum ContactEvent {
    /// Event occurring when two collision objects start being in contact.
    ///
    /// This event is generated whenever the narrow-phase finds a contact between two collision objects that did not have any contact at the last update.
    Started(ColliderHandle, ColliderHandle),
    /// Event occurring when two collision objects stop being in contact.
    ///
    /// This event is generated whenever the narrow-phase fails to find any contact between two collision objects that did have at least one contact at the last update.
    Stopped(ColliderHandle, ColliderHandle),
}

#[derive(Copy, Clone, Debug)]
/// Events occurring when two collision objects start or stop being in close proximity, contact, or disjoint.
pub struct IntersectionEvent {
    /// The first collider to which the proximity event applies.
    pub collider1: ColliderHandle,
    /// The second collider to which the proximity event applies.
    pub collider2: ColliderHandle,
    /// Are the two colliders intersecting?
    pub intersecting: bool,
}

impl IntersectionEvent {
    /// Instantiates a new proximity event.
    ///
    /// Panics if `prev_status` is equal to `new_status`.
    pub fn new(collider1: ColliderHandle, collider2: ColliderHandle, intersecting: bool) -> Self {
        Self {
            collider1,
            collider2,
            intersecting,
        }
    }
}

pub(crate) use self::broad_phase_multi_sap::{BroadPhasePairEvent, ColliderPair};
pub(crate) use self::collider_set::RemovedCollider;
pub(crate) use self::narrow_phase::ContactManifoldIndex;
pub(crate) use parry::partitioning::SimdQuadTree;
pub use parry::shape::*;

#[cfg(feature = "serde-serialize")]
pub(crate) fn default_persistent_query_dispatcher(
) -> std::sync::Arc<dyn parry::query::PersistentQueryDispatcher<ContactManifoldData, ContactData>> {
    std::sync::Arc::new(parry::query::DefaultQueryDispatcher)
}

#[cfg(feature = "serde-serialize")]
pub(crate) fn default_query_dispatcher() -> std::sync::Arc<dyn parry::query::QueryDispatcher> {
    std::sync::Arc::new(parry::query::DefaultQueryDispatcher)
}

mod broad_phase_multi_sap;
mod collider;
mod collider_set;
mod contact_pair;
mod interaction_graph;
mod interaction_groups;
mod narrow_phase;
mod pair_filter;
