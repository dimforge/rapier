//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase_multi_sap::{BroadPhase, BroadPhasePairEvent, ColliderPair};
pub use self::collider_components::*;
pub use self::contact_pair::{
    ContactData, ContactManifoldData, ContactPair, IntersectionPair, SolverContact, SolverFlags,
};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::interaction_groups::InteractionGroups;
pub use self::narrow_phase::NarrowPhase;

pub use self::collider::{Collider, ColliderBuilder};
pub use self::collider_set::ColliderSet;

pub use parry::query::TrackedContact;

/// A contact between two colliders.
pub type Contact = parry::query::TrackedContact<ContactData>;
/// A contact manifold between two colliders.
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
/// Events occurring when two colliders start or stop being in contact (or intersecting)
pub enum CollisionEvent {
    /// Event occurring when two colliders start being in contact (or intersecting)
    Started(ColliderHandle, ColliderHandle),
    /// Event occurring when two colliders stop being in contact (or intersecting).
    ///
    /// The boolean is set to `true` of this event originates from at least one of
    /// the colliders being removed from the `ColliderSet`.
    Stopped(ColliderHandle, ColliderHandle, bool),
}

impl CollisionEvent {
    pub(crate) fn new(h1: ColliderHandle, h2: ColliderHandle, start: bool) -> Self {
        if start {
            Self::Started(h1, h2)
        } else {
            Self::Stopped(h1, h2, false)
        }
    }

    /// Is this a `Started` collision event?
    pub fn started(self) -> bool {
        matches!(self, CollisionEvent::Started(..))
    }

    /// Is this a `Stopped` collision event?
    pub fn stopped(self) -> bool {
        matches!(self, CollisionEvent::Stopped(..))
    }

    /// The handle of the first collider involved in this collision event.
    pub fn collider1(self) -> ColliderHandle {
        match self {
            Self::Started(h, _) | Self::Stopped(h, _, _) => h,
        }
    }

    /// The handle of the second collider involved in this collision event.
    pub fn collider2(self) -> ColliderHandle {
        match self {
            Self::Started(_, h) | Self::Stopped(_, h, _) => h,
        }
    }
}

pub(crate) use self::broad_phase_multi_sap::SAPProxyIndex;
pub(crate) use self::narrow_phase::ContactManifoldIndex;
pub(crate) use parry::partitioning::QBVH;
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
mod collider_components;
mod contact_pair;
mod interaction_graph;
mod interaction_groups;
mod narrow_phase;

mod collider;
mod collider_set;
