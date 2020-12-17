//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase_multi_sap::BroadPhase;
pub use self::collider::{Collider, ColliderBuilder, ColliderShape};
pub use self::collider_set::{ColliderHandle, ColliderSet};
// pub use self::contact_generator::{ContactDispatcher, DefaultContactDispatcher};
pub use self::contact_pair::{ContactData, ContactManifoldData};
pub use self::contact_pair::{ContactPair, SolverFlags};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::narrow_phase::NarrowPhase;
pub use self::polygon::Polygon;
pub use self::proximity_detector::{DefaultProximityDispatcher, ProximityDispatcher};
pub use self::proximity_pair::ProximityPair;
pub use self::user_callbacks::{ContactPairFilter, PairFilterContext, ProximityPairFilter};
pub use eagl::query::Proximity;

pub use eagl::query::{KinematicsCategory, TrackedContact};

pub type Contact = eagl::query::TrackedContact<ContactData>;
pub type ContactManifold = eagl::query::ContactManifold<ContactManifoldData, ContactData>;
/// A segment shape.
pub type Segment = eagl::shape::Segment;
/// A cuboid shape.
pub type Cuboid = eagl::shape::Cuboid;
/// A triangle shape.
pub type Triangle = eagl::shape::Triangle;
/// A ball shape.
pub type Ball = eagl::shape::Ball;
/// A capsule shape.
pub type Capsule = eagl::shape::Capsule;
/// A heightfield shape.
pub type HeightField = eagl::shape::HeightField;
/// A cylindrical shape.
#[cfg(feature = "dim3")]
pub type Cylinder = eagl::shape::Cylinder;
/// A cone shape.
#[cfg(feature = "dim3")]
pub type Cone = eagl::shape::Cone;
/// An axis-aligned bounding box.
pub type AABB = eagl::bounding_volume::AABB;
/// A ray that can be cast against colliders.
pub type Ray = eagl::query::Ray;
/// The intersection between a ray and a  collider.
pub type RayIntersection = eagl::query::RayIntersection;
/// The the projection of a point on a collider.
pub type PointProjection = eagl::query::PointProjection;

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
pub struct ProximityEvent {
    /// The first collider to which the proximity event applies.
    pub collider1: ColliderHandle,
    /// The second collider to which the proximity event applies.
    pub collider2: ColliderHandle,
    /// The previous state of proximity between the two collision objects.
    pub prev_status: Proximity,
    /// The new state of proximity between the two collision objects.
    pub new_status: Proximity,
}

impl ProximityEvent {
    /// Instantiates a new proximity event.
    ///
    /// Panics if `prev_status` is equal to `new_status`.
    pub fn new(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        prev_status: Proximity,
        new_status: Proximity,
    ) -> Self {
        assert_ne!(
            prev_status, new_status,
            "The previous and new status of a proximity event must not be the same."
        );
        Self {
            collider1,
            collider2,
            prev_status,
            new_status,
        }
    }
}

#[cfg(feature = "simd-is-enabled")]
pub(crate) use self::ball::WBall;
pub(crate) use self::broad_phase_multi_sap::{BroadPhasePairEvent, ColliderPair};
pub(crate) use self::collider_set::RemovedCollider;
#[cfg(feature = "simd-is-enabled")]
pub(crate) use self::contact_pair::WContact;
pub(crate) use self::narrow_phase::ContactManifoldIndex;
pub(crate) use eagl::partitioning::WQuadtree;
//pub(crate) use self::z_order::z_cmp_floats;
pub use self::interaction_groups::InteractionGroups;
pub use eagl::shape::*;

mod ball;
mod broad_phase_multi_sap;
mod collider;
mod collider_set;
// mod contact_generator;
mod contact_pair;
mod interaction_graph;
mod narrow_phase;
mod polygon;
mod proximity_detector;
mod proximity_pair;
pub(crate) mod sat;
//mod z_order;
mod interaction_groups;
mod user_callbacks;
