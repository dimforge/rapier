//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase_multi_sap::BroadPhase;
pub use self::collider::{Collider, ColliderBuilder, ColliderShape};
pub use self::collider_set::{ColliderHandle, ColliderSet};
pub use self::contact::{ContactData, ContactManifoldData};
pub use self::contact::{ContactPair, SolverFlags};
pub use self::contact_generator::{ContactDispatcher, DefaultContactDispatcher};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::narrow_phase::NarrowPhase;
pub use self::polygon::Polygon;
pub use self::proximity::ProximityPair;
pub use self::proximity_detector::{DefaultProximityDispatcher, ProximityDispatcher};
pub use self::user_callbacks::{ContactPairFilter, PairFilterContext, ProximityPairFilter};
pub use buckler::query::Proximity;

pub use buckler::query::{KinematicsCategory, TrackedContact};

pub type Contact = buckler::query::TrackedContact<ContactData>;
pub type ContactManifold = buckler::query::ContactManifold<ContactManifoldData, ContactData>;
/// A segment shape.
pub type Segment = buckler::shape::Segment;
/// A cuboid shape.
pub type Cuboid = buckler::shape::Cuboid;
/// A triangle shape.
pub type Triangle = buckler::shape::Triangle;
/// A ball shape.
pub type Ball = buckler::shape::Ball;
/// A capsule shape.
pub type Capsule = buckler::shape::Capsule;
/// A heightfield shape.
pub type HeightField = buckler::shape::HeightField;
/// A cylindrical shape.
#[cfg(feature = "dim3")]
pub type Cylinder = buckler::shape::Cylinder;
/// A cone shape.
#[cfg(feature = "dim3")]
pub type Cone = buckler::shape::Cone;
/// An axis-aligned bounding box.
pub type AABB = buckler::bounding_volume::AABB;
/// A ray that can be cast against colliders.
pub type Ray = buckler::query::Ray;
/// The intersection between a ray and a  collider.
pub type RayIntersection = buckler::query::RayIntersection;
/// The the projection of a point on a collider.
pub type PointProjection = buckler::query::PointProjection;

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
pub(crate) use self::contact::WContact;
pub(crate) use self::narrow_phase::ContactManifoldIndex;
pub(crate) use buckler::partitioning::WQuadtree;
//pub(crate) use self::z_order::z_cmp_floats;
pub use self::interaction_groups::InteractionGroups;
pub use buckler::shape::*;

mod ball;
mod broad_phase_multi_sap;
mod collider;
mod collider_set;
mod contact;
mod contact_generator;
mod interaction_graph;
mod narrow_phase;
mod polygon;
mod proximity;
mod proximity_detector;
pub(crate) mod sat;
pub(crate) mod triangle;
//mod z_order;
mod interaction_groups;
mod user_callbacks;
