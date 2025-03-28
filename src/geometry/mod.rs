//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase::BroadPhase;
pub use self::broad_phase_multi_sap::{BroadPhaseMultiSap, BroadPhasePairEvent, ColliderPair};
pub use self::collider::{Collider, ColliderBuilder};
pub use self::collider_components::*;
pub use self::collider_set::ColliderSet;
pub use self::contact_pair::{
    ContactData, ContactManifoldData, ContactPair, IntersectionPair, SolverContact, SolverFlags,
};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::interaction_groups::{Group, InteractionGroups};
pub use self::mesh_converter::{MeshConverter, MeshConverterError};
pub use self::narrow_phase::NarrowPhase;

pub use parry::bounding_volume::BoundingVolume;
pub use parry::query::{PointQuery, PointQueryWithLocation, RayCast, TrackedContact};
pub use parry::shape::SharedShape;

use crate::math::{Real, Vector};

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
pub type Aabb = parry::bounding_volume::Aabb;
/// A ray that can be cast against colliders.
pub type Ray = parry::query::Ray;
/// The intersection between a ray and a  collider.
pub type RayIntersection = parry::query::RayIntersection;
/// The projection of a point on a collider.
pub type PointProjection = parry::query::PointProjection;
/// The result of a shape-cast between two shapes.
pub type ShapeCastHit = parry::query::ShapeCastHit;
/// The default broad-phase implementation recommended for general-purpose usage.
pub type DefaultBroadPhase = BroadPhaseMultiSap;

bitflags::bitflags! {
    /// Flags providing more information regarding a collision event.
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    #[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
    pub struct CollisionEventFlags: u32 {
        /// Flag set if at least one of the colliders involved in the
        /// collision was a sensor when the event was fired.
        const SENSOR = 0b0001;
        /// Flag set if a `CollisionEvent::Stopped` was fired because
        /// at least one of the colliders was removed.
        const REMOVED = 0b0010;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Hash, Debug)]
/// Events occurring when two colliders start or stop colliding
pub enum CollisionEvent {
    /// Event occurring when two colliders start colliding
    Started(ColliderHandle, ColliderHandle, CollisionEventFlags),
    /// Event occurring when two colliders stop colliding.
    Stopped(ColliderHandle, ColliderHandle, CollisionEventFlags),
}

impl CollisionEvent {
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
            Self::Started(h, _, _) | Self::Stopped(h, _, _) => h,
        }
    }

    /// The handle of the second collider involved in this collision event.
    pub fn collider2(self) -> ColliderHandle {
        match self {
            Self::Started(_, h, _) | Self::Stopped(_, h, _) => h,
        }
    }

    /// Was at least one of the colliders involved in the collision a sensor?
    pub fn sensor(self) -> bool {
        match self {
            Self::Started(_, _, f) | Self::Stopped(_, _, f) => {
                f.contains(CollisionEventFlags::SENSOR)
            }
        }
    }

    /// Was at least one of the colliders involved in the collision removed?
    pub fn removed(self) -> bool {
        match self {
            Self::Started(_, _, f) | Self::Stopped(_, _, f) => {
                f.contains(CollisionEventFlags::REMOVED)
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default)]
/// Event occurring when the sum of the magnitudes of the contact forces
/// between two colliders exceed a threshold.
pub struct ContactForceEvent {
    /// The first collider involved in the contact.
    pub collider1: ColliderHandle,
    /// The second collider involved in the contact.
    pub collider2: ColliderHandle,
    /// The sum of all the forces between the two colliders.
    pub total_force: Vector<Real>,
    /// The sum of the magnitudes of each force between the two colliders.
    ///
    /// Note that this is **not** the same as the magnitude of `self.total_force`.
    /// Here we are summing the magnitude of all the forces, instead of taking
    /// the magnitude of their sum.
    pub total_force_magnitude: Real,
    /// The world-space (unit) direction of the force with strongest magnitude.
    pub max_force_direction: Vector<Real>,
    /// The magnitude of the largest force at a contact point of this contact pair.
    pub max_force_magnitude: Real,
}

impl ContactForceEvent {
    /// Init a contact force event from a contact pair.
    pub fn from_contact_pair(dt: Real, pair: &ContactPair, total_force_magnitude: Real) -> Self {
        let mut result = ContactForceEvent {
            collider1: pair.collider1,
            collider2: pair.collider2,
            total_force_magnitude,
            ..ContactForceEvent::default()
        };

        for m in &pair.manifolds {
            let mut total_manifold_impulse = 0.0;
            for pt in m.contacts() {
                total_manifold_impulse += pt.data.impulse;

                if pt.data.impulse > result.max_force_magnitude {
                    result.max_force_magnitude = pt.data.impulse;
                    result.max_force_direction = m.data.normal;
                }
            }

            result.total_force += m.data.normal * total_manifold_impulse;
        }

        let inv_dt = crate::utils::inv(dt);
        // NOTE: convert impulses to forces. Note that we
        //       don’t need to convert the `total_force_magnitude`
        //       because it’s an input of this function already
        //       assumed to be a force instead of an impulse.
        result.total_force *= inv_dt;
        result.max_force_magnitude *= inv_dt;
        result
    }
}

pub(crate) use self::broad_phase::BroadPhaseProxyIndex;
pub(crate) use self::collider_set::ModifiedColliders;
pub(crate) use self::narrow_phase::ContactManifoldIndex;
pub(crate) use parry::partitioning::Qbvh;
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

mod broad_phase;
mod broad_phase_qbvh;
mod collider;
mod collider_set;
mod mesh_converter;
