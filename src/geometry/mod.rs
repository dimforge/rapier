//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase_multi_sap::BroadPhase;
pub use self::capsule::Capsule;
pub use self::collider::{Collider, ColliderBuilder, Shape};
pub use self::collider_set::{ColliderHandle, ColliderSet};
pub use self::contact::{
    Contact, ContactKinematics, ContactManifold, ContactPair, KinematicsCategory,
};
pub use self::contact_generator::{ContactDispatcher, DefaultContactDispatcher};
#[cfg(feature = "dim2")]
pub(crate) use self::cuboid_feature2d::{CuboidFeature, CuboidFeatureFace};
#[cfg(feature = "dim3")]
pub(crate) use self::cuboid_feature3d::{CuboidFeature, CuboidFeatureFace};
pub use self::interaction_graph::{
    ColliderGraphIndex, InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex,
};
pub use self::narrow_phase::NarrowPhase;
pub use self::polygon::Polygon;
pub use self::proximity::ProximityPair;
pub use self::proximity_detector::{DefaultProximityDispatcher, ProximityDispatcher};
pub use self::trimesh::Trimesh;
pub use ncollide::query::Proximity;

/// A cuboid shape.
pub type Cuboid = ncollide::shape::Cuboid<f32>;
/// A triangle shape.
pub type Triangle = ncollide::shape::Triangle<f32>;
/// A ball shape.
pub type Ball = ncollide::shape::Ball<f32>;
/// A heightfield shape.
pub type HeightField = ncollide::shape::HeightField<f32>;
/// An axis-aligned bounding box.
pub type AABB = ncollide::bounding_volume::AABB<f32>;
/// Event triggered when two non-sensor colliders start or stop being in contact.
pub type ContactEvent = ncollide::pipeline::ContactEvent<ColliderHandle>;
/// Event triggered when a sensor collider starts or stop being in proximity with another collider (sensor or not).
pub type ProximityEvent = ncollide::pipeline::ProximityEvent<ColliderHandle>;
/// A ray that can be cast against colliders.
pub type Ray = ncollide::query::Ray<f32>;
/// The intersection between a ray and a  collider.
pub type RayIntersection = ncollide::query::RayIntersection<f32>;

#[cfg(feature = "simd-is-enabled")]
pub(crate) use self::ball::WBall;
pub(crate) use self::broad_phase::{ColliderPair, WAABBHierarchy, WAABBHierarchyIntersections};
pub(crate) use self::broad_phase_multi_sap::BroadPhasePairEvent;
#[cfg(feature = "simd-is-enabled")]
pub(crate) use self::contact::WContact;
#[cfg(feature = "dim2")]
pub(crate) use self::contact_generator::{clip_segments, clip_segments_with_normal};
pub(crate) use self::narrow_phase::ContactManifoldIndex;
#[cfg(feature = "dim3")]
pub(crate) use self::polyhedron_feature3d::PolyhedronFace;
pub(crate) use self::waabb::WAABB;
//pub(crate) use self::z_order::z_cmp_floats;

mod ball;
mod broad_phase;
mod broad_phase_multi_sap;
mod capsule;
mod collider;
mod collider_set;
mod contact;
mod contact_generator;
pub(crate) mod cuboid;
#[cfg(feature = "dim2")]
mod cuboid_feature2d;
#[cfg(feature = "dim3")]
mod cuboid_feature3d;
mod interaction_graph;
mod narrow_phase;
mod polygon;
#[cfg(feature = "dim3")]
mod polyhedron_feature3d;
mod proximity;
mod proximity_detector;
pub(crate) mod sat;
pub(crate) mod triangle;
mod trimesh;
mod waabb;
//mod z_order;
