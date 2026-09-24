pub use self::collider::*;
pub use self::shape_views::ColliderView;
pub use rapier::geometry::SolverFlags;
pub use rapier::geometry::{MeshConverter, MeshConverterError};
pub use rapier::parry::query::{ShapeCastOptions, ShapeCastStatus, Unsupported};
#[cfg(feature = "dim3")]
pub use rapier::parry::shape::HeightFieldFlags;
pub use rapier::parry::shape::{CompoundFlags, PolylineFlags, TriMeshFlags};
pub use rapier::parry::transformation::{vhacd::VHACDParameters, voxelization::FillMode};

use crate::math::{Real, Rot, Vect};
use rapier::prelude::FeatureId;

mod collider;
mod collider_impl;
/// Wrappers around Rapier shapes to access their properties.
pub mod shape_views;
#[cfg(feature = "to-bevy-mesh")]
pub mod to_bevy_mesh;

/// Result of the projection of a point on a shape.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PointProjection {
    /// Whether or not the point to project was inside of the shape.
    pub is_inside: bool,
    /// The projection result.
    pub point: Vect,
    /// The index of the part of the shape the point was projected on (for compound shapes,
    /// triangle meshes, polylines, voxels, etc.), or `0` for a shape with no sub-shape.
    pub subshape: u32,
}

impl PointProjection {
    pub(crate) fn from_rapier(raw: rapier::parry::query::PointProjection) -> Self {
        Self {
            is_inside: raw.is_inside,
            point: raw.point,
            subshape: raw.subshape,
        }
    }
}
impl From<rapier::parry::query::PointProjection> for PointProjection {
    fn from(projection: rapier::parry::query::PointProjection) -> PointProjection {
        PointProjection::from_rapier(projection)
    }
}

/// Structure containing the result of a successful ray cast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RayIntersection {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with `origin + dir * time_of_impact` where `origin` is the origin of the ray;
    /// `dir` is its direction and `time_of_impact` is the value of this field.
    pub time_of_impact: Real,

    /// The intersection point between the ray and the object.
    pub point: Vect,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    pub normal: Vect,

    /// Feature at the intersection point.
    pub feature: FeatureId,

    /// The index of the part of the shape hit by the ray (for compound shapes, triangle meshes,
    /// polylines, voxels, etc.), or `0` for a shape with no sub-shape.
    pub subshape: u32,
}

impl RayIntersection {
    pub(crate) fn from_rapier(
        inter: rapier::parry::query::RayIntersection,
        unscaled_origin: Vect,
        unscaled_dir: Vect,
    ) -> Self {
        Self {
            time_of_impact: inter.time_of_impact,
            point: unscaled_origin + unscaled_dir * inter.time_of_impact,
            normal: inter.normal,
            feature: inter.feature,
            subshape: inter.subshape,
        }
    }
}

/// The result of a shape cast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ShapeCastHit {
    /// The time at which the objects touch.
    pub time_of_impact: Real,
    /// Detail about the impact points.
    ///
    /// `None` if `status` is `PenetratingOrWithinTargetDist` and
    /// [`ShapeCastOptions::compute_impact_geometry_on_penetration`] was `false`.
    pub details: Option<ShapeCastHitDetails>,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: ShapeCastStatus,
    /// The index of the part of the first shape that was hit (for compound shapes, triangle
    /// meshes, polylines, voxels, etc.), or `0` for a shape with no sub-shape.
    ///
    /// For scene queries and character collisions, the first shape is the collider being hit.
    pub subshape1: u32,
    /// The index of the part of the second shape that was hit (for compound shapes, triangle
    /// meshes, polylines, voxels, etc.), or `0` for a shape with no sub-shape.
    ///
    /// For scene queries and character collisions, the second shape is the shape being cast.
    pub subshape2: u32,
}

/// In depth information about a shape-cast hit.
///
/// The frame of each field depends on the query:
/// - For scene queries (e.g. [`RapierContext::cast_shape`],
///   [`RapierContext::cast_shape_nonlinear`]) and [`CharacterCollision::hit`], the first shape is
///   the collider hit: `witness1` and `normal1` are in world-space. The second shape is the cast
///   shape (or the character): `witness2` and `normal2` are in its local-space, at its pose at the
///   time of impact.
/// - For [`Collider::cast_shape`] and [`Collider::cast_shape_nonlinear`], the first shape is `self`
///   and the second one `other`: every field is in the local-space of its shape.
///
/// [`RapierContext::cast_shape`]: crate::plugin::context::systemparams::RapierContext::cast_shape
/// [`RapierContext::cast_shape_nonlinear`]: crate::plugin::context::systemparams::RapierContext::cast_shape_nonlinear
/// [`CharacterCollision::hit`]: crate::control::CharacterCollision::hit
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ShapeCastHitDetails {
    /// The closest point on the first shape at the time of impact (world-space for scene
    /// queries, local-space otherwise).
    pub witness1: Vect,
    /// The closest point on the second shape at the time of impact, in its local-space.
    pub witness2: Vect,
    /// The outward normal on the first shape at the time of impact (world-space for scene
    /// queries, local-space otherwise).
    pub normal1: Vect,
    /// The outward normal on the second shape at the time of impact, in its local-space.
    pub normal2: Vect,
}

impl ShapeCastHit {
    /// Convert from internal `rapier::query::ShapeCastHit`.
    pub fn from_rapier(
        hit: rapier::parry::query::ShapeCastHit,
        details_always_computed: bool,
    ) -> Self {
        let details = match (details_always_computed, hit.status) {
            (_, ShapeCastStatus::Failed) => None,
            (false, ShapeCastStatus::PenetratingOrWithinTargetDist) => None,
            _ => Some(ShapeCastHitDetails {
                witness1: hit.witness1,
                witness2: hit.witness2,
                normal1: hit.normal1,
                normal2: hit.normal2,
            }),
        };
        Self {
            time_of_impact: hit.time_of_impact,
            status: hit.status,
            details,
            subshape1: hit.subshape1,
            subshape2: hit.subshape2,
        }
    }
}

/// The result of a contact query between two colliders.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ShapeContact {
    /// The contact point on the first collider, in world-space.
    pub point1: Vect,
    /// The contact point on the second collider, in world-space.
    pub point2: Vect,
    /// The contact normal, pointing outward from the first collider, in world-space.
    pub normal1: Vect,
    /// The contact normal, pointing outward from the second collider, in world-space.
    ///
    /// This is always equal to `-normal1`.
    pub normal2: Vect,
    /// The signed distance between the two contact points.
    ///
    /// It is negative if the colliders are penetrating.
    pub distance: Real,
    /// The index of the part of the first collider involved in the contact (for compound
    /// shapes, triangle meshes, polylines, etc.), or `0` for a shape with no sub-shape.
    pub subshape1: u32,
    /// The index of the part of the second collider involved in the contact (for compound
    /// shapes, triangle meshes, polylines, etc.), or `0` for a shape with no sub-shape.
    pub subshape2: u32,
}

impl ShapeContact {
    pub(crate) fn from_rapier(contact: rapier::parry::query::Contact) -> Self {
        Self {
            point1: contact.point1,
            point2: contact.point2,
            normal1: contact.normal1,
            normal2: contact.normal2,
            distance: contact.dist,
            subshape1: contact.subshape1,
            subshape2: contact.subshape2,
        }
    }
}

/// The result of a closest-points query between two colliders.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ShapeClosestPoints {
    /// The two colliders are intersecting.
    Intersecting,
    /// The two colliders are disjoint but closer than the queried maximum distance.
    ///
    /// Contains the closest point on the first collider, then the closest point on the second
    /// collider, both in world-space.
    WithinMargin(Vect, Vect),
    /// The two colliders are further apart than the queried maximum distance.
    Disjoint,
}

impl ShapeClosestPoints {
    pub(crate) fn from_rapier(points: rapier::parry::query::ClosestPoints) -> Self {
        match points {
            rapier::parry::query::ClosestPoints::Intersecting => Self::Intersecting,
            rapier::parry::query::ClosestPoints::WithinMargin(p1, p2) => Self::WithinMargin(p1, p2),
            rapier::parry::query::ClosestPoints::Disjoint => Self::Disjoint,
        }
    }
}

/// A rigid motion combining a constant linear and a constant angular velocity, used for
/// nonlinear shape casting with [`Collider::cast_shape_nonlinear`].
///
/// At time `t`, the collider is rotated by `angular_velocity * t` around the point
/// `local_center` (expressed in the collider's local-space), and translated by
/// `linear_velocity * t`, starting from the pose given by `translation` and `rotation`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NonlinearMotion {
    /// The translation of the collider at time `0.0`.
    pub translation: Vect,
    /// The rotation of the collider at time `0.0`.
    pub rotation: Rot,
    /// The local-space point around which the collider rotates.
    pub local_center: Vect,
    /// The linear velocity of the collider.
    pub linear_velocity: Vect,
    /// The angular velocity of the collider.
    #[cfg(feature = "dim2")]
    pub angular_velocity: Real,
    /// The angular velocity of the collider.
    #[cfg(feature = "dim3")]
    pub angular_velocity: Vect,
}

impl NonlinearMotion {
    /// A motion that keeps the collider at the given position.
    pub fn constant_position(translation: Vect, rotation: Rot) -> Self {
        Self {
            translation,
            rotation,
            local_center: Vect::ZERO,
            linear_velocity: Vect::ZERO,
            #[cfg(feature = "dim2")]
            angular_velocity: 0.0,
            #[cfg(feature = "dim3")]
            angular_velocity: Vect::ZERO,
        }
    }

    /// Converts this motion to Rapier's `NonlinearRigidMotion`.
    pub fn into_rapier(self) -> rapier::parry::query::NonlinearRigidMotion {
        rapier::parry::query::NonlinearRigidMotion::new(
            crate::utils::pose_from(self.translation, self.rotation),
            self.local_center,
            self.linear_velocity,
            self.angular_velocity,
        )
    }
}
