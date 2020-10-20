use crate::dynamics::{MassProperties, RigidBodyHandle, RigidBodySet};
#[cfg(feature = "dim3")]
use crate::geometry::PolygonalFeatureMap;
use crate::geometry::{
    Ball, Capsule, ColliderGraphIndex, Contact, Cuboid, Cylinder, HeightField, InteractionGraph,
    Polygon, Proximity, Ray, RayIntersection, Triangle, Trimesh,
};
use crate::math::{AngVector, Isometry, Point, Rotation, Vector};
use downcast_rs::{impl_downcast, DowncastSync};
use erased_serde::Serialize;
use na::Point3;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::query::{PointQuery, RayCast};
use num::Zero;
use num_derive::FromPrimitive;
use std::any::Any;

#[derive(Copy, Clone, Debug, FromPrimitive)]
/// Enum representing the type of a shape.
pub enum ShapeType {
    /// A ball shape.
    Ball = 1,
    /// A convex polygon shape.
    Polygon,
    /// A cuboid shape.
    Cuboid,
    /// A capsule shape.
    Capsule,
    /// A triangle shape.
    Triangle,
    /// A triangle mesh shape.
    Trimesh,
    /// A heightfield shape.
    HeightField,
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder,
    // /// A custom shape type.
    // Custom(u8),
}

/// Trait implemented by shapes usable by Rapier.
pub trait Shape: RayCast<f32> + PointQuery<f32> + DowncastSync {
    /// Convert this shape as a serializable entity.
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        None
    }

    /// Computes the AABB of this shape.
    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32>;

    /// Compute the mass-properties of this shape given its uniform density.
    fn mass_properties(&self, density: f32) -> MassProperties;

    /// Gets the type tag of this shape.
    fn shape_type(&self) -> ShapeType;

    /// Converts this shape to a polygonal feature-map, if it is one.
    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<&dyn PolygonalFeatureMap> {
        None
    }

    // fn as_rounded(&self) -> Option<&Rounded<Box<AnyShape>>> {
    //     None
    // }
}

impl_downcast!(sync Shape);

impl dyn Shape {
    /// Converts this abstract shape to a ball, if it is one.
    pub fn as_ball(&self) -> Option<&Ball> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a cuboid, if it is one.
    pub fn as_cuboid(&self) -> Option<&Cuboid> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a capsule, if it is one.
    pub fn as_capsule(&self) -> Option<&Capsule> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a triangle, if it is one.
    pub fn as_triangle(&self) -> Option<&Triangle> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a triangle mesh, if it is one.
    pub fn as_trimesh(&self) -> Option<&Trimesh> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a heightfield, if it is one.
    pub fn as_heightfield(&self) -> Option<&HeightField> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a cylinder, if it is one.
    pub fn as_cylinder(&self) -> Option<&Cylinder> {
        self.downcast_ref()
    }
}

impl Shape for Ball {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_ball(density, self.radius)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Ball
    }
}

// impl Shape for Polygon {
//     #[cfg(feature = "serde-serialize")]
//     fn as_serialize(&self) -> Option<&dyn Serialize> {
//         Some(self as &dyn Serialize)
//     }
//
//     fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
//         self.aabb(position)
//     }
//
//     fn mass_properties(&self, _density: f32) -> MassProperties {
//         unimplemented!()
//     }
//
//     fn shape_type(&self) -> ShapeType {
//         ShapeType::Polygon
//     }
// }

impl Shape for Cuboid {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_cuboid(density, self.half_extents)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cuboid
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<&dyn PolygonalFeatureMap> {
        Some(self as &dyn PolygonalFeatureMap)
    }
}

impl Shape for Capsule {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_capsule(density, self.half_height, self.radius)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Capsule
    }
}

impl Shape for Triangle {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Triangle
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<&dyn PolygonalFeatureMap> {
        Some(self as &dyn PolygonalFeatureMap)
    }
}

impl Shape for Trimesh {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.aabb(position)
    }

    fn mass_properties(&self, _density: f32) -> MassProperties {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Trimesh
    }
}

impl Shape for HeightField {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, _density: f32) -> MassProperties {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::HeightField
    }
}

#[cfg(feature = "dim3")]
impl Shape for Cylinder {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_cylinder(density, self.half_height, self.radius)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cylinder
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<&dyn PolygonalFeatureMap> {
        Some(self as &dyn PolygonalFeatureMap)
    }
}
