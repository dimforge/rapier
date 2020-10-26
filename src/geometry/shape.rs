use crate::dynamics::MassProperties;
use crate::geometry::{
    Ball, Capsule, Cuboid, HeightField, Roundable, Rounded, Segment, Triangle, Trimesh,
};
use crate::math::Isometry;
use downcast_rs::{impl_downcast, DowncastSync};
#[cfg(feature = "serde-serialize")]
use erased_serde::Serialize;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::query::{PointQuery, RayCast};
use num::Zero;
use num_derive::FromPrimitive;
#[cfg(feature = "dim3")]
use {
    crate::geometry::{Cone, Cylinder, PolygonalFeatureMap},
    ncollide::bounding_volume::BoundingVolume,
};

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
    /// A segment shape.
    Segment,
    /// A triangle shape.
    Triangle,
    /// A triangle mesh shape.
    Trimesh,
    /// A heightfield shape.
    HeightField,
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder,
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cone,
    // /// A custom shape type.
    // Custom(u8),
    // /// A cuboid with rounded corners.
    // RoundedCuboid,
    // /// A triangle with rounded corners.
    // RoundedTriangle,
    // /// A triangle-mesh with rounded corners.
    // RoundedTrimesh,
    // /// An heightfield with rounded corners.
    // RoundedHeightField,
    /// A cylinder with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCylinder,
    // /// A cone with rounded corners.
    // RoundedCone,
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
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
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
    #[cfg(feature = "dim3")]
    pub fn as_cylinder(&self) -> Option<&Cylinder> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone(&self) -> Option<&Cone> {
        self.downcast_ref()
    }

    /// Converts this abstract shape to a cone, if it is one.
    pub fn as_rounded<S>(&self) -> Option<&Rounded<S>>
    where
        S: Roundable,
        Rounded<S>: Shape,
    {
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
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }
}

impl Shape for Capsule {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.aabb(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_capsule(density, self.segment.a, self.segment.b, self.radius)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Capsule
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((&self.segment as &dyn PolygonalFeatureMap, self.radius))
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

    fn mass_properties(&self, _density: f32) -> MassProperties {
        MassProperties::zero()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Triangle
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }
}

impl Shape for Segment {
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
        ShapeType::Segment
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
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
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }
}

#[cfg(feature = "dim3")]
impl Shape for Cone {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.bounding_volume(position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        MassProperties::from_cone(density, self.half_height, self.radius)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Cone
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }
}

#[cfg(feature = "dim3")]
impl Shape for Rounded<Cylinder> {
    fn as_serialize(&self) -> Option<&dyn Serialize> {
        Some(self as &dyn Serialize)
    }

    fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        self.shape.compute_aabb(position).loosened(self.radius)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        // We ignore the margin here.
        self.shape.mass_properties(density)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::RoundCylinder
    }

    #[cfg(feature = "dim3")]
    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, f32)> {
        Some((&self.shape as &dyn PolygonalFeatureMap, self.radius))
    }
}
