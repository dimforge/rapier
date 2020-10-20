use crate::geometry::proximity_detector::{
    PrimitiveProximityDetector, ProximityDetector, ProximityPhase,
    TrimeshShapeProximityDetectorWorkspace,
};
use crate::geometry::{Shape, ShapeType};
use std::any::Any;

/// Trait implemented by structures responsible for selecting a collision-detection algorithm
/// for a given pair of shapes.
pub trait ProximityDispatcher {
    /// Select the proximity detection algorithm for the given pair of primitive shapes.
    fn dispatch_primitives(
        &self,
        shape1: ShapeType,
        shape2: ShapeType,
    ) -> (
        PrimitiveProximityDetector,
        Option<Box<dyn Any + Send + Sync>>,
    );
    /// Select the proximity detection algorithm for the given pair of non-primitive shapes.
    fn dispatch(
        &self,
        shape1: ShapeType,
        shape2: ShapeType,
    ) -> (ProximityPhase, Option<Box<dyn Any + Send + Sync>>);
}

/// The default proximity dispatcher used by Rapier.
pub struct DefaultProximityDispatcher;

impl ProximityDispatcher for DefaultProximityDispatcher {
    fn dispatch_primitives(
        &self,
        shape1: ShapeType,
        shape2: ShapeType,
    ) -> (
        PrimitiveProximityDetector,
        Option<Box<dyn Any + Send + Sync>>,
    ) {
        match (shape1, shape2) {
            (ShapeType::Ball, ShapeType::Ball) => (
                PrimitiveProximityDetector {
                    #[cfg(feature = "simd-is-enabled")]
                    detect_proximity_simd: super::detect_proximity_ball_ball_simd,
                    detect_proximity: super::detect_proximity_ball_ball,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Cuboid, ShapeType::Cuboid) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_cuboid_cuboid,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Polygon, ShapeType::Polygon) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_polygon_polygon,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Triangle, ShapeType::Ball) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Ball, ShapeType::Triangle) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Cuboid, ShapeType::Ball) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Ball, ShapeType::Cuboid) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Triangle, ShapeType::Cuboid) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_cuboid_triangle,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (ShapeType::Cuboid, ShapeType::Triangle) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_cuboid_triangle,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            _ => (PrimitiveProximityDetector::default(), None),
        }
    }

    fn dispatch(
        &self,
        shape1: ShapeType,
        shape2: ShapeType,
    ) -> (ProximityPhase, Option<Box<dyn Any + Send + Sync>>) {
        match (shape1, shape2) {
            (ShapeType::Trimesh, _) => (
                ProximityPhase::NearPhase(ProximityDetector {
                    detect_proximity: super::detect_proximity_trimesh_shape,
                    ..ProximityDetector::default()
                }),
                Some(Box::new(TrimeshShapeProximityDetectorWorkspace::new())),
            ),
            (_, ShapeType::Trimesh) => (
                ProximityPhase::NearPhase(ProximityDetector {
                    detect_proximity: super::detect_proximity_trimesh_shape,
                    ..ProximityDetector::default()
                }),
                Some(Box::new(TrimeshShapeProximityDetectorWorkspace::new())),
            ),
            _ => {
                let (gen, workspace) = self.dispatch_primitives(shape1, shape2);
                (ProximityPhase::ExactPhase(gen), workspace)
            }
        }
    }
}
