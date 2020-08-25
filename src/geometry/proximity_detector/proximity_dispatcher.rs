use crate::geometry::proximity_detector::{
    PrimitiveProximityDetector, ProximityDetector, ProximityPhase,
    TrimeshShapeProximityDetectorWorkspace,
};
use crate::geometry::Shape;
use std::any::Any;

/// Trait implemented by structures responsible for selecting a collision-detection algorithm
/// for a given pair of shapes.
pub trait ProximityDispatcher {
    /// Select the proximity detection algorithm for the given pair of primitive shapes.
    fn dispatch_primitives(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (
        PrimitiveProximityDetector,
        Option<Box<dyn Any + Send + Sync>>,
    );
    /// Select the proximity detection algorithm for the given pair of non-primitive shapes.
    fn dispatch(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (ProximityPhase, Option<Box<dyn Any + Send + Sync>>);
}

/// The default proximity dispatcher used by Rapier.
pub struct DefaultProximityDispatcher;

impl ProximityDispatcher for DefaultProximityDispatcher {
    fn dispatch_primitives(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (
        PrimitiveProximityDetector,
        Option<Box<dyn Any + Send + Sync>>,
    ) {
        match (shape1, shape2) {
            (Shape::Ball(_), Shape::Ball(_)) => (
                PrimitiveProximityDetector {
                    #[cfg(feature = "simd-is-enabled")]
                    detect_proximity_simd: super::detect_proximity_ball_ball_simd,
                    detect_proximity: super::detect_proximity_ball_ball,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Cuboid(_), Shape::Cuboid(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_cuboid_cuboid,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Polygon(_), Shape::Polygon(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_polygon_polygon,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Triangle(_), Shape::Ball(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Ball(_), Shape::Triangle(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Cuboid(_), Shape::Ball(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Ball(_), Shape::Cuboid(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_ball_convex,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Triangle(_), Shape::Cuboid(_)) => (
                PrimitiveProximityDetector {
                    detect_proximity: super::detect_proximity_cuboid_triangle,
                    ..PrimitiveProximityDetector::default()
                },
                None,
            ),
            (Shape::Cuboid(_), Shape::Triangle(_)) => (
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
        shape1: &Shape,
        shape2: &Shape,
    ) -> (ProximityPhase, Option<Box<dyn Any + Send + Sync>>) {
        match (shape1, shape2) {
            (Shape::Trimesh(_), _) => (
                ProximityPhase::NearPhase(ProximityDetector {
                    detect_proximity: super::detect_proximity_trimesh_shape,
                    ..ProximityDetector::default()
                }),
                Some(Box::new(TrimeshShapeProximityDetectorWorkspace::new())),
            ),
            (_, Shape::Trimesh(_)) => (
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
