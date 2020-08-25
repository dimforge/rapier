pub use self::ball_ball_proximity_detector::detect_proximity_ball_ball;
#[cfg(feature = "simd-is-enabled")]
pub use self::ball_ball_proximity_detector::detect_proximity_ball_ball_simd;
pub use self::ball_convex_proximity_detector::detect_proximity_ball_convex;
pub use self::cuboid_cuboid_proximity_detector::detect_proximity_cuboid_cuboid;
pub use self::cuboid_triangle_proximity_detector::detect_proximity_cuboid_triangle;
pub use self::polygon_polygon_proximity_detector::detect_proximity_polygon_polygon;
pub use self::proximity_detector::{
    PrimitiveProximityDetectionContext, PrimitiveProximityDetector, ProximityDetectionContext,
    ProximityDetector, ProximityPhase,
};
#[cfg(feature = "simd-is-enabled")]
pub use self::proximity_detector::{
    PrimitiveProximityDetectionContextSimd, ProximityDetectionContextSimd,
};
pub use self::proximity_dispatcher::{DefaultProximityDispatcher, ProximityDispatcher};
pub use self::trimesh_shape_proximity_detector::{
    detect_proximity_trimesh_shape, TrimeshShapeProximityDetectorWorkspace,
};

mod ball_ball_proximity_detector;
mod ball_convex_proximity_detector;
mod ball_polygon_proximity_detector;
mod cuboid_cuboid_proximity_detector;
mod cuboid_polygon_proximity_detector;
mod cuboid_triangle_proximity_detector;
mod polygon_polygon_proximity_detector;
mod proximity_detector;
mod proximity_dispatcher;
mod trimesh_shape_proximity_detector;
