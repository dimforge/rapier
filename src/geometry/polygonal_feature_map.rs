use crate::geometry::PolyhedronFace;
use crate::geometry::{cuboid, Cuboid, Cylinder, Triangle};
use crate::math::{Point, Vector};
use approx::AbsDiffEq;
use na::{Unit, Vector2, Vector3};
use ncollide::shape::Segment;
use ncollide::shape::SupportMap;

/// Trait implemented by convex shapes with features with polyhedral approximations.
pub trait PolygonalFeatureMap: SupportMap<f32> {
    fn local_support_feature(&self, dir: &Unit<Vector<f32>>, out_feature: &mut PolyhedronFace);
}

impl PolygonalFeatureMap for Segment<f32> {
    fn local_support_feature(&self, _: &Unit<Vector<f32>>, out_feature: &mut PolyhedronFace) {
        *out_feature = PolyhedronFace::from(*self);
    }
}

impl PolygonalFeatureMap for Triangle {
    fn local_support_feature(&self, _: &Unit<Vector<f32>>, out_feature: &mut PolyhedronFace) {
        *out_feature = PolyhedronFace::from(*self);
    }
}

impl PolygonalFeatureMap for Cuboid {
    fn local_support_feature(&self, dir: &Unit<Vector<f32>>, out_feature: &mut PolyhedronFace) {
        let face = cuboid::support_face(self, **dir);
        *out_feature = PolyhedronFace::from(face);
    }
}

impl PolygonalFeatureMap for Cylinder {
    fn local_support_feature(&self, dir: &Unit<Vector<f32>>, out_features: &mut PolyhedronFace) {
        let dir2 = Vector2::new(dir.x, dir.z)
            .try_normalize(f32::default_epsilon())
            .unwrap_or(Vector2::x());

        if dir.y.abs() < 0.5 {
            // We return a segment lying on the cylinder's curved part.
            out_features.vertices[0] = Point::new(
                dir2.x * self.radius,
                -self.half_height,
                dir2.y * self.radius,
            );
            out_features.vertices[1] =
                Point::new(dir2.x * self.radius, self.half_height, dir2.y * self.radius);
            out_features.eids = [0, 0, 0, 0]; // FIXME
            out_features.fid = 1;
            out_features.num_vertices = 2;
            out_features.vids = [0, 1, 1, 1]; // FIXME
        } else {
            // We return a square approximation of the cylinder cap.
            let y = self.half_height.copysign(dir.y);
            out_features.vertices[0] = Point::new(dir2.x * self.radius, y, dir2.y * self.radius);
            out_features.vertices[1] = Point::new(-dir2.y * self.radius, y, dir2.x * self.radius);
            out_features.vertices[2] = Point::new(-dir2.x * self.radius, y, -dir2.y * self.radius);
            out_features.vertices[3] = Point::new(dir2.y * self.radius, y, -dir2.x * self.radius);
            out_features.eids = [0, 1, 2, 3]; // FIXME
            out_features.fid = if dir.y < 0.0 { 0 } else { 2 };
            out_features.num_vertices = 4;
            out_features.vids = [0, 1, 2, 3]; // FIXME
        }
    }
}
