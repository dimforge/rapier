use crate::geometry::{Cuboid, Polygon, Segment, Triangle};
use crate::math::{Isometry, Point, Vector, DIM};
use crate::utils::WSign;
use na::Unit;

#[allow(dead_code)]
pub fn polygon_polygon_compute_separation_features(
    p1: &Polygon,
    p2: &Polygon,
    m12: &Isometry<f32>,
) -> (f32, usize, usize) {
    let mut max_separation = -f32::MAX;
    let mut separation_features = (0, 0);

    for (i, (p1, n1)) in p1.vertices.iter().zip(p1.normals.iter()).enumerate() {
        let j = p2.support_point(&m12.inverse_transform_vector(&-n1));
        let dpt = m12 * p2.vertices[j] - p1;
        let separation = dpt.dot(n1);

        if separation > max_separation {
            max_separation = separation;
            separation_features = (i, j);
        }
    }

    (max_separation, separation_features.0, separation_features.1)
}
