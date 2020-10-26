use crate::geometry::{cuboid, Cuboid, Polygon, Triangle};
use crate::math::{Isometry, Point, Vector, DIM};
use crate::utils::WSign;
use na::Unit;
use ncollide::shape::{Segment, SupportMap};

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

#[cfg(feature = "dim3")]
pub fn cuboid_cuboid_compute_separation_wrt_local_line(
    cube1: &Cuboid,
    cube2: &Cuboid,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
    axis1: &Vector<f32>,
) -> (f32, Vector<f32>) {
    let signum = pos12.translation.vector.dot(axis1).copy_sign_to(1.0);
    let axis1 = axis1 * signum;
    let local_pt1 = cuboid::local_support_point(cube1, axis1);
    let local_pt2 = cuboid::local_support_point(cube2, pos21 * -axis1);
    let pt2 = pos12 * local_pt2;
    let separation = (pt2 - local_pt1).dot(&axis1);
    (separation, axis1)
}

#[cfg(feature = "dim3")]
pub fn cuboid_cuboid_find_local_separating_edge_twoway(
    cube1: &Cuboid,
    cube2: &Cuboid,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    use approx::AbsDiffEq;
    let mut best_separation = -std::f32::MAX;
    let mut best_dir = Vector::zeros();

    let x2 = pos12 * Vector::x();
    let y2 = pos12 * Vector::y();
    let z2 = pos12 * Vector::z();

    // We have 3 * 3 = 9 axes to test.
    let axes = [
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -x2.z, x2.y),
        Vector::new(x2.z, 0.0, -x2.x),
        Vector::new(-x2.y, x2.x, 0.0),
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -y2.z, y2.y),
        Vector::new(y2.z, 0.0, -y2.x),
        Vector::new(-y2.y, y2.x, 0.0),
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -z2.z, z2.y),
        Vector::new(z2.z, 0.0, -z2.x),
        Vector::new(-z2.y, z2.x, 0.0),
    ];

    for axis1 in &axes {
        let norm1 = axis1.norm();
        if norm1 > f32::default_epsilon() {
            let (separation, axis1) = cuboid_cuboid_compute_separation_wrt_local_line(
                cube1,
                cube2,
                pos12,
                pos21,
                &(axis1 / norm1),
            );

            if separation > best_separation {
                best_separation = separation;
                best_dir = axis1;
            }
        }
    }

    (best_separation, best_dir)
}

pub fn cuboid_cuboid_find_local_separating_normal_oneway(
    cube1: &Cuboid,
    cube2: &Cuboid,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    let mut best_separation = -std::f32::MAX;
    let mut best_dir = Vector::zeros();

    for i in 0..DIM {
        let sign = pos12.translation.vector[i].copy_sign_to(1.0);
        let axis1 = Vector::ith(i, sign);
        let local_pt2 = cuboid::local_support_point(cube2, pos21 * -axis1);
        let pt2 = pos12 * local_pt2;
        let separation = pt2[i] * sign - cube1.half_extents[i];

        if separation > best_separation {
            best_separation = separation;
            best_dir = axis1;
        }
    }

    (best_separation, best_dir)
}

/*
 *
 *
 * Triangles.
 *
 *
 */

#[cfg(feature = "dim3")]
pub fn cube_support_map_compute_separation_wrt_local_line<S: SupportMap<f32>>(
    cube1: &Cuboid,
    shape2: &S,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
    axis1: &Unit<Vector<f32>>,
) -> (f32, Unit<Vector<f32>>) {
    let signum = pos12.translation.vector.dot(axis1).copy_sign_to(1.0);
    let axis1 = Unit::new_unchecked(**axis1 * signum);
    let local_pt1 = cuboid::local_support_point(cube1, *axis1);
    let local_pt2 = shape2.local_support_point_toward(&(pos21 * -axis1));
    let pt2 = pos12 * local_pt2;
    let separation = (pt2 - local_pt1).dot(&axis1);
    (separation, axis1)
}

#[cfg(feature = "dim3")]
pub fn cube_support_map_find_local_separating_edge_twoway(
    cube1: &Cuboid,
    shape2: &impl SupportMap<f32>,
    axes: &[Vector<f32>],
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    use approx::AbsDiffEq;
    let mut best_separation = -std::f32::MAX;
    let mut best_dir = Vector::zeros();

    for axis1 in axes {
        if let Some(axis1) = Unit::try_new(*axis1, f32::default_epsilon()) {
            let (separation, axis1) = cube_support_map_compute_separation_wrt_local_line(
                cube1, shape2, pos12, pos21, &axis1,
            );

            if separation > best_separation {
                best_separation = separation;
                best_dir = *axis1;
            }
        }
    }

    (best_separation, best_dir)
}

#[cfg(feature = "dim3")]
pub fn cube_triangle_find_local_separating_edge_twoway(
    cube1: &Cuboid,
    triangle2: &Triangle,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    let x2 = pos12 * (triangle2.b - triangle2.a);
    let y2 = pos12 * (triangle2.c - triangle2.b);
    let z2 = pos12 * (triangle2.a - triangle2.c);

    // We have 3 * 3 = 3 axes to test.
    let axes = [
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -x2.z, x2.y),
        Vector::new(x2.z, 0.0, -x2.x),
        Vector::new(-x2.y, x2.x, 0.0),
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -y2.z, y2.y),
        Vector::new(y2.z, 0.0, -y2.x),
        Vector::new(-y2.y, y2.x, 0.0),
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -z2.z, z2.y),
        Vector::new(z2.z, 0.0, -z2.x),
        Vector::new(-z2.y, z2.x, 0.0),
    ];

    cube_support_map_find_local_separating_edge_twoway(cube1, triangle2, &axes, pos12, pos21)
}

#[cfg(feature = "dim3")]
pub fn cube_segment_find_local_separating_edge_twoway(
    cube1: &Cuboid,
    segment2: &Segment<f32>,
    pos12: &Isometry<f32>,
    pos21: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    let x2 = pos12 * (segment2.b - segment2.a);

    let axes = [
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(0.0, -x2.z, x2.y),
        Vector::new(x2.z, 0.0, -x2.x),
        Vector::new(-x2.y, x2.x, 0.0),
    ];

    cube_support_map_find_local_separating_edge_twoway(cube1, segment2, &axes, pos12, pos21)
}

pub fn cube_support_map_find_local_separating_normal_oneway<S: SupportMap<f32>>(
    cube1: &Cuboid,
    shape2: &S,
    pos12: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    let mut best_separation = -std::f32::MAX;
    let mut best_dir = Vector::zeros();

    for i in 0..DIM {
        for sign in &[-1.0, 1.0] {
            let axis1 = Vector::ith(i, *sign);
            let pt2 = shape2.support_point_toward(&pos12, &Unit::new_unchecked(-axis1));
            let separation = pt2[i] * *sign - cube1.half_extents[i];

            if separation > best_separation {
                best_separation = separation;
                best_dir = axis1;
            }
        }
    }

    (best_separation, best_dir)
}

// NOTE: this only works with cuboid on the rhs because it has its symmetry origin at zero
// (therefore we can check only one normal direction).
pub fn point_cuboid_find_local_separating_normal_oneway(
    point1: Point<f32>,
    normal1: Option<Unit<Vector<f32>>>,
    shape2: &Cuboid,
    pos12: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    let mut best_separation = -std::f32::MAX;
    let mut best_dir = Vector::zeros();

    if let Some(normal1) = normal1 {
        let axis1 = if (pos12.translation.vector - point1.coords).dot(&normal1) >= 0.0 {
            normal1
        } else {
            -normal1
        };

        let pt2 = shape2.support_point_toward(&pos12, &-axis1);
        let separation = (pt2 - point1).dot(&axis1);

        if separation > best_separation {
            best_separation = separation;
            best_dir = *axis1;
        }
    }

    (best_separation, best_dir)
}

pub fn triangle_cuboid_find_local_separating_normal_oneway(
    triangle1: &Triangle,
    shape2: &Cuboid,
    pos12: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    point_cuboid_find_local_separating_normal_oneway(triangle1.a, triangle1.normal(), shape2, pos12)
}

#[cfg(feature = "dim2")]
pub fn segment_cuboid_find_local_separating_normal_oneway(
    segment1: &Segment<f32>,
    shape2: &Cuboid,
    pos12: &Isometry<f32>,
) -> (f32, Vector<f32>) {
    point_cuboid_find_local_separating_normal_oneway(segment1.a, segment1.normal(), shape2, pos12)
}
