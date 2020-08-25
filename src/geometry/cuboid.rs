#[cfg(feature = "dim3")]
use crate::geometry::PolyhedronFace;
use crate::geometry::{Cuboid, CuboidFeature, CuboidFeatureFace};
use crate::math::{Point, Vector};
use crate::utils::WSign;

pub fn local_support_point(cube: &Cuboid, local_dir: Vector<f32>) -> Point<f32> {
    local_dir.copy_sign_to(cube.half_extents).into()
}

// #[cfg(feature = "dim2")]
// pub fn polygon_ref(
//     cuboid: Cuboid,
//     out_vertices: &mut [Point<f32>; 4],
//     out_normals: &mut [Vector<f32>; 4],
// ) -> PolygonRef {
//     *out_vertices = [
//         Point::new(cuboid.half_extents.x, -cuboid.half_extents.y),
//         Point::new(cuboid.half_extents.x, cuboid.half_extents.y),
//         Point::new(-cuboid.half_extents.x, cuboid.half_extents.y),
//         Point::new(-cuboid.half_extents.x, -cuboid.half_extents.y),
//     ];
//     *out_normals = [Vector::x(), Vector::y(), -Vector::x(), -Vector::y()];
//
//     PolygonRef {
//         vertices: &out_vertices[..],
//         normals: &out_normals[..],
//     }
// }

#[cfg(feature = "dim2")]
pub fn vertex_feature_id(vertex: Point<f32>) -> u8 {
    ((vertex.x.to_bits() >> 31) & 0b001 | (vertex.y.to_bits() >> 30) & 0b010) as u8
}

// #[cfg(feature = "dim3")]
// pub fn vertex_feature_id(vertex: Point<f32>) -> u8 {
//     ((vertex.x.to_bits() >> 31) & 0b001
//         | (vertex.y.to_bits() >> 30) & 0b010
//         | (vertex.z.to_bits() >> 29) & 0b100) as u8
// }

#[cfg(feature = "dim3")]
pub fn polyhedron_support_face(cube: &Cuboid, local_dir: Vector<f32>) -> PolyhedronFace {
    support_face(cube, local_dir).into()
}

#[cfg(feature = "dim2")]
pub(crate) fn support_feature(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeature {
    // In 2D, it is best for stability to always return a face.
    // It won't have any notable impact on performances anyway.
    CuboidFeature::Face(support_face(cube, local_dir))

    /*
    let amax = local_dir.amax();

    const MAX_DOT_THRESHOLD: f32 = 0.98480775301; // 10 degrees.

    if amax > MAX_DOT_THRESHOLD {
        // Support face.
        CuboidFeature::Face(cube.support_face(local_dir))
    } else {
        // Support vertex
        CuboidFeature::Vertex(cube.support_vertex(local_dir))
    }
    */
}

#[cfg(feature = "dim3")]
pub(crate) fn support_feature(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeature {
    CuboidFeature::Face(support_face(cube, local_dir))
    /*
    const MAX_DOT_THRESHOLD: f32 = crate::utils::COS_10_DEGREES;
    const MIN_DOT_THRESHOLD: f32 = 1.0 - MAX_DOT_THRESHOLD;

    let amax = local_dir.amax();
    let amin = local_dir.amin();

    if amax > MAX_DOT_THRESHOLD {
        // Support face.
        CuboidFeature::Face(support_face(cube, local_dir))
    } else if amin < MIN_DOT_THRESHOLD {
        // Support edge.
        CuboidFeature::Edge(support_edge(cube, local_dir))
    } else {
        // Support vertex.
        CuboidFeature::Vertex(support_vertex(cube, local_dir))
    }
    */
}

// #[cfg(feature = "dim3")]
// pub(crate) fn support_vertex(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeatureVertex {
//     let vertex = local_support_point(cube, local_dir);
//     let vid = vertex_feature_id(vertex);
//
//     CuboidFeatureVertex { vertex, vid }
// }

// #[cfg(feature = "dim3")]
// pub(crate) fn support_edge(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeatureEdge {
//     let he = cube.half_extents;
//     let i = local_dir.iamin();
//     let j = (i + 1) % 3;
//     let k = (i + 2) % 3;
//     let mut a = Point::origin();
//     a[i] = he[i];
//     a[j] = local_dir[j].copy_sign_to(he[j]);
//     a[k] = local_dir[k].copy_sign_to(he[k]);
//
//     let mut b = a;
//     b[i] = -he[i];
//
//     let vid1 = vertex_feature_id(a);
//     let vid2 = vertex_feature_id(b);
//     let eid = (vid1.max(vid2) << 3) | vid1.min(vid2) | 0b11_000_000;
//
//     CuboidFeatureEdge {
//         vertices: [a, b],
//         vids: [vid1, vid2],
//         eid,
//     }
// }

#[cfg(feature = "dim2")]
pub fn support_face(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeatureFace {
    let he = cube.half_extents;
    let i = local_dir.iamin();
    let j = (i + 1) % 2;
    let mut a = Point::origin();
    a[i] = he[i];
    a[j] = local_dir[j].copy_sign_to(he[j]);

    let mut b = a;
    b[i] = -he[i];

    let vid1 = vertex_feature_id(a);
    let vid2 = vertex_feature_id(b);
    let fid = (vid1.max(vid2) << 2) | vid1.min(vid2) | 0b11_00_00;

    CuboidFeatureFace {
        vertices: [a, b],
        vids: [vid1, vid2],
        fid,
    }
}

#[cfg(feature = "dim3")]
pub(crate) fn support_face(cube: &Cuboid, local_dir: Vector<f32>) -> CuboidFeatureFace {
    // NOTE: can we use the orthonormal basis of local_dir
    // to make this AoSoA friendly?
    let he = cube.half_extents;
    let iamax = local_dir.iamax();
    let sign = local_dir[iamax].copy_sign_to(1.0);

    let vertices = match iamax {
        0 => [
            Point::new(he.x * sign, he.y, he.z),
            Point::new(he.x * sign, -he.y, he.z),
            Point::new(he.x * sign, -he.y, -he.z),
            Point::new(he.x * sign, he.y, -he.z),
        ],
        1 => [
            Point::new(he.x, he.y * sign, he.z),
            Point::new(-he.x, he.y * sign, he.z),
            Point::new(-he.x, he.y * sign, -he.z),
            Point::new(he.x, he.y * sign, -he.z),
        ],
        2 => [
            Point::new(he.x, he.y, he.z * sign),
            Point::new(he.x, -he.y, he.z * sign),
            Point::new(-he.x, -he.y, he.z * sign),
            Point::new(-he.x, he.y, he.z * sign),
        ],
        _ => unreachable!(),
    };

    pub fn vid(i: u8) -> u8 {
        // Each vertex has an even feature id.
        i * 2
    }

    let sign_index = ((sign as i8 + 1) / 2) as usize;
    // The vertex id as numbered depending on the sign of the vertex
    // component. A + sign means the corresponding bit is 0 while a -
    // sign means the corresponding bit is 1.
    // For exampl the vertex [2.0, -1.0, -3.0] has the id 0b011
    let vids = match iamax {
        0 => [
            [vid(0b000), vid(0b010), vid(0b011), vid(0b001)],
            [vid(0b100), vid(0b110), vid(0b111), vid(0b101)],
        ][sign_index],
        1 => [
            [vid(0b000), vid(0b100), vid(0b101), vid(0b001)],
            [vid(0b010), vid(0b110), vid(0b111), vid(0b011)],
        ][sign_index],
        2 => [
            [vid(0b000), vid(0b010), vid(0b110), vid(0b100)],
            [vid(0b001), vid(0b011), vid(0b111), vid(0b101)],
        ][sign_index],
        _ => unreachable!(),
    };

    // The feature ids of edges is obtained from the vertex ids
    // of their endpoints.
    // Assuming vid1 > vid2, we do:   (vid1 << 3) | vid2 | 0b11000000
    //
    let eids = match iamax {
        0 => [
            [0b11_010_000, 0b11_011_010, 0b11_011_001, 0b11_001_000],
            [0b11_110_100, 0b11_111_110, 0b11_111_101, 0b11_101_100],
        ][sign_index],
        1 => [
            [0b11_100_000, 0b11_101_100, 0b11_101_001, 0b11_001_000],
            [0b11_110_010, 0b11_111_110, 0b11_111_011, 0b11_011_010],
        ][sign_index],
        2 => [
            [0b11_010_000, 0b11_110_010, 0b11_110_100, 0b11_100_000],
            [0b11_011_001, 0b11_111_011, 0b11_111_101, 0b11_101_001],
        ][sign_index],
        _ => unreachable!(),
    };

    // The face with normals [x, y, z] are numbered [10, 11, 12].
    // The face with negated normals are numbered [13, 14, 15].
    let fid = iamax + sign_index * 3 + 10;

    CuboidFeatureFace {
        vertices,
        vids,
        eids,
        fid: fid as u8,
    }
}
