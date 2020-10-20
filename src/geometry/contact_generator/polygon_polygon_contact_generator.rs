use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{sat, Contact, ContactManifold, KinematicsCategory, Polygon, Shape};
use crate::math::{Isometry, Point};
#[cfg(feature = "dim2")]
use crate::{math::Vector, utils};

pub fn generate_contacts_polygon_polygon(ctxt: &mut PrimitiveContactGenerationContext) {
    unimplemented!()
    // if let (Shape::Polygon(polygon1), Shape::Polygon(polygon2)) = (ctxt.shape1, ctxt.shape2) {
    //     generate_contacts(
    //         polygon1,
    //         &ctxt.position1,
    //         polygon2,
    //         &ctxt.position2,
    //         ctxt.manifold,
    //     );
    //     ctxt.manifold.update_warmstart_multiplier();
    // } else {
    //     unreachable!()
    // }
    //
    // ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}

fn generate_contacts<'a>(
    mut p1: &'a Polygon,
    mut m1: &'a Isometry<f32>,
    mut p2: &'a Polygon,
    mut m2: &'a Isometry<f32>,
    manifold: &'a mut ContactManifold,
) {
    let mut m12 = m1.inverse() * m2;
    let mut m21 = m12.inverse();

    if manifold.try_update_contacts(&m12) {
        return;
    }

    let mut sep1 = sat::polygon_polygon_compute_separation_features(p1, p2, &m12);
    if sep1.0 > 0.0 {
        manifold.points.clear();
        return;
    }

    let mut sep2 = sat::polygon_polygon_compute_separation_features(p2, p1, &m21);
    if sep2.0 > 0.0 {
        manifold.points.clear();
        return;
    }

    let mut swapped = false;
    if sep2.0 > sep1.0 {
        std::mem::swap(&mut sep1, &mut sep2);
        std::mem::swap(&mut m1, &mut m2);
        std::mem::swap(&mut p1, &mut p2);
        std::mem::swap(&mut m12, &mut m21);
        manifold.swap_identifiers();
        swapped = true;
    }

    let support_face1 = sep1.1;
    let local_n1 = p1.normals[support_face1];
    let local_n2 = m21 * -local_n1;
    let support_face2 = p2.support_face(&local_n2);
    let len1 = p1.vertices.len();
    let len2 = p2.vertices.len();

    let seg1 = (
        p1.vertices[support_face1],
        p1.vertices[(support_face1 + 1) % len1],
    );
    let seg2 = (
        m12 * p2.vertices[support_face2],
        m12 * p2.vertices[(support_face2 + 1) % len2],
    );
    if let Some((clip_a, clip_b)) = clip_segments(seg1, seg2) {
        let dist_a = (clip_a.1 - clip_a.0).dot(&local_n1);
        let dist_b = (clip_b.1 - clip_b.0).dot(&local_n1);

        let mut impulses_a = (0.0, Contact::zero_tangent_impulse());
        let mut impulses_b = (0.0, Contact::zero_tangent_impulse());

        let fids_a = (
            ((support_face1 * 2 + clip_a.2) % (len1 * 2)) as u8,
            ((support_face2 * 2 + clip_a.3) % (len2 * 2)) as u8,
        );

        let fids_b = (
            ((support_face1 * 2 + clip_b.2) % (len1 * 2)) as u8,
            ((support_face2 * 2 + clip_b.3) % (len2 * 2)) as u8,
        );

        if manifold.points.len() != 0 {
            assert_eq!(manifold.points.len(), 2);

            // We already had 2 points in the previous iteration.
            // Match the features to see if we keep the cached impulse.
            let original_fids_a;
            let original_fids_b;

            // NOTE: the previous manifold may have its bodies swapped wrt. our new manifold.
            // So we have to adjust accordingly the features we will be comparing.
            if swapped {
                original_fids_a = (manifold.points[0].fid1, manifold.points[0].fid2);
                original_fids_b = (manifold.points[1].fid1, manifold.points[1].fid2);
            } else {
                original_fids_a = (manifold.points[0].fid2, manifold.points[0].fid1);
                original_fids_b = (manifold.points[1].fid2, manifold.points[1].fid1);
            }

            if fids_a == original_fids_a {
                impulses_a = (
                    manifold.points[0].impulse,
                    manifold.points[0].tangent_impulse,
                );
            } else if fids_a == original_fids_b {
                impulses_a = (
                    manifold.points[1].impulse,
                    manifold.points[1].tangent_impulse,
                );
            }
            if fids_b == original_fids_a {
                impulses_b = (
                    manifold.points[0].impulse,
                    manifold.points[0].tangent_impulse,
                );
            } else if fids_b == original_fids_b {
                impulses_b = (
                    manifold.points[1].impulse,
                    manifold.points[1].tangent_impulse,
                );
            }
        }

        manifold.points.clear();
        manifold.points.push(Contact {
            local_p1: clip_a.0,
            local_p2: m21 * clip_a.1,
            impulse: impulses_a.0,
            tangent_impulse: impulses_a.1,
            fid1: fids_a.0,
            fid2: fids_a.1,
            dist: dist_a,
        });

        manifold.points.push(Contact {
            local_p1: clip_b.0,
            local_p2: m21 * clip_b.1,
            impulse: impulses_b.0,
            tangent_impulse: impulses_b.1,
            fid1: fids_b.0,
            fid2: fids_b.1,
            dist: dist_b,
        });

        manifold.local_n1 = local_n1;
        manifold.local_n2 = local_n2;
        manifold.kinematics.category = KinematicsCategory::PlanePoint;
        manifold.kinematics.radius1 = 0.0;
        manifold.kinematics.radius2 = 0.0;
    } else {
        manifold.points.clear();
    }
}

// Features in clipping points are:
// 0 = First vertex.
// 1 = On the face.
// 2 = Second vertex.
pub(crate) type ClippingPoints = (Point<f32>, Point<f32>, usize, usize);

#[cfg(feature = "dim2")]
pub(crate) fn clip_segments_with_normal(
    mut seg1: (Point<f32>, Point<f32>),
    mut seg2: (Point<f32>, Point<f32>),
    normal: Vector<f32>,
) -> Option<(ClippingPoints, ClippingPoints)> {
    use crate::utils::WBasis;
    let tangent = normal.orthonormal_basis()[0];

    let mut range1 = [seg1.0.coords.dot(&tangent), seg1.1.coords.dot(&tangent)];
    let mut range2 = [seg2.0.coords.dot(&tangent), seg2.1.coords.dot(&tangent)];
    let mut features1 = [0, 2];
    let mut features2 = [0, 2];

    if range1[1] < range1[0] {
        range1.swap(0, 1);
        features1.swap(0, 1);
        std::mem::swap(&mut seg1.0, &mut seg1.1);
    }

    if range2[1] < range2[0] {
        range2.swap(0, 1);
        features2.swap(0, 1);
        std::mem::swap(&mut seg2.0, &mut seg2.1);
    }

    if range2[0] > range1[1] || range1[0] > range2[1] {
        // No clip point.
        return None;
    }

    let ca = if range2[0] > range1[0] {
        let bcoord = (range2[0] - range1[0]) * utils::inv(range1[1] - range1[0]);
        let p1 = seg1.0 + (seg1.1 - seg1.0) * bcoord;
        let p2 = seg2.0;

        (p1, p2, 1, features2[0])
    } else {
        let bcoord = (range1[0] - range2[0]) * utils::inv(range2[1] - range2[0]);
        let p1 = seg1.0;
        let p2 = seg2.0 + (seg2.1 - seg2.0) * bcoord;

        (p1, p2, features1[0], 1)
    };

    let cb = if range2[1] < range1[1] {
        let bcoord = (range2[1] - range1[0]) * utils::inv(range1[1] - range1[0]);
        let p1 = seg1.0 + (seg1.1 - seg1.0) * bcoord;
        let p2 = seg2.1;

        (p1, p2, 1, features2[1])
    } else {
        let bcoord = (range1[1] - range2[0]) * utils::inv(range2[1] - range2[0]);
        let p1 = seg1.1;
        let p2 = seg2.0 + (seg2.1 - seg2.0) * bcoord;

        (p1, p2, features1[1], 1)
    };

    Some((ca, cb))
}

pub(crate) fn clip_segments(
    mut seg1: (Point<f32>, Point<f32>),
    mut seg2: (Point<f32>, Point<f32>),
) -> Option<(ClippingPoints, ClippingPoints)> {
    // NOTE: no need to normalize the tangent.
    let tangent1 = seg1.1 - seg1.0;
    let sqnorm_tangent1 = tangent1.norm_squared();

    let mut range1 = [0.0, sqnorm_tangent1];
    let mut range2 = [
        (seg2.0 - seg1.0).dot(&tangent1),
        (seg2.1 - seg1.0).dot(&tangent1),
    ];
    let mut features1 = [0, 2];
    let mut features2 = [0, 2];

    if range1[1] < range1[0] {
        range1.swap(0, 1);
        features1.swap(0, 1);
        std::mem::swap(&mut seg1.0, &mut seg1.1);
    }

    if range2[1] < range2[0] {
        range2.swap(0, 1);
        features2.swap(0, 1);
        std::mem::swap(&mut seg2.0, &mut seg2.1);
    }

    if range2[0] > range1[1] || range1[0] > range2[1] {
        // No clip point.
        return None;
    }

    let length1 = range1[1] - range1[0];
    let length2 = range2[1] - range2[0];

    let ca = if range2[0] > range1[0] {
        let bcoord = (range2[0] - range1[0]) / length1;
        let p1 = seg1.0 + tangent1 * bcoord;
        let p2 = seg2.0;

        (p1, p2, 1, features2[0])
    } else {
        let bcoord = (range1[0] - range2[0]) / length2;
        let p1 = seg1.0;
        let p2 = seg2.0 + (seg2.1 - seg2.0) * bcoord;

        (p1, p2, features1[0], 1)
    };

    let cb = if range2[1] < range1[1] {
        let bcoord = (range2[1] - range1[0]) / length1;
        let p1 = seg1.0 + tangent1 * bcoord;
        let p2 = seg2.1;

        (p1, p2, 1, features2[1])
    } else {
        let bcoord = (range1[1] - range2[0]) / length2;
        let p1 = seg1.1;
        let p2 = seg2.0 + (seg2.1 - seg2.0) * bcoord;

        (p1, p2, features1[1], 1)
    };

    Some((ca, cb))
}
