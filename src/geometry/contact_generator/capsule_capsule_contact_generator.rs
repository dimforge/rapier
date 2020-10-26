use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{Capsule, Contact, ContactManifold, KinematicsCategory};
use crate::math::Isometry;
use crate::math::Vector;
use approx::AbsDiffEq;
use na::Unit;
#[cfg(feature = "dim2")]
use ncollide::shape::SegmentPointLocation;

pub fn generate_contacts_capsule_capsule(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Some(capsule1), Some(capsule2)) = (ctxt.shape1.as_capsule(), ctxt.shape2.as_capsule()) {
        generate_contacts(
            ctxt.prediction_distance,
            capsule1,
            ctxt.position1,
            capsule2,
            ctxt.position2,
            ctxt.manifold,
        );
    }

    ctxt.manifold.update_warmstart_multiplier();
    ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}

#[cfg(feature = "dim2")]
pub fn generate_contacts<'a>(
    prediction_distance: f32,
    capsule1: &'a Capsule,
    pos1: &'a Isometry<f32>,
    capsule2: &'a Capsule,
    pos2: &'a Isometry<f32>,
    manifold: &mut ContactManifold,
) {
    // FIXME: the contact kinematics is not correctly set here.
    // We use the common "Point-Plane" kinematics with zero radius everytime.
    // Instead we should select point/point ore point-plane (with non-zero
    // radius for the point) depending on the features involved in the contact.
    let pos12 = pos1.inverse() * pos2;
    let pos21 = pos12.inverse();

    let seg1 = capsule1.segment;
    let seg2_1 = capsule2.segment.transformed(&pos12);
    let (loc1, loc2) = ncollide::query::closest_points_segment_segment_with_locations_nD(
        (&seg1.a, &seg1.b),
        (&seg2_1.a, &seg2_1.b),
    );

    // We do this clone to perform contact tracking and transfer impulses.
    // FIXME: find a more efficient way of doing this.
    let old_manifold_points = manifold.points.clone();
    manifold.points.clear();

    let swapped = false;

    let fid1 = if let SegmentPointLocation::OnVertex(v1) = loc1 {
        v1 as u8 * 2
    } else {
        1
    };
    let fid2 = if let SegmentPointLocation::OnVertex(v2) = loc2 {
        v2 as u8 * 2
    } else {
        1
    };

    let bcoords1 = loc1.barycentric_coordinates();
    let bcoords2 = loc2.barycentric_coordinates();
    let local_p1 = seg1.a * bcoords1[0] + seg1.b.coords * bcoords1[1];
    let local_p2 = seg2_1.a * bcoords2[0] + seg2_1.b.coords * bcoords2[1];

    let local_n1 =
        Unit::try_new(local_p2 - local_p1, f32::default_epsilon()).unwrap_or(Vector::y_axis());
    let dist = (local_p2 - local_p1).dot(&local_n1) - capsule1.radius - capsule2.radius;

    if dist <= prediction_distance {
        let local_n2 = pos21 * -local_n1;
        let contact = Contact::new(local_p1, pos21 * local_p2, fid1, fid2, dist);
        manifold.points.push(contact);

        manifold.local_n1 = *local_n1;
        manifold.local_n2 = *local_n2;
        manifold.kinematics.category = KinematicsCategory::PlanePoint;
        manifold.kinematics.radius1 = 0.0;
        manifold.kinematics.radius2 = 0.0;
    } else {
        // No contact within tolerance.
        return;
    }

    if let (Some(dir1), Some(dir2)) = (seg1.direction(), seg2_1.direction()) {
        if dir1.dot(&dir2).abs() >= crate::utils::COS_FRAC_PI_8
            && dir1.dot(&local_n1).abs() < crate::utils::SIN_FRAC_PI_8
        {
            // Capsules axes are almost parallel and are almost perpendicular to the normal.
            // Find a second contact point.
            if let Some((clip_a, clip_b)) = crate::geometry::clip_segments_with_normal(
                (seg1.a, seg1.b),
                (seg2_1.a, seg2_1.b),
                *local_n1,
            ) {
                let contact =
                    if (clip_a.0 - local_p1).norm_squared() > f32::default_epsilon() * 100.0 {
                        // Use clip_a as the second contact.
                        Contact::new(
                            clip_a.0,
                            pos21 * clip_a.1,
                            clip_a.2 as u8,
                            clip_a.3 as u8,
                            (clip_a.1 - clip_a.0).dot(&local_n1),
                        )
                    } else {
                        // Use clip_b as the second contact.
                        Contact::new(
                            clip_b.0,
                            pos21 * clip_b.1,
                            clip_b.2 as u8,
                            clip_b.3 as u8,
                            (clip_b.1 - clip_b.0).dot(&local_n1),
                        )
                    };

                manifold.points.push(contact);
            }
        }
    }

    if swapped {
        for point in &mut manifold.points {
            point.local_p1 += manifold.local_n1 * capsule2.radius;
            point.local_p2 += manifold.local_n2 * capsule1.radius;
            point.dist -= capsule1.radius + capsule2.radius;
        }
    } else {
        for point in &mut manifold.points {
            point.local_p1 += manifold.local_n1 * capsule1.radius;
            point.local_p2 += manifold.local_n2 * capsule2.radius;
            point.dist -= capsule1.radius + capsule2.radius;
        }
    }

    super::match_contacts(manifold, &old_manifold_points, swapped);
}

#[cfg(feature = "dim3")]
pub fn generate_contacts<'a>(
    prediction_distance: f32,
    capsule1: &'a Capsule,
    pos1: &'a Isometry<f32>,
    capsule2: &'a Capsule,
    pos2: &'a Isometry<f32>,
    manifold: &mut ContactManifold,
) {
    let pos12 = pos1.inverse() * pos2;
    let pos21 = pos12.inverse();

    let seg1 = capsule1.segment;
    let seg2_1 = capsule2.segment.transformed(&pos12);
    let (loc1, loc2) = ncollide::query::closest_points_segment_segment_with_locations_nD(
        (&seg1.a, &seg1.b),
        (&seg2_1.a, &seg2_1.b),
    );

    {
        let bcoords1 = loc1.barycentric_coordinates();
        let bcoords2 = loc2.barycentric_coordinates();
        let local_p1 = seg1.a * bcoords1[0] + seg1.b.coords * bcoords1[1];
        let local_p2 = seg2_1.a * bcoords2[0] + seg2_1.b.coords * bcoords2[1];

        let local_n1 =
            Unit::try_new(local_p2 - local_p1, f32::default_epsilon()).unwrap_or(Vector::y_axis());
        let dist = (local_p2 - local_p1).dot(&local_n1) - capsule1.radius - capsule2.radius;

        if dist <= prediction_distance {
            let local_n2 = pos21 * -local_n1;
            let contact = Contact::new(
                local_p1 + *local_n1 * capsule1.radius,
                pos21 * local_p2 + *local_n2 * capsule2.radius,
                0,
                0,
                dist,
            );

            if manifold.points.len() != 0 {
                manifold.points[0].copy_geometry_from(contact);
            } else {
                manifold.points.push(contact);
            }

            manifold.local_n1 = *local_n1;
            manifold.local_n2 = *local_n2;
            manifold.kinematics.category = KinematicsCategory::PlanePoint;
            manifold.kinematics.radius1 = 0.0;
            manifold.kinematics.radius2 = 0.0;
        } else {
            manifold.points.clear();
        }
    }
}
