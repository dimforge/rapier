use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
#[cfg(feature = "dim3")]
use crate::geometry::PolyhedronFace;
use crate::geometry::{cuboid, sat, Capsule, ContactManifold, Cuboid, KinematicsCategory};
#[cfg(feature = "dim2")]
use crate::geometry::{CuboidFeature, CuboidFeatureFace};
use crate::math::Isometry;
use crate::math::Vector;

pub fn generate_contacts_cuboid_capsule(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Some(cube1), Some(capsule2)) = (ctxt.shape1.as_cuboid(), ctxt.shape2.as_capsule()) {
        generate_contacts(
            ctxt.prediction_distance,
            cube1,
            capsule2,
            ctxt.manifold,
            false,
        );
        ctxt.manifold.update_warmstart_multiplier();
    } else if let (Some(capsule1), Some(cube2)) =
        (ctxt.shape1.as_capsule(), ctxt.shape2.as_cuboid())
    {
        generate_contacts(
            ctxt.prediction_distance,
            cube2,
            capsule1,
            ctxt.manifold,
            true,
        );
        ctxt.manifold.update_warmstart_multiplier();
    }
    ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}

pub fn generate_contacts<'a>(
    prediction_distance: f32,
    cube1: &'a Cuboid,
    capsule2: &'a Capsule,
    manifold: &mut ContactManifold,
    swapped: bool,
) {
    let mut pos12;
    let mut pos21;

    if !swapped {
        pos12 = manifold.position1.inverse() * manifold.position2;
        pos21 = pos12.inverse();

        if manifold.try_update_contacts(&pos12) {
            return;
        }
    } else {
        pos12 = manifold.position2.inverse() * manifold.position1;
        pos21 = pos12.inverse();

        if manifold.try_update_contacts(&pos21) {
            return;
        }
    }

    let segment2 = capsule2.segment;

    /*
     *
     * Point-Face cases.
     *
     */
    let sep1 = sat::cube_support_map_find_local_separating_normal_oneway(cube1, &segment2, &pos12);
    if sep1.0 > capsule2.radius + prediction_distance {
        manifold.points.clear();
        return;
    }

    #[cfg(feature = "dim3")]
    let sep2 = (-f32::MAX, Vector::x());
    #[cfg(feature = "dim2")]
    let sep2 = sat::segment_cuboid_find_local_separating_normal_oneway(&segment2, cube1, &pos21);
    if sep2.0 > capsule2.radius + prediction_distance {
        manifold.points.clear();
        return;
    }

    /*
     *
     * Edge-Edge cases.
     *
     */
    #[cfg(feature = "dim2")]
    let sep3 = (-f32::MAX, Vector::x()); // This case does not exist in 2D.
    #[cfg(feature = "dim3")]
    let sep3 =
        sat::cube_segment_find_local_separating_edge_twoway(cube1, &segment2, &pos12, &pos21);
    if sep3.0 > capsule2.radius + prediction_distance {
        manifold.points.clear();
        return;
    }

    /*
     *
     * Select the best combination of features
     * and get the polygons to clip.
     *
     */
    let mut swapped_reference = sep2.0 > sep1.0 && sep2.0 > sep3.0;
    let mut best_sep = sep1;

    if swapped_reference {
        // The reference shape will be the second shape.
        // std::mem::swap(&mut cube1, &mut capsule2);
        std::mem::swap(&mut pos12, &mut pos21);
        best_sep = sep2;
    } else if sep3.0 > sep1.0 {
        best_sep = sep3;
    }

    let feature1;
    let mut feature2;

    #[cfg(feature = "dim2")]
    {
        if swapped_reference {
            feature1 = CuboidFeatureFace::from(segment2);
            feature2 = cuboid::support_face(cube1, pos21 * -best_sep.1);
        } else {
            feature1 = cuboid::support_face(cube1, best_sep.1);
            feature2 = CuboidFeatureFace::from(segment2);
        }
    }
    #[cfg(feature = "dim3")]
    {
        if swapped_reference {
            feature1 = PolyhedronFace::from(segment2);
            feature2 = cuboid::polyhedron_support_face(cube1, pos21 * -best_sep.1);
        } else {
            feature1 = cuboid::polyhedron_support_face(cube1, best_sep.1);
            feature2 = PolyhedronFace::from(segment2);
        }
    }

    feature2.transform_by(&pos12);

    if swapped ^ swapped_reference {
        manifold.swap_identifiers();
    }

    // We do this clone to perform contact tracking and transfer impulses.
    // FIXME: find a more efficient way of doing this.
    let old_manifold_points = manifold.points.clone();
    manifold.points.clear();

    #[cfg(feature = "dim2")]
    CuboidFeature::face_face_contacts(
        prediction_distance + capsule2.radius,
        &feature1,
        &best_sep.1,
        &feature2,
        &pos21,
        manifold,
    );
    #[cfg(feature = "dim3")]
    PolyhedronFace::contacts(
        prediction_distance + capsule2.radius,
        &feature1,
        &best_sep.1,
        &feature2,
        &pos21,
        manifold,
    );

    // Adjust points to take the radius into account.
    manifold.local_n1 = best_sep.1;
    manifold.local_n2 = pos21 * -best_sep.1;
    manifold.kinematics.category = KinematicsCategory::PlanePoint;
    manifold.kinematics.radius1 = 0.0;
    manifold.kinematics.radius2 = 0.0;

    if swapped_reference {
        for point in &mut manifold.points {
            point.local_p1 += manifold.local_n1 * capsule2.radius;
            point.dist -= capsule2.radius;
        }
    } else {
        for point in &mut manifold.points {
            point.local_p2 += manifold.local_n2 * capsule2.radius;
            point.dist -= capsule2.radius;
        }
    }

    // Transfer impulses.
    super::match_contacts(manifold, &old_manifold_points, swapped ^ swapped_reference);
}
