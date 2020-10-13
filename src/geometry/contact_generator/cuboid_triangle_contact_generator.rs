use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
#[cfg(feature = "dim3")]
use crate::geometry::PolyhedronFace;
use crate::geometry::{cuboid, sat, ContactManifold, Cuboid, KinematicsCategory, Shape, Triangle};
use crate::math::Isometry;
#[cfg(feature = "dim2")]
use crate::{
    geometry::{triangle, CuboidFeature},
    math::Vector,
};

pub fn generate_contacts_cuboid_triangle(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Shape::Cuboid(cube1), Shape::Triangle(triangle2)) = (ctxt.shape1, ctxt.shape2) {
        generate_contacts(
            ctxt.prediction_distance,
            cube1,
            ctxt.position1,
            triangle2,
            ctxt.position2,
            ctxt.manifold,
            false,
        );
        ctxt.manifold.update_warmstart_multiplier();
    } else if let (Shape::Triangle(triangle1), Shape::Cuboid(cube2)) = (ctxt.shape1, ctxt.shape2) {
        generate_contacts(
            ctxt.prediction_distance,
            cube2,
            ctxt.position2,
            triangle1,
            ctxt.position1,
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
    mut pos1: &'a Isometry<f32>,
    triangle2: &'a Triangle,
    mut pos2: &'a Isometry<f32>,
    manifold: &mut ContactManifold,
    swapped: bool,
) {
    let mut pos12 = pos1.inverse() * pos2;
    let mut pos21 = pos12.inverse();

    if (!swapped && manifold.try_update_contacts(&pos12, true))
        || (swapped && manifold.try_update_contacts(&pos21, true))
    {
        return;
    }

    /*
     *
     * Point-Face cases.
     *
     */
    let sep1 = sat::cube_support_map_find_local_separating_normal_oneway(cube1, triangle2, &pos12);
    if sep1.0 > prediction_distance {
        manifold.points.clear();
        return;
    }

    let sep2 = sat::triangle_cuboid_find_local_separating_normal_oneway(triangle2, cube1, &pos21);
    if sep2.0 > prediction_distance {
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
        sat::cube_triangle_find_local_separating_edge_twoway(cube1, triangle2, &pos12, &pos21);
    if sep3.0 > prediction_distance {
        manifold.points.clear();
        return;
    }

    /*
     *
     * Select the best combination of features
     * and get the polygons to clip.
     *
     */
    let mut swapped_reference = false;
    let mut best_sep = sep1;

    if sep2.0 > sep1.0 && sep2.0 > sep3.0 {
        // The reference shape will be the second shape.
        // std::mem::swap(&mut cube1, &mut triangle2);
        std::mem::swap(&mut pos1, &mut pos2);
        std::mem::swap(&mut pos12, &mut pos21);
        best_sep = sep2;
        swapped_reference = true;
    } else if sep3.0 > sep1.0 {
        best_sep = sep3;
    }

    let feature1;
    let mut feature2;

    #[cfg(feature = "dim2")]
    {
        if swapped_reference {
            feature1 = triangle::support_face(triangle2, best_sep.1);
            feature2 = cuboid::support_face(cube1, pos21 * -best_sep.1);
        } else {
            feature1 = cuboid::support_face(cube1, best_sep.1);
            feature2 = triangle::support_face(triangle2, pos21 * -best_sep.1);
        }
    }
    #[cfg(feature = "dim3")]
    {
        if swapped_reference {
            feature1 = PolyhedronFace::from(*triangle2);
            feature2 = cuboid::polyhedron_support_face(cube1, pos21 * -best_sep.1);
        } else {
            feature1 = cuboid::polyhedron_support_face(cube1, best_sep.1);
            feature2 = PolyhedronFace::from(*triangle2);
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
        prediction_distance,
        &feature1,
        &best_sep.1,
        &feature2,
        &pos21,
        manifold,
    );
    #[cfg(feature = "dim3")]
    PolyhedronFace::contacts(
        prediction_distance,
        &feature1,
        &best_sep.1,
        &feature2,
        &pos21,
        manifold,
    );

    manifold.local_n1 = best_sep.1;
    manifold.local_n2 = pos21 * -best_sep.1;
    manifold.kinematics.category = KinematicsCategory::PlanePoint;
    manifold.kinematics.radius1 = 0.0;
    manifold.kinematics.radius2 = 0.0;

    // Transfer impulses.
    super::match_contacts(manifold, &old_manifold_points, swapped ^ swapped_reference);
}
