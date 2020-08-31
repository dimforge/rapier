use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{cuboid, sat, ContactManifold, CuboidFeature, KinematicsCategory, Shape};
use crate::math::Isometry;
#[cfg(feature = "dim2")]
use crate::math::Vector;
use ncollide::shape::Cuboid;

pub fn generate_contacts_cuboid_cuboid(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Shape::Cuboid(cube1), Shape::Cuboid(cube2)) = (ctxt.shape1, ctxt.shape2) {
        generate_contacts(
            ctxt.prediction_distance,
            cube1,
            ctxt.collider1.position_wrt_parent(),
            ctxt.position1,
            cube2,
            ctxt.collider2.position_wrt_parent(),
            ctxt.position2,
            ctxt.manifold,
        );
    } else {
        unreachable!()
    }

    ctxt.manifold.update_warmstart_multiplier();
    ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}

pub fn generate_contacts<'a>(
    prediction_distance: f32,
    mut cube1: &'a Cuboid<f32>,
    mut origin1: &'a Isometry<f32>,
    mut pos1: &'a Isometry<f32>,
    mut cube2: &'a Cuboid<f32>,
    mut origin2: &'a Isometry<f32>,
    mut pos2: &'a Isometry<f32>,
    manifold: &mut ContactManifold,
) {
    let mut pos12 = pos1.inverse() * pos2;
    let mut pos21 = pos12.inverse();
    let mut orig_pos12 = origin1 * pos12 * origin2.inverse();
    let mut orig_pos21 = orig_pos12.inverse();

    if manifold.try_update_contacts(&orig_pos12) {
        return;
    }

    /*
     *
     * Point-Face
     *
     */
    let sep1 = sat::cuboid_cuboid_find_local_separating_normal_oneway(cube1, cube2, &pos12, &pos21);
    if sep1.0 > prediction_distance {
        manifold.points.clear();
        return;
    }

    let sep2 = sat::cuboid_cuboid_find_local_separating_normal_oneway(cube2, cube1, &pos21, &pos12);
    if sep2.0 > prediction_distance {
        manifold.points.clear();
        return;
    }

    /*
     *
     * Edge-Edge cases
     *
     */
    #[cfg(feature = "dim2")]
    let sep3 = (-f32::MAX, Vector::x()); // This case does not exist in 2D.
    #[cfg(feature = "dim3")]
    let sep3 = sat::cuboid_cuboid_find_local_separating_edge_twoway(cube1, cube2, &pos12, &pos21);
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
    let mut swapped = false;
    let mut best_sep = sep1;

    if sep2.0 > sep1.0 && sep2.0 > sep3.0 {
        // The reference shape will be the second shape.
        std::mem::swap(&mut cube1, &mut cube2);
        std::mem::swap(&mut pos12, &mut pos21);
        std::mem::swap(&mut orig_pos12, &mut orig_pos21);
        std::mem::swap(&mut origin1, &mut origin2);
        manifold.swap_identifiers();
        best_sep = sep2;
        swapped = true;
    } else if sep3.0 > sep1.0 {
        best_sep = sep3;
    }

    // We do this clone to perform contact tracking and transfer impulses.
    // FIXME: find a more efficient way of doing this.
    let old_manifold_points = manifold.points.clone();
    manifold.points.clear();

    // Now the reference feature is from `cube1` and the best separation is `best_sep`.
    // Everything must be expressed in the local-space of `cube1` for contact clipping.
    let mut feature1 = cuboid::support_feature(cube1, best_sep.1);
    feature1.transform_by(origin1);
    let mut feature2 = cuboid::support_feature(cube2, pos21 * -best_sep.1);
    feature2.transform_by(&pos12);
    feature2.transform_by(origin1);
    let n1 = origin1 * best_sep.1;

    match (&feature1, &feature2) {
        (CuboidFeature::Face(f1), CuboidFeature::Vertex(v2)) => {
            CuboidFeature::face_vertex_contacts(f1, &n1, v2, &orig_pos21, manifold)
        }
        #[cfg(feature = "dim3")]
        (CuboidFeature::Face(f1), CuboidFeature::Edge(e2)) => CuboidFeature::face_edge_contacts(
            prediction_distance,
            f1,
            &n1,
            e2,
            &orig_pos21,
            manifold,
            false,
        ),
        (CuboidFeature::Face(f1), CuboidFeature::Face(f2)) => CuboidFeature::face_face_contacts(
            prediction_distance,
            f1,
            &n1,
            f2,
            &orig_pos21,
            manifold,
        ),
        #[cfg(feature = "dim3")]
        (CuboidFeature::Edge(e1), CuboidFeature::Edge(e2)) => {
            CuboidFeature::edge_edge_contacts(e1, &n1, e2, &orig_pos21, manifold)
        }
        #[cfg(feature = "dim3")]
        (CuboidFeature::Edge(e1), CuboidFeature::Face(f2)) => {
            // Since f2 is also expressed in the local-space of the first
            // feature, the position we provide here is orig_pos21.
            CuboidFeature::face_edge_contacts(
                prediction_distance,
                f2,
                &-n1,
                e1,
                &orig_pos21,
                manifold,
                true,
            )
        }
        _ => unreachable!(), // The other cases are not possible.
    }

    manifold.local_n1 = n1;
    manifold.local_n2 = orig_pos21 * -n1;
    manifold.kinematics.category = KinematicsCategory::PlanePoint;
    manifold.kinematics.radius1 = 0.0;
    manifold.kinematics.radius2 = 0.0;

    // Transfer impulses.
    super::match_contacts(manifold, &old_manifold_points, swapped);
}
