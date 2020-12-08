use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{Ball, Contact, ContactManifoldData, KinematicsCategory};
use crate::math::Isometry;
use buckler::query::PointQuery;
use na::Unit;

pub fn generate_contacts_ball_convex(ctxt: &mut PrimitiveContactGenerationContext) {
    if let Some(ball1) = ctxt.shape1.as_ball() {
        ctxt.manifold.swap_identifiers();
        do_generate_contacts(ctxt.shape2, ball1, ctxt, true);
    } else if let Some(ball2) = ctxt.shape2.as_ball() {
        do_generate_contacts(ctxt.shape1, ball2, ctxt, false);
    }

    ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}

fn do_generate_contacts<P: ?Sized + PointQuery>(
    point_query1: &P,
    ball2: &Ball,
    ctxt: &mut PrimitiveContactGenerationContext,
    swapped: bool,
) {
    let position1;
    let position2;

    if swapped {
        position1 = ctxt.position2;
        position2 = ctxt.position1;
    } else {
        position1 = ctxt.position1;
        position2 = ctxt.position2;
    }

    let local_p2_1 = position1.inverse_transform_point(&position2.translation.vector.into());
    let proj = point_query1.project_local_point(&local_p2_1, cfg!(feature = "dim3"));
    let dpos = local_p2_1 - proj.local_point;

    #[allow(unused_mut)] // Because `mut local_n1, mut dist` is needed in 2D but not in 3D.
    if let Some((mut local_n1, mut dist)) = Unit::try_new_and_get(dpos, 0.0) {
        #[cfg(feature = "dim2")]
        if proj.is_inside {
            local_n1 = -local_n1;
            dist = -dist;
        }

        if dist <= ball2.radius + ctxt.prediction_distance {
            let local_n2 = position2.inverse_transform_vector(&(position1 * -*local_n1));
            let local_p2 = (local_n2 * ball2.radius).into();
            let contact_point = Contact::new(proj.local_point, local_p2, 0, 0, dist - ball2.radius);

            if ctxt.manifold.points.len() != 1 {
                ctxt.manifold.points.clear();
                ctxt.manifold.points.push(contact_point);
            } else {
                // Copy only the geometry so we keep the warmstart impulses.
                ctxt.manifold.points[0].copy_geometry_from(contact_point);
            }

            ctxt.manifold.local_n1 = *local_n1;
            ctxt.manifold.local_n2 = local_n2;
            ctxt.manifold.kinematics.category = KinematicsCategory::PlanePoint;
            ctxt.manifold.kinematics.radius1 = 0.0;
            ctxt.manifold.kinematics.radius2 = ball2.radius;
            ContactManifoldData::update_warmstart_multiplier(ctxt.manifold);
        } else {
            ctxt.manifold.points.clear();
        }
    }
}
