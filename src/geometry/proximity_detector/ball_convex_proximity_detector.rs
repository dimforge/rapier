use crate::geometry::proximity_detector::PrimitiveProximityDetectionContext;
use crate::geometry::{Ball, Proximity};
use crate::math::Isometry;
use buckler::query::PointQuery;

pub fn detect_proximity_ball_convex(ctxt: &mut PrimitiveProximityDetectionContext) -> Proximity {
    if let Some(ball1) = ctxt.shape1.as_ball() {
        do_detect_proximity(ctxt.shape2, ball1, &ctxt)
    } else if let Some(ball2) = ctxt.shape2.as_ball() {
        do_detect_proximity(ctxt.shape1, ball2, &ctxt)
    } else {
        panic!("Invalid shape types provide.")
    }
}

fn do_detect_proximity<P: ?Sized + PointQuery>(
    point_query1: &P,
    ball2: &Ball,
    ctxt: &PrimitiveProximityDetectionContext,
) -> Proximity {
    let local_p2_1 = ctxt
        .position1
        .inverse_transform_point(&ctxt.position2.translation.vector.into());

    let proj = point_query1.project_local_point(&local_p2_1, cfg!(feature = "dim3"));
    let dpos = local_p2_1 - proj.local_point;
    let dist = dpos.norm();

    if proj.is_inside {
        return Proximity::Intersecting;
    }

    if dist <= ball2.radius + ctxt.prediction_distance {
        if dist <= ball2.radius {
            Proximity::Intersecting
        } else {
            Proximity::WithinMargin
        }
    } else {
        Proximity::Disjoint
    }
}
