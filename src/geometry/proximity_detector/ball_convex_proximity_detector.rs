use crate::geometry::proximity_detector::PrimitiveProximityDetectionContext;
use crate::geometry::{Ball, Proximity, Shape};
use crate::math::Isometry;
use ncollide::query::PointQuery;

pub fn detect_proximity_ball_convex(ctxt: &mut PrimitiveProximityDetectionContext) -> Proximity {
    if let Shape::Ball(ball1) = ctxt.shape1 {
        match ctxt.shape2 {
            Shape::Triangle(tri2) => do_detect_proximity(tri2, ball1, &ctxt),
            Shape::Cuboid(cube2) => do_detect_proximity(cube2, ball1, &ctxt),
            _ => unimplemented!(),
        }
    } else if let Shape::Ball(ball2) = ctxt.shape2 {
        match ctxt.shape1 {
            Shape::Triangle(tri1) => do_detect_proximity(tri1, ball2, &ctxt),
            Shape::Cuboid(cube1) => do_detect_proximity(cube1, ball2, &ctxt),
            _ => unimplemented!(),
        }
    } else {
        panic!("Invalid shape types provide.")
    }
}

fn do_detect_proximity<P: PointQuery<f32>>(
    point_query1: &P,
    ball2: &Ball,
    ctxt: &PrimitiveProximityDetectionContext,
) -> Proximity {
    let local_p2_1 = ctxt
        .position1
        .inverse_transform_point(&ctxt.position2.translation.vector.into());

    // TODO: add a `project_local_point` to the PointQuery trait to avoid
    // the identity isometry.
    let proj =
        point_query1.project_point(&Isometry::identity(), &local_p2_1, cfg!(feature = "dim3"));
    let dpos = local_p2_1 - proj.point;
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
