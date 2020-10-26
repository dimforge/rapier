#![allow(dead_code)]

use crate::geometry::proximity_detector::PrimitiveProximityDetectionContext;
use crate::geometry::{sat, Polygon, Proximity};
use crate::math::Isometry;

pub fn detect_proximity_polygon_polygon(
    _ctxt: &mut PrimitiveProximityDetectionContext,
) -> Proximity {
    unimplemented!()
    // if let (Some(polygon1), Some(polygon2)) = (ctxt.shape1.as_polygon(), ctxt.shape2.as_polygon()) {
    //     detect_proximity(
    //         ctxt.prediction_distance,
    //         polygon1,
    //         &ctxt.position1,
    //         polygon2,
    //         &ctxt.position2,
    //     )
    // } else {
    //     unreachable!()
    // }
}

fn detect_proximity<'a>(
    prediction_distance: f32,
    p1: &'a Polygon,
    m1: &'a Isometry<f32>,
    p2: &'a Polygon,
    m2: &'a Isometry<f32>,
) -> Proximity {
    let m12 = m1.inverse() * m2;
    let m21 = m12.inverse();

    let sep1 = sat::polygon_polygon_compute_separation_features(p1, p2, &m12);
    if sep1.0 > prediction_distance {
        return Proximity::Disjoint;
    }

    let sep2 = sat::polygon_polygon_compute_separation_features(p2, p1, &m21);
    if sep2.0 > prediction_distance {
        return Proximity::Disjoint;
    }

    if sep2.0 > sep1.0 {
        if sep2.0 > 0.0 {
            Proximity::WithinMargin
        } else {
            Proximity::Intersecting
        }
    } else {
        if sep1.0 > 0.0 {
            Proximity::WithinMargin
        } else {
            Proximity::Intersecting
        }
    }
}
