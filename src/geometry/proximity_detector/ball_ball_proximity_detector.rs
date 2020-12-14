use crate::geometry::proximity_detector::PrimitiveProximityDetectionContext;

use crate::geometry::Proximity;
use crate::math::Point;
#[cfg(feature = "simd-is-enabled")]
use {
    crate::geometry::{proximity_detector::PrimitiveProximityDetectionContextSimd, WBall},
    crate::math::{SimdReal, SIMD_WIDTH},
    simba::simd::SimdValue,
};

#[cfg(feature = "simd-is-enabled")]
fn ball_distance_simd(ball1: &WBall, ball2: &WBall) -> SimdReal {
    let dcenter = ball2.center - ball1.center;
    let center_dist = dcenter.magnitude();
    center_dist - ball1.radius - ball2.radius
}

#[cfg(feature = "simd-is-enabled")]
pub fn detect_proximity_ball_ball_simd(
    ctxt: &mut PrimitiveProximityDetectionContextSimd,
) -> [Proximity; SIMD_WIDTH] {
    let pos_ba = ctxt.positions2.inverse() * ctxt.positions1;
    let radii_a =
        SimdReal::from(array![|ii| ctxt.shapes1[ii].as_ball().unwrap().radius; SIMD_WIDTH]);
    let radii_b =
        SimdReal::from(array![|ii| ctxt.shapes2[ii].as_ball().unwrap().radius; SIMD_WIDTH]);

    let wball_a = WBall::new(Point::origin(), radii_a);
    let wball_b = WBall::new(pos_ba.inverse_transform_point(&Point::origin()), radii_b);
    let distances = ball_distance_simd(&wball_a, &wball_b);
    let mut proximities = [Proximity::Disjoint; SIMD_WIDTH];

    for i in 0..SIMD_WIDTH {
        // FIXME: compare the dist before computing the proximity.
        let dist = distances.extract(i);
        if dist > ctxt.prediction_distance {
            proximities[i] = Proximity::Disjoint;
        } else if dist > 0.0 {
            proximities[i] = Proximity::WithinMargin;
        } else {
            proximities[i] = Proximity::Intersecting
        }
    }

    proximities
}

pub fn detect_proximity_ball_ball(ctxt: &mut PrimitiveProximityDetectionContext) -> Proximity {
    let pos_ba = ctxt.position2.inverse() * ctxt.position1;
    let radius_a = ctxt.shape1.as_ball().unwrap().radius;
    let radius_b = ctxt.shape2.as_ball().unwrap().radius;

    let center_a = Point::origin();
    let center_b = pos_ba.inverse_transform_point(&Point::origin());

    let dcenter = center_b - center_a;
    let center_dist = dcenter.magnitude();
    let dist = center_dist - radius_a - radius_b;

    if dist > ctxt.prediction_distance {
        Proximity::Disjoint
    } else if dist > 0.0 {
        Proximity::WithinMargin
    } else {
        Proximity::Intersecting
    }
}
