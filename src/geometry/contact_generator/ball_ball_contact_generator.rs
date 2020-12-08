use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{Contact, ContactManifoldData, KinematicsCategory};
use crate::math::{Point, Vector};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::geometry::contact_generator::PrimitiveContactGenerationContextSimd,
    crate::geometry::{WBall, WContact},
    crate::math::{Isometry, SimdFloat, SIMD_WIDTH},
    simba::simd::SimdValue,
};

#[cfg(feature = "simd-is-enabled")]
fn generate_contacts_simd(ball1: &WBall, ball2: &WBall, pos21: &Isometry<SimdFloat>) -> WContact {
    let dcenter = ball2.center - ball1.center;
    let center_dist = dcenter.magnitude();
    let normal = dcenter / center_dist;

    WContact {
        local_p1: ball1.center + normal * ball1.radius,
        local_p2: pos21 * (ball2.center - normal * ball2.radius),
        local_n1: normal,
        local_n2: pos21 * -normal,
        fid1: [0; SIMD_WIDTH],
        fid2: [0; SIMD_WIDTH],
        dist: center_dist - ball1.radius - ball2.radius,
    }
}

#[cfg(feature = "simd-is-enabled")]
pub fn generate_contacts_ball_ball_simd(ctxt: &mut PrimitiveContactGenerationContextSimd) {
    let pos_ba = ctxt.positions2.inverse() * ctxt.positions1;
    let radii_a =
        SimdFloat::from(array![|ii| ctxt.shapes1[ii].as_ball().unwrap().radius; SIMD_WIDTH]);
    let radii_b =
        SimdFloat::from(array![|ii| ctxt.shapes2[ii].as_ball().unwrap().radius; SIMD_WIDTH]);

    let wball_a = WBall::new(Point::origin(), radii_a);
    let wball_b = WBall::new(pos_ba.inverse_transform_point(&Point::origin()), radii_b);
    let contacts = generate_contacts_simd(&wball_a, &wball_b, &pos_ba);

    for (i, manifold) in ctxt.manifolds.iter_mut().enumerate() {
        // FIXME: compare the dist before extracting the contact.
        let (contact, local_n1, local_n2) = contacts.extract(i);
        if contact.dist <= ctxt.prediction_distance {
            if manifold.points.len() != 0 {
                manifold.points[0].copy_geometry_from(contact);
            } else {
                manifold.points.push(contact);
            }

            manifold.local_n1 = local_n1;
            manifold.local_n2 = local_n2;
            manifold.kinematics.category = KinematicsCategory::PointPoint;
            manifold.kinematics.radius1 = radii_a.extract(i);
            manifold.kinematics.radius2 = radii_b.extract(i);
            ContactManifoldData::update_warmstart_multiplier(manifold);
        } else {
            manifold.points.clear();
        }

        manifold.sort_contacts(ctxt.prediction_distance);
    }
}

pub fn generate_contacts_ball_ball(ctxt: &mut PrimitiveContactGenerationContext) {
    let pos_ba = ctxt.position2.inverse() * ctxt.position1;
    let radius_a = ctxt.shape1.as_ball().unwrap().radius;
    let radius_b = ctxt.shape2.as_ball().unwrap().radius;

    let dcenter = pos_ba.inverse_transform_point(&Point::origin()).coords;
    let center_dist = dcenter.magnitude();
    let dist = center_dist - radius_a - radius_b;

    if dist < ctxt.prediction_distance {
        let local_n1 = if center_dist != 0.0 {
            dcenter / center_dist
        } else {
            Vector::y()
        };

        let local_n2 = pos_ba.inverse_transform_vector(&-local_n1);
        let local_p1 = local_n1 * radius_a;
        let local_p2 = local_n2 * radius_b;
        let contact = Contact::new(local_p1.into(), local_p2.into(), 0, 0, dist);

        if ctxt.manifold.points.len() != 0 {
            ctxt.manifold.points[0].copy_geometry_from(contact);
        } else {
            ctxt.manifold.points.push(contact);
        }

        ctxt.manifold.local_n1 = local_n1;
        ctxt.manifold.local_n2 = local_n2;
        ctxt.manifold.kinematics.category = KinematicsCategory::PointPoint;
        ctxt.manifold.kinematics.radius1 = radius_a;
        ctxt.manifold.kinematics.radius2 = radius_b;
        ContactManifoldData::update_warmstart_multiplier(ctxt.manifold);
    } else {
        ctxt.manifold.points.clear();
    }

    ctxt.manifold.sort_contacts(ctxt.prediction_distance);
}
