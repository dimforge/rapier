use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{
    Contact, ContactManifold, KinematicsCategory, PolygonalFeatureMap, PolyhedronFace,
};
use crate::math::{Isometry, Vector};
use crate::na::UnitQuaternion;
use na::Unit;
use ncollide::query;
use ncollide::query::algorithms::{gjk::GJKResult, VoronoiSimplex};

pub struct PfmPfmContactManifoldGeneratorWorkspace {
    simplex: VoronoiSimplex<f32>,
    last_gjk_dir: Option<Unit<Vector<f32>>>,
    last_optimal_dir: Option<Unit<Vector<f32>>>,
    feature1: PolyhedronFace,
    feature2: PolyhedronFace,
}

impl Default for PfmPfmContactManifoldGeneratorWorkspace {
    fn default() -> Self {
        Self {
            simplex: VoronoiSimplex::new(),
            last_gjk_dir: None,
            last_optimal_dir: None,
            feature1: PolyhedronFace::new(),
            feature2: PolyhedronFace::new(),
        }
    }
}

pub fn generate_contacts_pfm_pfm(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Some(pfm1), Some(pfm2)) = (
        ctxt.collider1.shape().as_polygonal_feature_map(),
        ctxt.collider2.shape().as_polygonal_feature_map(),
    ) {
        do_generate_contacts(pfm1, pfm2, ctxt);
        ctxt.manifold.update_warmstart_multiplier();
        ctxt.manifold.sort_contacts(ctxt.prediction_distance);
    }
}

fn do_generate_contacts(
    pfm1: &dyn PolygonalFeatureMap,
    pfm2: &dyn PolygonalFeatureMap,
    ctxt: &mut PrimitiveContactGenerationContext,
) {
    let pos12 = ctxt.position1.inverse() * ctxt.position2;
    let pos21 = pos12.inverse();

    // if ctxt.manifold.try_update_contacts(&pos12, true) {
    //     return;
    // }

    let workspace: &mut PfmPfmContactManifoldGeneratorWorkspace = ctxt
        .workspace
        .as_mut()
        .expect("The PfmPfmContactManifoldGeneratorWorkspace is missing.")
        .downcast_mut()
        .expect("Invalid workspace type, expected a PfmPfmContactManifoldGeneratorWorkspace.");

    let contact = query::contact_support_map_support_map_with_params(
        &Isometry::identity(),
        pfm1,
        &pos12,
        pfm2,
        ctxt.prediction_distance,
        &mut workspace.simplex,
        workspace.last_gjk_dir,
    );

    let old_manifold_points = ctxt.manifold.points.clone();
    ctxt.manifold.points.clear();

    match contact {
        GJKResult::ClosestPoints(local_p1, local_p2, dir) => {
            workspace.last_gjk_dir = Some(dir);
            let normal1 = dir;
            let normal2 = pos21 * -dir;
            pfm1.local_support_feature(&normal1, &mut workspace.feature1);
            pfm2.local_support_feature(&normal2, &mut workspace.feature2);
            workspace.feature2.transform_by(&pos12);

            PolyhedronFace::contacts(
                ctxt.prediction_distance,
                &workspace.feature1,
                &normal1,
                &workspace.feature2,
                &pos21,
                ctxt.manifold,
            );

            // if ctxt.manifold.all_contacts().is_empty() {
            //     // Add at least the deepest contact.
            //     let dist = (local_p2 - local_p1).dot(&dir);
            //     ctxt.manifold.points.push(Contact {
            //         local_p1,
            //         local_p2: pos21 * local_p2,
            //         impulse: 0.0,
            //         tangent_impulse: Contact::zero_tangent_impulse(),
            //         fid1: 0, // FIXME
            //         fid2: 0, // FIXME
            //         dist,
            //     });
            // }

            // Adjust points to take the radius into account.
            ctxt.manifold.local_n1 = *normal1;
            ctxt.manifold.local_n2 = *normal2;
            ctxt.manifold.kinematics.category = KinematicsCategory::PlanePoint; // FIXME
            ctxt.manifold.kinematics.radius1 = 0.0;
            ctxt.manifold.kinematics.radius2 = 0.0;
        }
        GJKResult::NoIntersection(dir) => {
            workspace.last_gjk_dir = Some(dir);
        }
        _ => {}
    }
}

fn do_generate_contacts2(
    pfm1: &dyn PolygonalFeatureMap,
    pfm2: &dyn PolygonalFeatureMap,
    ctxt: &mut PrimitiveContactGenerationContext,
) {
    let pos12 = ctxt.position1.inverse() * ctxt.position2;
    let pos21 = pos12.inverse();

    // if ctxt.manifold.try_update_contacts(&pos12, true) {
    //     return;
    // }

    let workspace: &mut PfmPfmContactManifoldGeneratorWorkspace = ctxt
        .workspace
        .as_mut()
        .expect("The PfmPfmContactManifoldGeneratorWorkspace is missing.")
        .downcast_mut()
        .expect("Invalid workspace type, expected a PfmPfmContactManifoldGeneratorWorkspace.");

    fn generate_single_contact_pair(
        pfm1: &dyn PolygonalFeatureMap,
        pfm2: &dyn PolygonalFeatureMap,
        pos12: &Isometry<f32>,
        pos21: &Isometry<f32>,
        prediction_distance: f32,
        manifold: &mut ContactManifold,
        workspace: &mut PfmPfmContactManifoldGeneratorWorkspace,
    ) -> Option<Unit<Vector<f32>>> {
        let contact = query::contact_support_map_support_map_with_params(
            &Isometry::identity(),
            pfm1,
            &pos12,
            pfm2,
            prediction_distance,
            &mut workspace.simplex,
            workspace.last_gjk_dir,
        );

        match contact {
            GJKResult::ClosestPoints(local_p1, local_p2, dir) => {
                // Add at least the deepest contact.
                let dist = (local_p2 - local_p1).dot(&dir);
                manifold.points.push(Contact {
                    local_p1,
                    local_p2: pos21 * local_p2,
                    impulse: 0.0,
                    tangent_impulse: Contact::zero_tangent_impulse(),
                    fid1: 0, // FIXME
                    fid2: 0, // FIXME
                    dist,
                });

                Some(dir)
            }
            GJKResult::NoIntersection(dir) => Some(dir),
            _ => None,
        }
    }

    let old_manifold_points = ctxt.manifold.points.clone();
    ctxt.manifold.points.clear();

    if let Some(local_n1) = generate_single_contact_pair(
        pfm1,
        pfm2,
        &pos12,
        &pos21,
        ctxt.prediction_distance,
        ctxt.manifold,
        workspace,
    ) {
        workspace.last_gjk_dir = Some(local_n1);

        if !ctxt.manifold.points.is_empty() {
            use crate::utils::WBasis;
            // Use perturbations to generate other contact points.
            let basis = local_n1.orthonormal_basis();
            let perturbation_angle = std::f32::consts::PI / 180.0 * 15.0; // FIXME: this should be a function of the shape size.
            let perturbations = [
                UnitQuaternion::new(basis[0] * perturbation_angle),
                UnitQuaternion::new(basis[0] * -perturbation_angle),
                UnitQuaternion::new(basis[1] * perturbation_angle),
                UnitQuaternion::new(basis[1] * -perturbation_angle),
            ];

            for rot in &perturbations {
                let new_pos12 = pos12 * rot;
                let new_pos21 = new_pos12.inverse();
                generate_single_contact_pair(
                    pfm1,
                    pfm2,
                    &new_pos12,
                    &new_pos21,
                    ctxt.prediction_distance,
                    ctxt.manifold,
                    workspace,
                );
                println!("After perturbation: {}", ctxt.manifold.points.len());
            }

            // Set manifold normal.
            ctxt.manifold.local_n1 = *local_n1;
            ctxt.manifold.local_n2 = pos21 * -*local_n1;
            ctxt.manifold.kinematics.category = KinematicsCategory::PlanePoint; // FIXME
            ctxt.manifold.kinematics.radius1 = 0.0;
            ctxt.manifold.kinematics.radius2 = 0.0;

            ctxt.manifold.try_update_contacts(&pos12, false);
        }
    }
}
