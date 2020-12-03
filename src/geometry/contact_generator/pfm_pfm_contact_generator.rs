use crate::data::MaybeSerializableData;
use crate::geometry::contact_generator::PrimitiveContactGenerationContext;
use crate::geometry::{KinematicsCategory, PolygonalFeatureMap, PolyhedronFace};
use crate::math::{Isometry, Vector};
#[cfg(feature = "serde-serialize")]
use erased_serde::Serialize;
use na::Unit;
use ncollide::query;
use ncollide::query::algorithms::{gjk::GJKResult, VoronoiSimplex};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct PfmPfmContactManifoldGeneratorWorkspace {
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "VoronoiSimplex::new")
    )]
    simplex: VoronoiSimplex<f32>,
    last_gjk_dir: Option<Unit<Vector<f32>>>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    feature1: PolyhedronFace,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    feature2: PolyhedronFace,
}

impl Default for PfmPfmContactManifoldGeneratorWorkspace {
    fn default() -> Self {
        Self {
            simplex: VoronoiSimplex::new(),
            last_gjk_dir: None,
            feature1: PolyhedronFace::new(),
            feature2: PolyhedronFace::new(),
        }
    }
}

pub fn generate_contacts_pfm_pfm(ctxt: &mut PrimitiveContactGenerationContext) {
    if let (Some((pfm1, border_radius1)), Some((pfm2, border_radius2))) = (
        ctxt.shape1.as_polygonal_feature_map(),
        ctxt.shape2.as_polygonal_feature_map(),
    ) {
        do_generate_contacts(pfm1, border_radius1, pfm2, border_radius2, ctxt);
        ctxt.manifold.update_warmstart_multiplier();
        ctxt.manifold.sort_contacts(ctxt.prediction_distance);
    }
}

fn do_generate_contacts(
    pfm1: &dyn PolygonalFeatureMap,
    border_radius1: f32,
    pfm2: &dyn PolygonalFeatureMap,
    border_radius2: f32,
    ctxt: &mut PrimitiveContactGenerationContext,
) {
    let pos12 = ctxt.manifold.position1.inverse() * ctxt.manifold.position2;
    let pos21 = pos12.inverse();

    // We use very small thresholds for the manifold update because something to high would
    // cause numerical drifts with the effect of introducing bumps in
    // what should have been smooth rolling motions.
    if ctxt
        .manifold
        .try_update_contacts_eps(&pos12, crate::utils::COS_1_DEGREES, 1.0e-6)
    {
        return;
    }

    let workspace: &mut PfmPfmContactManifoldGeneratorWorkspace = ctxt
        .workspace
        .as_mut()
        .expect("The PfmPfmContactManifoldGeneratorWorkspace is missing.")
        .downcast_mut()
        .expect("Invalid workspace type, expected a PfmPfmContactManifoldGeneratorWorkspace.");

    let total_prediction = ctxt.prediction_distance + border_radius1 + border_radius2;
    let contact = query::contact_support_map_support_map_with_params(
        &Isometry::identity(),
        pfm1,
        &pos12,
        pfm2,
        total_prediction,
        &mut workspace.simplex,
        workspace.last_gjk_dir,
    );

    let old_manifold_points = ctxt.manifold.points.clone();
    ctxt.manifold.points.clear();

    match contact {
        GJKResult::ClosestPoints(_, _, dir) => {
            workspace.last_gjk_dir = Some(dir);
            let normal1 = dir;
            let normal2 = pos21 * -dir;
            pfm1.local_support_feature(&normal1, &mut workspace.feature1);
            pfm2.local_support_feature(&normal2, &mut workspace.feature2);
            workspace.feature2.transform_by(&pos12);

            PolyhedronFace::contacts(
                total_prediction,
                &workspace.feature1,
                &normal1,
                &workspace.feature2,
                &pos21,
                ctxt.manifold,
            );

            if border_radius1 != 0.0 || border_radius2 != 0.0 {
                for contact in &mut ctxt.manifold.points {
                    contact.local_p1 += *normal1 * border_radius1;
                    contact.local_p2 += *normal2 * border_radius2;
                    contact.dist -= border_radius1 + border_radius2;
                }
            }

            // Adjust points to take the radius into account.
            ctxt.manifold.local_n1 = *normal1;
            ctxt.manifold.local_n2 = *normal2;
            ctxt.manifold.kinematics.category = KinematicsCategory::PlanePoint; // TODO: is this the more appropriate?
            ctxt.manifold.kinematics.radius1 = 0.0;
            ctxt.manifold.kinematics.radius2 = 0.0;
        }
        GJKResult::NoIntersection(dir) => {
            workspace.last_gjk_dir = Some(dir);
        }
        _ => {}
    }

    // Transfer impulses.
    super::match_contacts(&mut ctxt.manifold, &old_manifold_points, false);
}

impl MaybeSerializableData for PfmPfmContactManifoldGeneratorWorkspace {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<(u32, &dyn Serialize)> {
        Some((
            super::WorkspaceSerializationTag::PfmPfmContactGeneratorWorkspace as u32,
            self,
        ))
    }

    fn clone_dyn(&self) -> Box<dyn MaybeSerializableData> {
        Box::new(self.clone())
    }
}
