use crate::data::hashmap::{Entry, HashMap};
use crate::data::MaybeSerializableData;
use crate::geometry::contact_generator::{
    ContactGenerationContext, ContactGeneratorWorkspace, PrimitiveContactGenerationContext,
    PrimitiveContactGenerator,
};
#[cfg(feature = "dim2")]
use crate::geometry::Capsule;
use crate::geometry::{Collider, ContactManifold, HeightField, Shape};
use crate::ncollide::bounding_volume::BoundingVolume;
#[cfg(feature = "serde-serialize")]
use erased_serde::Serialize;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
struct SubDetector {
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    generator: Option<PrimitiveContactGenerator>,
    manifold_id: usize,
    timestamp: bool,
    workspace: Option<ContactGeneratorWorkspace>,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct HeightFieldShapeContactGeneratorWorkspace {
    timestamp: bool,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    old_manifolds: Vec<ContactManifold>,
    sub_detectors: HashMap<usize, SubDetector>,
}

impl HeightFieldShapeContactGeneratorWorkspace {
    pub fn new() -> Self {
        Self {
            timestamp: false,
            old_manifolds: Vec::new(),
            sub_detectors: HashMap::default(),
        }
    }
}

pub fn generate_contacts_heightfield_shape(ctxt: &mut ContactGenerationContext) {
    let collider1 = &ctxt.colliders[ctxt.pair.pair.collider1];
    let collider2 = &ctxt.colliders[ctxt.pair.pair.collider2];

    if let Some(heightfield1) = collider1.shape().as_heightfield() {
        do_generate_contacts(heightfield1, collider1, collider2, ctxt, false)
    } else if let Some(heightfield2) = collider2.shape().as_heightfield() {
        do_generate_contacts(heightfield2, collider2, collider1, ctxt, true)
    }
}

fn do_generate_contacts(
    heightfield1: &HeightField,
    collider1: &Collider,
    collider2: &Collider,
    ctxt: &mut ContactGenerationContext,
    _flipped: bool,
) {
    let workspace: &mut HeightFieldShapeContactGeneratorWorkspace = ctxt
        .pair
        .generator_workspace
        .as_mut()
        .expect("The HeightFieldShapeContactGeneratorWorkspace is missing.")
        .0
        .downcast_mut()
        .expect("Invalid workspace type, expected a HeightFieldShapeContactGeneratorWorkspace.");
    let new_timestamp = !workspace.timestamp;
    workspace.timestamp = new_timestamp;

    /*
     * Compute interferences.
     */
    let pos12 = collider1.position.inverse() * collider2.position;
    // TODO: somehow precompute the AABB and reuse it?
    let ls_aabb2 = collider2
        .shape()
        .compute_aabb(&pos12)
        .loosened(ctxt.prediction_distance);

    std::mem::swap(&mut workspace.old_manifolds, &mut ctxt.pair.manifolds);
    ctxt.pair.manifolds.clear();
    let coll_pair = ctxt.pair.pair;
    let manifolds = &mut ctxt.pair.manifolds;
    let prediction_distance = ctxt.prediction_distance;
    let dispatcher = ctxt.dispatcher;
    let solver_flags = ctxt.solver_flags;
    let shape_type2 = collider2.shape().shape_type();

    heightfield1.map_elements_in_local_aabb(&ls_aabb2, &mut |i, part1, _| {
        let position1 = collider1.position();
        #[cfg(feature = "dim2")]
        let sub_shape1 = Capsule::new(part1.a, part1.b, 0.0); // TODO: use a segment instead.
        #[cfg(feature = "dim3")]
        let sub_shape1 = *part1;

        let sub_detector = match workspace.sub_detectors.entry(i) {
            Entry::Occupied(entry) => {
                let sub_detector = entry.into_mut();
                let manifold = workspace.old_manifolds[sub_detector.manifold_id].take();
                sub_detector.manifold_id = manifolds.len();
                sub_detector.timestamp = new_timestamp;
                manifolds.push(manifold);
                sub_detector
            }
            Entry::Vacant(entry) => {
                let (generator, workspace2) =
                    dispatcher.dispatch_primitives(sub_shape1.shape_type(), shape_type2);
                let sub_detector = SubDetector {
                    generator: Some(generator),
                    manifold_id: manifolds.len(),
                    timestamp: new_timestamp,
                    workspace: workspace2,
                };
                let manifold = ContactManifold::with_subshape_indices(
                    coll_pair,
                    collider1,
                    collider2,
                    i,
                    0,
                    solver_flags,
                );
                manifolds.push(manifold);

                entry.insert(sub_detector)
            }
        };

        if sub_detector.generator.is_none() {
            // We probably lost the generator after deserialization.
            // So we need to dispatch again.
            let (generator, workspace2) =
                dispatcher.dispatch_primitives(sub_shape1.shape_type(), shape_type2);
            sub_detector.generator = Some(generator);

            // Don't overwrite the workspace if we already deserialized one.
            if sub_detector.workspace.is_none() {
                sub_detector.workspace = workspace2;
            }
        }

        let manifold = &mut manifolds[sub_detector.manifold_id];

        let mut ctxt2 = if coll_pair.collider1 != manifold.pair.collider1 {
            manifold.position1 = collider2.position;
            manifold.position2 = collider1.position;
            PrimitiveContactGenerationContext {
                prediction_distance,
                collider1: collider2,
                collider2: collider1,
                shape1: collider2.shape(),
                shape2: &sub_shape1,
                position1: collider2.position(),
                position2: position1,
                manifold,
                workspace: sub_detector.workspace.as_mut().map(|w| &mut *w.0),
            }
        } else {
            manifold.position1 = collider1.position;
            manifold.position2 = collider2.position;
            PrimitiveContactGenerationContext {
                prediction_distance,
                collider1,
                collider2,
                shape1: &sub_shape1,
                shape2: collider2.shape(),
                position1,
                position2: collider2.position(),
                manifold,
                workspace: sub_detector.workspace.as_mut().map(|w| &mut *w.0),
            }
        };

        (sub_detector.generator.unwrap().generate_contacts)(&mut ctxt2)
    });

    workspace
        .sub_detectors
        .retain(|_, detector| detector.timestamp == new_timestamp)
}

impl MaybeSerializableData for HeightFieldShapeContactGeneratorWorkspace {
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<(u32, &dyn Serialize)> {
        Some((
            super::WorkspaceSerializationTag::HeightfieldShapeContactGeneratorWorkspace as u32,
            self,
        ))
    }

    fn clone_dyn(&self) -> Box<dyn MaybeSerializableData> {
        Box::new(self.clone())
    }
}
