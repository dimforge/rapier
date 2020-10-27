use crate::geometry::contact_generator::{
    ContactGenerationContext, PrimitiveContactGenerationContext, PrimitiveContactGenerator,
};
#[cfg(feature = "dim2")]
use crate::geometry::Capsule;
use crate::geometry::{Collider, ContactManifold, HeightField, Shape, ShapeType};
use crate::ncollide::bounding_volume::BoundingVolume;
use std::any::Any;
use std::collections::hash_map::Entry;
use std::collections::HashMap;

struct SubDetector {
    generator: PrimitiveContactGenerator,
    manifold_id: usize,
    timestamp: bool,
    workspace: Option<Box<(dyn Any + Send + Sync)>>,
}

pub struct HeightFieldShapeContactGeneratorWorkspace {
    timestamp: bool,
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
        .downcast_mut()
        .expect("Invalid workspace type, expected a HeightFieldShapeContactGeneratorWorkspace.");
    let shape_type2 = collider2.shape().shape_type();

    /*
     * Detect if the detector context has been reset.
     */
    if !ctxt.pair.manifolds.is_empty() && workspace.sub_detectors.is_empty() {
        // Rebuild the subdetector hashmap.
        for (manifold_id, manifold) in ctxt.pair.manifolds.iter().enumerate() {
            let subshape_id = if manifold.pair.collider1 == ctxt.pair.pair.collider1 {
                manifold.subshape_index_pair.0
            } else {
                manifold.subshape_index_pair.1
            };
            let (generator, workspace2) = ctxt
                .dispatcher
                .dispatch_primitives(ShapeType::Capsule, shape_type2);

            let sub_detector = SubDetector {
                generator,
                manifold_id,
                timestamp: workspace.timestamp,
                workspace: workspace2,
            };

            workspace.sub_detectors.insert(subshape_id, sub_detector);
        }
    }

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
                    generator,
                    manifold_id: manifolds.len(),
                    timestamp: new_timestamp,
                    workspace: workspace2,
                };
                let manifold =
                    ContactManifold::with_subshape_indices(coll_pair, collider1, collider2, i, 0);
                manifolds.push(manifold);

                entry.insert(sub_detector)
            }
        };

        let manifold = &mut manifolds[sub_detector.manifold_id];

        let mut ctxt2 = if coll_pair.collider1 != manifold.pair.collider1 {
            PrimitiveContactGenerationContext {
                prediction_distance,
                collider1: collider2,
                collider2: collider1,
                shape1: collider2.shape(),
                shape2: &sub_shape1,
                position1: collider2.position(),
                position2: position1,
                manifold,
                workspace: sub_detector.workspace.as_deref_mut(),
            }
        } else {
            PrimitiveContactGenerationContext {
                prediction_distance,
                collider1,
                collider2,
                shape1: &sub_shape1,
                shape2: collider2.shape(),
                position1,
                position2: collider2.position(),
                manifold,
                workspace: sub_detector.workspace.as_deref_mut(),
            }
        };

        (sub_detector.generator.generate_contacts)(&mut ctxt2)
    });

    workspace
        .sub_detectors
        .retain(|_, detector| detector.timestamp == new_timestamp)
}
