#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::dynamics::RigidBodySet;
use crate::geometry::contact_generator::{
    ContactDispatcher, ContactGenerationContext, DefaultContactDispatcher,
};
use crate::geometry::proximity_detector::{
    DefaultProximityDispatcher, ProximityDetectionContext, ProximityDispatcher,
};
//#[cfg(feature = "simd-is-enabled")]
//use crate::geometry::{
//    contact_generator::ContactGenerationContextSimd,
//    proximity_detector::ProximityDetectionContextSimd, WBall,
//};
use crate::geometry::{
    BroadPhasePairEvent, ColliderHandle, ContactEvent, ProximityEvent, ProximityPair,
};
use crate::geometry::{ColliderSet, ContactManifold, ContactPair, InteractionGraph};
//#[cfg(feature = "simd-is-enabled")]
//use crate::math::{SimdFloat, SIMD_WIDTH};
use crate::ncollide::query::Proximity;
use crate::pipeline::EventHandler;
//use simba::simd::SimdValue;

/// The narrow-phase responsible for computing precise contact information between colliders.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct NarrowPhase {
    contact_graph: InteractionGraph<ContactPair>,
    proximity_graph: InteractionGraph<ProximityPair>,
    //    ball_ball: Vec<usize>,        // Workspace: Vec<*mut ContactPair>,
    //    shape_shape: Vec<usize>,      // Workspace: Vec<*mut ContactPair>,
    //    ball_ball_prox: Vec<usize>,   // Workspace: Vec<*mut ProximityPair>,
    //    shape_shape_prox: Vec<usize>, // Workspace: Vec<*mut ProximityPair>,
}

pub(crate) type ContactManifoldIndex = usize;

impl NarrowPhase {
    /// Creates a new empty narrow-phase.
    pub fn new() -> Self {
        Self {
            contact_graph: InteractionGraph::new(),
            proximity_graph: InteractionGraph::new(),
            //            ball_ball: Vec::new(),
            //            shape_shape: Vec::new(),
            //            ball_ball_prox: Vec::new(),
            //            shape_shape_prox: Vec::new(),
        }
    }

    /// The contact graph containing all contact pairs and their contact information.
    pub fn contact_graph(&self) -> &InteractionGraph<ContactPair> {
        &self.contact_graph
    }

    /// The proximity graph containing all proximity pairs and their proximity information.
    pub fn proximity_graph(&self) -> &InteractionGraph<ProximityPair> {
        &self.proximity_graph
    }

    // #[cfg(feature = "parallel")]
    // pub fn contact_pairs(&self) -> &[ContactPair] {
    //     &self.contact_graph.interactions
    // }

    // pub fn contact_pairs_mut(&mut self) -> &mut [ContactPair] {
    //     &mut self.contact_graph.interactions
    // }

    // #[cfg(feature = "parallel")]
    // pub(crate) fn contact_pairs_vec_mut(&mut self) -> &mut Vec<ContactPair> {
    //     &mut self.contact_graph.interactions
    // }

    pub(crate) fn remove_colliders(
        &mut self,
        handles: &[ColliderHandle],
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
    ) {
        for handle in handles {
            if let Some(collider) = colliders.get(*handle) {
                let proximity_graph_id = collider.proximity_graph_index;
                let contact_graph_id = collider.contact_graph_index;

                // Wake up every body in contact with the deleted collider.
                for (a, b, _) in self.contact_graph.interactions_with(contact_graph_id) {
                    if let Some(parent) = colliders.get(a).map(|c| c.parent) {
                        bodies.wake_up(parent, true)
                    }

                    if let Some(parent) = colliders.get(b).map(|c| c.parent) {
                        bodies.wake_up(parent, true)
                    }
                }

                // We have to manage the fact that one other collider will
                // have its graph index changed because of the node's swap-remove.
                if let Some(replacement) = self
                    .proximity_graph
                    .remove_node(proximity_graph_id)
                    .and_then(|h| colliders.get_mut(h))
                {
                    replacement.proximity_graph_index = proximity_graph_id;
                }

                if let Some(replacement) = self
                    .contact_graph
                    .remove_node(contact_graph_id)
                    .and_then(|h| colliders.get_mut(h))
                {
                    replacement.contact_graph_index = contact_graph_id;
                }
            }
        }
    }

    pub(crate) fn register_pairs(
        &mut self,
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
        broad_phase_events: &[BroadPhasePairEvent],
        events: &dyn EventHandler,
    ) {
        for event in broad_phase_events {
            match event {
                BroadPhasePairEvent::AddPair(pair) => {
                    // println!("Adding pair: {:?}", *pair);
                    if let (Some(co1), Some(co2)) =
                        colliders.get2_mut_internal(pair.collider1, pair.collider2)
                    {
                        if co1.parent == co2.parent {
                            // Same parents. Ignore collisions.
                            continue;
                        }

                        if co1.is_sensor() || co2.is_sensor() {
                            let gid1 = co1.proximity_graph_index;
                            let gid2 = co2.proximity_graph_index;

                            // NOTE: the collider won't have a graph index as long
                            // as it does not interact with anything.
                            if !InteractionGraph::<ProximityPair>::is_graph_index_valid(gid1) {
                                co1.proximity_graph_index =
                                    self.proximity_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<ProximityPair>::is_graph_index_valid(gid2) {
                                co2.proximity_graph_index =
                                    self.proximity_graph.graph.add_node(pair.collider2);
                            }

                            if self.proximity_graph.graph.find_edge(gid1, gid2).is_none() {
                                let dispatcher = DefaultProximityDispatcher;
                                let generator = dispatcher.dispatch(co1.shape(), co2.shape());
                                let interaction =
                                    ProximityPair::new(*pair, generator.0, generator.1);
                                let _ = self.proximity_graph.add_edge(
                                    co1.proximity_graph_index,
                                    co2.proximity_graph_index,
                                    interaction,
                                );
                            }
                        } else {
                            // NOTE: same code as above, but for the contact graph.
                            // TODO: refactor both pieces of code somehow?
                            let gid1 = co1.contact_graph_index;
                            let gid2 = co2.contact_graph_index;

                            // NOTE: the collider won't have a graph index as long
                            // as it does not interact with anything.
                            if !InteractionGraph::<ContactPair>::is_graph_index_valid(gid1) {
                                co1.contact_graph_index =
                                    self.contact_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<ContactPair>::is_graph_index_valid(gid2) {
                                co2.contact_graph_index =
                                    self.contact_graph.graph.add_node(pair.collider2);
                            }

                            if self.contact_graph.graph.find_edge(gid1, gid2).is_none() {
                                let dispatcher = DefaultContactDispatcher;
                                let generator = dispatcher.dispatch(co1.shape(), co2.shape());
                                let interaction = ContactPair::new(*pair, generator.0, generator.1);
                                let _ = self.contact_graph.add_edge(
                                    co1.contact_graph_index,
                                    co2.contact_graph_index,
                                    interaction,
                                );
                            }
                        }
                    }
                }
                BroadPhasePairEvent::DeletePair(pair) => {
                    if let (Some(co1), Some(co2)) =
                        colliders.get2_mut_internal(pair.collider1, pair.collider2)
                    {
                        if co1.is_sensor() || co2.is_sensor() {
                            let prox_pair = self
                                .proximity_graph
                                .remove_edge(co1.proximity_graph_index, co2.proximity_graph_index);

                            // Emit a proximity lost event if we had a proximity before removing the edge.
                            if let Some(prox) = prox_pair {
                                if prox.proximity != Proximity::Disjoint {
                                    let prox_event = ProximityEvent::new(
                                        pair.collider1,
                                        pair.collider2,
                                        prox.proximity,
                                        Proximity::Disjoint,
                                    );
                                    events.handle_proximity_event(prox_event)
                                }
                            }
                        } else {
                            let contact_pair = self
                                .contact_graph
                                .remove_edge(co1.contact_graph_index, co2.contact_graph_index);

                            // Emit a contact stopped event if we had a contact before removing the edge.
                            // Also wake up the dynamic bodies that were in contact.
                            if let Some(ctct) = contact_pair {
                                if ctct.has_any_active_contact() {
                                    bodies.wake_up(co1.parent, true);
                                    bodies.wake_up(co2.parent, true);

                                    events.handle_contact_event(ContactEvent::Stopped(
                                        pair.collider1,
                                        pair.collider2,
                                    ))
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn compute_proximities(
        &mut self,
        prediction_distance: f32,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) {
        par_iter_mut!(&mut self.proximity_graph.graph.edges).for_each(|edge| {
            let pair = &mut edge.weight;
            let co1 = &colliders[pair.pair.collider1];
            let co2 = &colliders[pair.pair.collider2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if (rb1.is_sleeping() || rb1.is_static()) && (rb2.is_sleeping() || rb2.is_static()) {
                // No need to update this contact because nothing moved.
                return;
            }

            let dispatcher = DefaultProximityDispatcher;
            if pair.detector.is_none() {
                // We need a redispatch for this detector.
                // This can happen, e.g., after restoring a snapshot of the narrow-phase.
                let (detector, workspace) = dispatcher.dispatch(co1.shape(), co2.shape());
                pair.detector = Some(detector);
                pair.detector_workspace = workspace;
            }

            let context = ProximityDetectionContext {
                dispatcher: &dispatcher,
                prediction_distance,
                colliders,
                pair,
            };

            context
                .pair
                .detector
                .unwrap()
                .detect_proximity(context, events);
        });

        /*
        // First, group pairs.
        // NOTE: the transmutes here are OK because the Vec are all cleared
        // before we leave this method.
        // We do this in order to avoid reallocating those vecs each time
        // we compute the contacts. Unsafe is necessary because we can't just
        // store a Vec<&mut ProximityPair> into the NarrowPhase struct without
        // polluting the World with lifetimes.
        let ball_ball_prox: &mut Vec<&mut ProximityPair> =
            unsafe { std::mem::transmute(&mut self.ball_ball_prox) };
        let shape_shape_prox: &mut Vec<&mut ProximityPair> =
            unsafe { std::mem::transmute(&mut self.shape_shape_prox) };

        let bodies = &bodies.bodies;

        // FIXME: don't iterate through all the interactions.
        for pair in &mut self.proximity_graph.interactions {
            let co1 = &colliders[pair.pair.collider1];
            let co2 = &colliders[pair.pair.collider2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if (rb1.is_sleeping() || !rb1.is_dynamic()) && (rb2.is_sleeping() || !rb2.is_dynamic())
            {
                // No need to update this proximity because nothing moved.
                continue;
            }

            match (co1.shape(), co2.shape()) {
                (Shape::Ball(_), Shape::Ball(_)) => ball_ball_prox.push(pair),
                _ => shape_shape_prox.push(pair),
            }
        }

        par_chunks_mut!(ball_ball_prox, SIMD_WIDTH).for_each(|pairs| {
            let context = ProximityDetectionContextSimd {
                dispatcher: &DefaultProximityDispatcher,
                prediction_distance,
                colliders,
                pairs,
            };
            context.pairs[0]
                .detector
                .detect_proximity_simd(context, events);
        });

        par_iter_mut!(shape_shape_prox).for_each(|pair| {
            let context = ProximityDetectionContext {
                dispatcher: &DefaultProximityDispatcher,
                prediction_distance,
                colliders,
                pair,
            };

            context.pair.detector.detect_proximity(context, events);
        });

        ball_ball_prox.clear();
        shape_shape_prox.clear();
        */
    }

    pub(crate) fn compute_contacts(
        &mut self,
        prediction_distance: f32,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) {
        par_iter_mut!(&mut self.contact_graph.graph.edges).for_each(|edge| {
            let pair = &mut edge.weight;
            let co1 = &colliders[pair.pair.collider1];
            let co2 = &colliders[pair.pair.collider2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if ((rb1.is_sleeping() || rb1.is_static()) && (rb2.is_sleeping() || rb2.is_static()))
                || (!rb1.is_dynamic() && !rb2.is_dynamic())
            {
                // No need to update this contact because nothing moved.
                return;
            }

            let dispatcher = DefaultContactDispatcher;
            if pair.generator.is_none() {
                // We need a redispatch for this generator.
                // This can happen, e.g., after restoring a snapshot of the narrow-phase.
                let (generator, workspace) = dispatcher.dispatch(co1.shape(), co2.shape());
                pair.generator = Some(generator);
                pair.generator_workspace = workspace;
            }

            let context = ContactGenerationContext {
                dispatcher: &dispatcher,
                prediction_distance,
                colliders,
                pair,
            };

            context
                .pair
                .generator
                .unwrap()
                .generate_contacts(context, events);
        });

        /*
        // First, group pairs.
        // NOTE: the transmutes here are OK because the Vec are all cleared
        // before we leave this method.
        // We do this in order to avoid reallocating those vecs each time
        // we compute the contacts. Unsafe is necessary because we can't just
        // store a Vec<&mut ContactPair> into the NarrowPhase struct without
        // polluting the World with lifetimes.
        let ball_ball: &mut Vec<&mut ContactPair> =
            unsafe { std::mem::transmute(&mut self.ball_ball) };
        let shape_shape: &mut Vec<&mut ContactPair> =
            unsafe { std::mem::transmute(&mut self.shape_shape) };

        let bodies = &bodies.bodies;

        // FIXME: don't iterate through all the interactions.
        for pair in &mut self.contact_graph.interactions {
            let co1 = &colliders[pair.pair.collider1];
            let co2 = &colliders[pair.pair.collider2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if (rb1.is_sleeping() || !rb1.is_dynamic()) && (rb2.is_sleeping() || !rb2.is_dynamic())
            {
                // No need to update this contact because nothing moved.
                continue;
            }

            match (co1.shape(), co2.shape()) {
                (Shape::Ball(_), Shape::Ball(_)) => ball_ball.push(pair),
                _ => shape_shape.push(pair),
            }
        }

        par_chunks_mut!(ball_ball, SIMD_WIDTH).for_each(|pairs| {
            let context = ContactGenerationContextSimd {
                dispatcher: &DefaultContactDispatcher,
                prediction_distance,
                colliders,
                pairs,
            };
            context.pairs[0]
                .generator
                .generate_contacts_simd(context, events);
        });

        par_iter_mut!(shape_shape).for_each(|pair| {
            let context = ContactGenerationContext {
                dispatcher: &DefaultContactDispatcher,
                prediction_distance,
                colliders,
                pair,
            };

            context.pair.generator.generate_contacts(context, events);
        });

        ball_ball.clear();
        shape_shape.clear();
        */
    }

    /// Retrieve all the interactions with at least one contact point, happening between two active bodies.
    // NOTE: this is very similar to the code from JointSet::select_active_interactions.
    pub(crate) fn sort_and_select_active_contacts<'a>(
        &'a mut self,
        bodies: &RigidBodySet,
        out_manifolds: &mut Vec<&'a mut ContactManifold>,
        out: &mut Vec<Vec<ContactManifoldIndex>>,
    ) {
        for out_island in &mut out[..bodies.num_islands()] {
            out_island.clear();
        }

        // FIXME: don't iterate through all the interactions.
        for inter in self.contact_graph.graph.edges.iter_mut() {
            for manifold in &mut inter.weight.manifolds {
                let rb1 = &bodies[manifold.body_pair.body1];
                let rb2 = &bodies[manifold.body_pair.body2];
                if manifold.num_active_contacts() != 0
                    && (!rb1.is_dynamic() || !rb1.is_sleeping())
                    && (!rb2.is_dynamic() || !rb2.is_sleeping())
                {
                    let island_index = if !rb1.is_dynamic() {
                        rb2.active_island_id
                    } else {
                        rb1.active_island_id
                    };

                    out[island_index].push(out_manifolds.len());
                    out_manifolds.push(manifold);
                }
            }
        }
    }
}
