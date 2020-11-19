//! Physics pipeline structures.

use crate::dynamics::{JointSet, RigidBodySet};
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderPair, ColliderSet, ContactPairFilter, NarrowPhase,
    ProximityPairFilter,
};
use crate::pipeline::EventHandler;

/// The collision pipeline, responsible for performing collision detection between colliders.
///
/// This structure only contains temporary data buffers. It can be dropped and replaced by a fresh
/// copy at any time. For performance reasons it is recommended to reuse the same physics pipeline
/// instance to benefit from the cached data.
// NOTE: this contains only workspace data, so there is no point in making this serializable.
pub struct CollisionPipeline {
    broadphase_collider_pairs: Vec<ColliderPair>,
    broad_phase_events: Vec<BroadPhasePairEvent>,
    empty_joints: JointSet,
}

#[allow(dead_code)]
fn check_pipeline_send_sync() {
    fn do_test<T: Sync>() {}
    do_test::<CollisionPipeline>();
}

impl CollisionPipeline {
    /// Initializes a new physics pipeline.
    pub fn new() -> CollisionPipeline {
        CollisionPipeline {
            broadphase_collider_pairs: Vec::new(),
            broad_phase_events: Vec::new(),
            empty_joints: JointSet::new(),
        }
    }

    /// Executes one step of the collision detection.
    pub fn step(
        &mut self,
        prediction_distance: f32,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        contact_pair_filter: Option<&dyn ContactPairFilter>,
        proximity_pair_filter: Option<&dyn ProximityPairFilter>,
        events: &dyn EventHandler,
    ) {
        bodies.maintain_active_set();
        self.broadphase_collider_pairs.clear();

        broad_phase.update_aabbs(prediction_distance, bodies, colliders);

        self.broad_phase_events.clear();
        broad_phase.find_pairs(&mut self.broad_phase_events);

        narrow_phase.register_pairs(colliders, bodies, &self.broad_phase_events, events);

        narrow_phase.compute_contacts(
            prediction_distance,
            bodies,
            colliders,
            contact_pair_filter,
            events,
        );
        narrow_phase.compute_proximities(
            prediction_distance,
            bodies,
            colliders,
            proximity_pair_filter,
            events,
        );

        bodies.update_active_set_with_contacts(
            colliders,
            narrow_phase,
            self.empty_joints.joint_graph(),
            0,
        );

        // // Update kinematic bodies velocities.
        // bodies.foreach_active_kinematic_body_mut_internal(|_, body| {
        //     body.compute_velocity_from_predicted_position(integration_parameters.inv_dt());
        // });

        // Update colliders positions and kinematic bodies positions.
        bodies.foreach_active_body_mut_internal(|_, rb| {
            if rb.is_kinematic() {
                rb.position = rb.predicted_position;
            } else {
                rb.update_predicted_position(0.0);
            }

            for handle in &rb.colliders {
                let collider = &mut colliders[*handle];
                collider.position = rb.position * collider.delta;
                collider.predicted_position = rb.predicted_position * collider.delta;
            }
        });

        bodies.modified_inactive_set.clear();
    }
}
