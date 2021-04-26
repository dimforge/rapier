//! Physics pipeline structures.

use crate::data::{ComponentSet, ComponentSetMut};
use crate::dynamics::{
    IslandManager, JointSet, RigidBodyActivation, RigidBodyColliders, RigidBodyDominance,
    RigidBodyIds, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{BroadPhase, BroadPhasePairEvent, ColliderPair, ColliderShape, NarrowPhase};
use crate::math::Real;
use crate::pipeline::{EventHandler, PhysicsHooks};

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
    pub fn step<Bodies, Colliders>(
        &mut self,
        _prediction_distance: Real,
        _broad_phase: &mut BroadPhase,
        _narrow_phase: &mut NarrowPhase,
        _islands: &mut IslandManager,
        _bodies: &mut Bodies,
        _colliders: &mut Colliders,
        _hooks: &dyn PhysicsHooks<Bodies, Colliders>,
        _events: &dyn EventHandler,
    ) where
        Bodies: ComponentSetMut<RigidBodyIds>
            + ComponentSetMut<RigidBodyActivation>
            + ComponentSet<RigidBodyColliders>
            + ComponentSetMut<RigidBodyVelocity>
            + ComponentSet<RigidBodyDominance>
            + ComponentSet<RigidBodyType>,
        Colliders: ComponentSetMut<ColliderShape>,
    {
        unimplemented!()
    }
}
