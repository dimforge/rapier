//! Physics related state
use crate::dynamics::{CCDSolver, IntegrationParameters, IslandManager, JointSet, RigidBodySet};
use crate::geometry::{BroadPhase, ColliderSet, ContactEvent, IntersectionEvent, NarrowPhase};
use crate::math::Vector;
use crate::pipeline::{PhysicsHooks, PhysicsPipeline, QueryPipeline};
use crossbeam::channel::Receiver;
use parry::math::Real;

#[cfg(feature = "serde")]
/// Shapshot of physics
pub struct PhysicsSnapshot {
    timestep_id: usize,
    broad_phase: Vec<u8>,
    narrow_phase: Vec<u8>,
    bodies: Vec<u8>,
    colliders: Vec<u8>,
    joints: Vec<u8>,
}

#[cfg(feature = "serde")]
impl PhysicsSnapshot {
    /// Create a new `PhysicsSnapshot` from provided parameters
    pub fn new(
        timestep_id: usize,
        broad_phase: &BroadPhase,
        narrow_phase: &NarrowPhase,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        joints: &JointSet,
    ) -> bincode::Result<Self> {
        Ok(Self {
            timestep_id,
            broad_phase: bincode::serialize(broad_phase)?,
            narrow_phase: bincode::serialize(narrow_phase)?,
            bodies: bincode::serialize(bodies)?,
            colliders: bincode::serialize(colliders)?,
            joints: bincode::serialize(joints)?,
        })
    }

    /// Restore physics snapshot from fields
    pub fn restore(
        &self,
    ) -> bincode::Result<(
        usize,
        BroadPhase,
        NarrowPhase,
        RigidBodySet,
        ColliderSet,
        JointSet,
    )> {
        Ok((
            self.timestep_id,
            bincode::deserialize(&self.broad_phase)?,
            bincode::deserialize(&self.narrow_phase)?,
            bincode::deserialize(&self.bodies)?,
            bincode::deserialize(&self.colliders)?,
            bincode::deserialize(&self.joints)?,
        ))
    }

    /// Print the snapshot length
    pub fn print_snapshot_len(&self) {
        let total = self.broad_phase.len()
            + self.narrow_phase.len()
            + self.bodies.len()
            + self.colliders.len()
            + self.joints.len();
        println!("Snapshot length: {}B", total);
        println!("|_ broad_phase: {}B", self.broad_phase.len());
        println!("|_ narrow_phase: {}B", self.narrow_phase.len());
        println!("|_ bodies: {}B", self.bodies.len());
        println!("|_ colliders: {}B", self.colliders.len());
        println!("|_ joints: {}B", self.joints.len());
    }
}

/// `PhysicsState` contains the complete state of a rapier simulation
pub struct PhysicsState {
    /// Islands
    pub islands: IslandManager,
    /// Broad Phase
    pub broad_phase: BroadPhase,
    /// Narrow Phase
    pub narrow_phase: NarrowPhase,
    /// Bodies
    pub bodies: RigidBodySet,
    /// Colliders
    pub colliders: ColliderSet,
    /// Joints
    pub joints: JointSet,
    /// CCD Solver
    pub ccd_solver: CCDSolver,
    /// Physics Pipeline
    pub pipeline: PhysicsPipeline,
    /// Query Pipeline
    pub query_pipeline: QueryPipeline,
    /// Integration Params
    pub integration_parameters: IntegrationParameters,
    /// Gravity `Vector`
    pub gravity: Vector<Real>,
    /// Physics Hooks
    pub hooks: Box<dyn PhysicsHooks<RigidBodySet, ColliderSet>>,
}

impl PhysicsState {
    /// Create a new, empty physics state
    pub fn new() -> Self {
        Self {
            islands: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            joints: JointSet::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            query_pipeline: QueryPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            gravity: Vector::y() * -9.81,
            hooks: Box::new(()),
        }
    }
}

/// Physics events, that plugins, and callbacks can subscribe to
pub struct PhysicsEvents {
    /// The `ContactEvents` that have been emitted
    pub contact_events: Receiver<ContactEvent>,
    /// The `IntersectionEvents` that have been emitted
    pub intersection_events: Receiver<IntersectionEvent>,
}

impl PhysicsEvents {
    /// Poll all events
    pub fn poll_all(&self) {
        while let Ok(_) = self.contact_events.try_recv() {}
        while let Ok(_) = self.intersection_events.try_recv() {}
    }
}
