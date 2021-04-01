use crossbeam::channel::Receiver;
use rapier::dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, ContactEvent, IntersectionEvent, NarrowPhase};
use rapier::math::Vector;
use rapier::pipeline::{PhysicsHooks, PhysicsPipeline, QueryPipeline};

pub struct PhysicsSnapshot {
    timestep_id: usize,
    broad_phase: Vec<u8>,
    narrow_phase: Vec<u8>,
    bodies: Vec<u8>,
    colliders: Vec<u8>,
    joints: Vec<u8>,
}

impl PhysicsSnapshot {
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

pub struct PhysicsState {
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub joints: JointSet,
    pub ccd_solver: CCDSolver,
    pub pipeline: PhysicsPipeline,
    pub query_pipeline: QueryPipeline,
    pub integration_parameters: IntegrationParameters,
    pub gravity: Vector<f32>,
    pub hooks: Box<dyn PhysicsHooks>,
}

impl PhysicsState {
    pub fn new() -> Self {
        Self {
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

pub struct PhysicsEvents {
    pub contact_events: Receiver<ContactEvent>,
    pub intersection_events: Receiver<IntersectionEvent>,
}

impl PhysicsEvents {
    pub fn poll_all(&self) {
        while let Ok(_) = self.contact_events.try_recv() {}
        while let Ok(_) = self.intersection_events.try_recv() {}
    }
}
