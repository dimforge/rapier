use rapier::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodySet,
};
use rapier::geometry::{
    BroadPhaseBvh, ColliderSet, CollisionEvent, ContactForceEvent, DefaultBroadPhase, NarrowPhase,
};
use rapier::math::{Real, Vector};
use rapier::pipeline::{PhysicsHooks, PhysicsPipeline};
use std::sync::mpsc::Receiver;

pub struct PhysicsSnapshot {
    timestep_id: usize,
    broad_phase: Vec<u8>,
    narrow_phase: Vec<u8>,
    bodies: Vec<u8>,
    colliders: Vec<u8>,
    impulse_joints: Vec<u8>,
    multibody_joints: Vec<u8>,
    island_manager: Vec<u8>,
}

pub struct DeserializedPhysicsSnapshot {
    pub timestep_id: usize,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub island_manager: IslandManager,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
}

impl PhysicsSnapshot {
    pub fn new(
        timestep_id: usize,
        broad_phase: &DefaultBroadPhase,
        narrow_phase: &NarrowPhase,
        island_manager: &IslandManager,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
    ) -> bincode::Result<Self> {
        Ok(Self {
            timestep_id,
            broad_phase: bincode::serialize(broad_phase)?,
            narrow_phase: bincode::serialize(narrow_phase)?,
            island_manager: bincode::serialize(island_manager)?,
            bodies: bincode::serialize(bodies)?,
            colliders: bincode::serialize(colliders)?,
            impulse_joints: bincode::serialize(impulse_joints)?,
            multibody_joints: bincode::serialize(multibody_joints)?,
        })
    }

    #[profiling::function]
    pub fn restore(&self) -> bincode::Result<DeserializedPhysicsSnapshot> {
        Ok(DeserializedPhysicsSnapshot {
            timestep_id: self.timestep_id,
            broad_phase: bincode::deserialize(&self.broad_phase)?,
            narrow_phase: bincode::deserialize(&self.narrow_phase)?,
            island_manager: bincode::deserialize(&self.island_manager)?,
            bodies: bincode::deserialize(&self.bodies)?,
            colliders: bincode::deserialize(&self.colliders)?,
            impulse_joints: bincode::deserialize(&self.impulse_joints)?,
            multibody_joints: bincode::deserialize(&self.multibody_joints)?,
        })
    }

    pub fn print_snapshot_len(&self) {
        let total = self.broad_phase.len()
            + self.narrow_phase.len()
            + self.island_manager.len()
            + self.bodies.len()
            + self.colliders.len()
            + self.impulse_joints.len()
            + self.multibody_joints.len();
        println!("Snapshot length: {total}B");
        println!("|_ broad_phase: {}B", self.broad_phase.len());
        println!("|_ narrow_phase: {}B", self.narrow_phase.len());
        println!("|_ island_manager: {}B", self.island_manager.len());
        println!("|_ bodies: {}B", self.bodies.len());
        println!("|_ colliders: {}B", self.colliders.len());
        println!("|_ impulse_joints: {}B", self.impulse_joints.len());
        println!("|_ multibody_joints: {}B", self.multibody_joints.len());
    }
}

pub struct PhysicsState {
    pub islands: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub pipeline: PhysicsPipeline,
    pub integration_parameters: IntegrationParameters,
    pub gravity: Vector<Real>,
    pub hooks: Box<dyn PhysicsHooks>,
}

impl Default for PhysicsState {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsState {
    pub fn new() -> Self {
        Self {
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::default(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            gravity: Vector::y() * -9.81,
            hooks: Box::new(()),
        }
    }
}

pub struct PhysicsEvents {
    pub collision_events: Receiver<CollisionEvent>,
    pub contact_force_events: Receiver<ContactForceEvent>,
}

impl PhysicsEvents {
    pub fn poll_all(&self) {
        while self.collision_events.try_recv().is_ok() {}
        while self.contact_force_events.try_recv().is_ok() {}
    }
}
