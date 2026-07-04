use rapier::dynamics::{ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodySet};
use rapier::geometry::{
    BroadPhaseBvh, BvhOptimizationStrategy, ColliderSet, CollisionEvent, ContactForceEvent,
    DefaultBroadPhase, NarrowPhase,
};
use rapier::pipeline::PhysicsWorld;
use std::sync::mpsc::Receiver;

/// Which broad-phase acceleration structure the testbed builds for a scene.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum RapierBroadPhaseType {
    #[default]
    BvhSubtreeOptimizer,
    BvhWithoutOptimization,
}

impl RapierBroadPhaseType {
    pub fn init_broad_phase(self) -> BroadPhaseBvh {
        match self {
            RapierBroadPhaseType::BvhSubtreeOptimizer => {
                BroadPhaseBvh::with_optimization_strategy(BvhOptimizationStrategy::SubtreeOptimizer)
            }
            RapierBroadPhaseType::BvhWithoutOptimization => {
                BroadPhaseBvh::with_optimization_strategy(BvhOptimizationStrategy::None)
            }
        }
    }
}

/// Snapshots the full simulation state of a [`PhysicsWorld`].
pub fn snapshot_world(world: &PhysicsWorld, timestep_id: usize) -> PhysicsSnapshot {
    PhysicsSnapshot::new(
        timestep_id,
        &world.broad_phase,
        &world.narrow_phase,
        &world.islands,
        &world.bodies,
        &world.colliders,
        &world.impulse_joints,
        &world.multibody_joints,
    )
    .expect("Failed to create physics snapshot")
}

/// Restores a [`PhysicsWorld`] from a snapshot produced by [`snapshot_world`].
pub fn restore_world(world: &mut PhysicsWorld, snapshot: &PhysicsSnapshot) {
    let restored = snapshot
        .restore()
        .expect("Failed to restore physics snapshot");
    world.broad_phase = restored.broad_phase;
    world.narrow_phase = restored.narrow_phase;
    world.islands = restored.island_manager;
    world.bodies = restored.bodies;
    world.colliders = restored.colliders;
    world.impulse_joints = restored.impulse_joints;
    world.multibody_joints = restored.multibody_joints;
}

#[derive(Clone)]
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
