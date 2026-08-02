use rapier::geometry::{BroadPhaseBvh, BvhOptimizationStrategy, CollisionEvent, ContactForceEvent};
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
    PhysicsSnapshot::new(timestep_id, world).expect("Failed to create physics snapshot")
}

/// Restores a [`PhysicsWorld`] from a snapshot produced by [`snapshot_world`],
/// returning the timestep id the snapshot was taken at.
pub fn restore_world(world: &mut PhysicsWorld, snapshot: &PhysicsSnapshot) -> usize {
    let (restored, timestep_id) = snapshot
        .restore()
        .expect("Failed to restore physics snapshot");
    let PhysicsWorld {
        gravity,
        integration_parameters,
        physics_pipeline: _,
        islands,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver: _,
    } = restored;
    world.gravity = gravity;
    world.integration_parameters = integration_parameters;
    world.islands = islands;
    world.broad_phase = broad_phase;
    world.narrow_phase = narrow_phase;
    world.bodies = bodies;
    world.colliders = colliders;
    world.impulse_joints = impulse_joints;
    world.multibody_joints = multibody_joints;
    timestep_id
}

/// A serialized [`PhysicsWorld`], plus the timestep it was taken at.
///
/// The world serializes exactly the state a step reads (see [`PhysicsWorld`]); the
/// destructuring in [`restore_world`] is deliberate, so a new field there has to be
/// considered here rather than silently dropped.
#[derive(Clone)]
pub struct PhysicsSnapshot {
    timestep_id: usize,
    world: Vec<u8>,
}

impl PhysicsSnapshot {
    pub fn new(timestep_id: usize, world: &PhysicsWorld) -> bincode::Result<Self> {
        Ok(Self {
            timestep_id,
            world: bincode::serialize(world)?,
        })
    }

    #[profiling::function]
    pub fn restore(&self) -> bincode::Result<(PhysicsWorld, usize)> {
        Ok((bincode::deserialize(&self.world)?, self.timestep_id))
    }

    pub fn print_snapshot_len(&self) {
        println!("Snapshot length: {}B", self.world.len());
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
