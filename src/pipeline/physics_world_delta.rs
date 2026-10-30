//! Snapshot deltas of a whole [`PhysicsWorld`]: its components' deltas, bundled.

use crate::data::{ChangeTick, Delta, DeltaError, SnapshotDelta};
use crate::dynamics::{
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBodySet,
    SoftBodySet,
};
use crate::geometry::{BroadPhaseBvh, ColliderSet, NarrowPhase};
use crate::math::Vector;
use crate::pipeline::PhysicsWorld;
use serde::{Deserialize, Serialize};

/// Applies `$m!(field, Type)` to every component of a [`PhysicsWorld`] that has deltas.
macro_rules! for_each_component {
    ($m: ident) => {
        $m!(islands, IslandManager);
        $m!(broad_phase, BroadPhaseBvh);
        $m!(narrow_phase, NarrowPhase);
        $m!(bodies, RigidBodySet);
        $m!(colliders, ColliderSet);
        $m!(impulse_joints, ImpulseJointSet);
        $m!(multibody_joints, MultibodyJointSet);
        $m!(soft_bodies, SoftBodySet);
    };
}

/// A point in a [`PhysicsWorld`]'s change history: one [`ChangeTick`] per component.
///
/// Returned by [`PhysicsWorld::delta_since`] and [`PhysicsWorld::apply_delta`]; serializable,
/// so receivers can send it back to acknowledge a state.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[allow(missing_docs)]
pub struct WorldChangeTick {
    pub islands: ChangeTick<IslandManager>,
    pub broad_phase: ChangeTick<BroadPhaseBvh>,
    pub narrow_phase: ChangeTick<NarrowPhase>,
    pub bodies: ChangeTick<RigidBodySet>,
    pub colliders: ChangeTick<ColliderSet>,
    pub impulse_joints: ChangeTick<ImpulseJointSet>,
    pub multibody_joints: ChangeTick<MultibodyJointSet>,
    pub soft_bodies: ChangeTick<SoftBodySet>,
}

/// The changes of a [`PhysicsWorld`] since a baseline, produced by
/// [`PhysicsWorld::delta_since`]: a serde type, applied with [`PhysicsWorld::apply_delta`].
///
/// The gravity and the integration parameters (a few bytes) are always included.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[allow(missing_docs)]
pub struct WorldDelta {
    pub gravity: Vector,
    pub integration_parameters: IntegrationParameters,
    pub islands: Delta<IslandManager>,
    pub broad_phase: Delta<BroadPhaseBvh>,
    pub narrow_phase: Delta<NarrowPhase>,
    pub bodies: Delta<RigidBodySet>,
    pub colliders: Delta<ColliderSet>,
    pub impulse_joints: Delta<ImpulseJointSet>,
    pub multibody_joints: Delta<MultibodyJointSet>,
    pub soft_bodies: Delta<SoftBodySet>,
}

impl WorldDelta {
    /// The tick a receiver is at once this delta is applied.
    pub fn target_tick(&self) -> WorldChangeTick {
        WorldChangeTick {
            islands: self.islands.target_tick(),
            broad_phase: self.broad_phase.target_tick(),
            narrow_phase: self.narrow_phase.target_tick(),
            bodies: self.bodies.target_tick(),
            colliders: self.colliders.target_tick(),
            impulse_joints: self.impulse_joints.target_tick(),
            multibody_joints: self.multibody_joints.target_tick(),
            soft_bodies: self.soft_bodies.target_tick(),
        }
    }
}

impl PhysicsWorld {
    /// Starts stamping the writes of every component with change ticks, for
    /// [`Self::delta_since`]. Restarting it invalidates every tick issued before.
    pub fn enable_change_tracking(&mut self) {
        macro_rules! enable {
            ($field: ident, $ty: ty) => {
                self.$field.enable_change_tracking();
            };
        }
        for_each_component!(enable);
    }

    /// Stops stamping writes, and drops the stamps.
    pub fn disable_change_tracking(&mut self) {
        macro_rules! disable {
            ($field: ident, $ty: ty) => {
                self.$field.disable_change_tracking();
            };
        }
        for_each_component!(disable);
    }

    /// The changes made to the world since `since` (`None`: the whole state), and the tick a
    /// receiver is at once it applied them. Requires [`Self::enable_change_tracking`].
    ///
    /// A receiver that applies every delta from its baseline serializes byte-identically to
    /// this world and keeps simulating identically, like a restored snapshot.
    pub fn delta_since(
        &mut self,
        since: Option<&WorldChangeTick>,
    ) -> Result<(WorldDelta, WorldChangeTick), DeltaError> {
        let (islands, _) = self.islands.delta_since(since.map(|t| t.islands))?;
        let (broad_phase, _) = self.broad_phase.delta_since(since.map(|t| t.broad_phase))?;
        let (narrow_phase, _) = self.narrow_phase.delta_since(since.map(|t| t.narrow_phase))?;
        let (bodies, _) = self.bodies.delta_since(since.map(|t| t.bodies))?;
        let (colliders, _) = self.colliders.delta_since(since.map(|t| t.colliders))?;
        let (impulse_joints, _) = self
            .impulse_joints
            .delta_since(since.map(|t| t.impulse_joints))?;
        let (multibody_joints, _) = self
            .multibody_joints
            .delta_since(since.map(|t| t.multibody_joints))?;
        let (soft_bodies, _) = self.soft_bodies.delta_since(since.map(|t| t.soft_bodies))?;
        let delta = WorldDelta {
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        };
        let tick = delta.target_tick();
        Ok((delta, tick))
    }

    /// Patches a delta produced by [`Self::delta_since`] into this world, which must hold the
    /// delta's baseline state (or be in any state for a full delta). Returns the tick to
    /// acknowledge to the sender.
    ///
    /// On error, the components applied before the failing one keep their new state: restore
    /// a full state (a snapshot, or a delta from `None`) to recover.
    pub fn apply_delta(&mut self, delta: WorldDelta) -> Result<WorldChangeTick, DeltaError> {
        let tick = delta.target_tick();
        self.gravity = delta.gravity;
        self.integration_parameters = delta.integration_parameters;
        self.islands.apply_delta(delta.islands)?;
        self.broad_phase.apply_delta(delta.broad_phase)?;
        self.narrow_phase.apply_delta(delta.narrow_phase)?;
        self.bodies.apply_delta(delta.bodies)?;
        self.colliders.apply_delta(delta.colliders)?;
        self.impulse_joints.apply_delta(delta.impulse_joints)?;
        self.multibody_joints.apply_delta(delta.multibody_joints)?;
        self.soft_bodies.apply_delta(delta.soft_bodies)?;
        Ok(tick)
    }
}
