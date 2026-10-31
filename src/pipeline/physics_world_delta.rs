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

/// Applies `$m!(field)` to every component of a world that has deltas.
macro_rules! for_each_component {
    ($m: ident) => {
        $m!(islands);
        $m!(broad_phase);
        $m!(narrow_phase);
        $m!(bodies);
        $m!(colliders);
        $m!(impulse_joints);
        $m!(multibody_joints);
        $m!(soft_bodies);
    };
}

/// A point in a world's change history: one [`ChangeTick`] per component.
///
/// Returned by [`PhysicsWorld::delta_since`] and [`PhysicsWorld::apply_delta`]; serializable,
/// so receivers can send it back to acknowledge a state.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct WorldChangeTick {
    /// The island manager's tick.
    pub islands: ChangeTick<IslandManager>,
    /// The broad phase's tick.
    pub broad_phase: ChangeTick<BroadPhaseBvh>,
    /// The narrow phase's tick.
    pub narrow_phase: ChangeTick<NarrowPhase>,
    /// The rigid-body set's tick.
    pub bodies: ChangeTick<RigidBodySet>,
    /// The collider set's tick.
    pub colliders: ChangeTick<ColliderSet>,
    /// The impulse-joint set's tick.
    pub impulse_joints: ChangeTick<ImpulseJointSet>,
    /// The multibody-joint set's tick.
    pub multibody_joints: ChangeTick<MultibodyJointSet>,
    /// The soft-body set's tick.
    pub soft_bodies: ChangeTick<SoftBodySet>,
}

/// The changes of a world since a baseline, produced by [`PhysicsWorld::delta_since`]: a serde
/// type, applied with [`PhysicsWorld::apply_delta`].
///
/// The gravity and the integration parameters (a few bytes) are always included.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorldDelta {
    /// The sender's gravity.
    pub gravity: Vector,
    /// The sender's integration parameters.
    pub integration_parameters: IntegrationParameters,
    /// The island manager's delta.
    pub islands: Delta<IslandManager>,
    /// The broad phase's delta.
    pub broad_phase: Delta<BroadPhaseBvh>,
    /// The narrow phase's delta.
    pub narrow_phase: Delta<NarrowPhase>,
    /// The rigid-body set's delta.
    pub bodies: Delta<RigidBodySet>,
    /// The collider set's delta.
    pub colliders: Delta<ColliderSet>,
    /// The impulse-joint set's delta.
    pub impulse_joints: Delta<ImpulseJointSet>,
    /// The multibody-joint set's delta.
    pub multibody_joints: Delta<MultibodyJointSet>,
    /// The soft-body set's delta.
    pub soft_bodies: Delta<SoftBodySet>,
}

impl WorldDelta {
    /// Whether this delta carries the whole world state (produced with no baseline).
    pub fn is_full(&self) -> bool {
        self.bodies.is_full()
    }

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

/// Mutable access to every part of a world that deltas cover, for code storing them outside a
/// [`PhysicsWorld`] (like the language bindings).
///
/// The world-level delta operations are defined here once; building this struct lists every
/// part, so a part added later is a compile error at each such call site.
#[allow(missing_docs)]
pub struct WorldComponentsMut<'a> {
    pub gravity: &'a mut Vector,
    pub integration_parameters: &'a mut IntegrationParameters,
    pub islands: &'a mut IslandManager,
    pub broad_phase: &'a mut BroadPhaseBvh,
    pub narrow_phase: &'a mut NarrowPhase,
    pub bodies: &'a mut RigidBodySet,
    pub colliders: &'a mut ColliderSet,
    pub impulse_joints: &'a mut ImpulseJointSet,
    pub multibody_joints: &'a mut MultibodyJointSet,
    pub soft_bodies: &'a mut SoftBodySet,
}

impl WorldComponentsMut<'_> {
    /// See [`PhysicsWorld::enable_change_tracking`].
    pub fn enable_change_tracking(&mut self) {
        macro_rules! enable {
            ($field: ident) => {
                self.$field.enable_change_tracking();
            };
        }
        for_each_component!(enable);
    }

    /// See [`PhysicsWorld::enable_change_tracking_with_seed`].
    pub fn enable_change_tracking_with_seed(&mut self, seed: u64) {
        macro_rules! enable {
            ($field: ident) => {
                self.$field.enable_change_tracking_with_seed(seed);
            };
        }
        for_each_component!(enable);
    }

    /// See [`PhysicsWorld::disable_change_tracking`].
    pub fn disable_change_tracking(&mut self) {
        macro_rules! disable {
            ($field: ident) => {
                self.$field.disable_change_tracking();
            };
        }
        for_each_component!(disable);
    }

    /// See [`PhysicsWorld::is_change_tracking_enabled`].
    pub fn is_change_tracking_enabled(&self) -> bool {
        self.bodies.is_change_tracking_enabled()
    }

    /// See [`PhysicsWorld::change_tick`].
    pub fn change_tick(&mut self) -> Result<WorldChangeTick, DeltaError> {
        Ok(WorldChangeTick {
            islands: self.islands.change_tick()?,
            broad_phase: self.broad_phase.change_tick()?,
            narrow_phase: self.narrow_phase.change_tick()?,
            bodies: self.bodies.change_tick()?,
            colliders: self.colliders.change_tick()?,
            impulse_joints: self.impulse_joints.change_tick()?,
            multibody_joints: self.multibody_joints.change_tick()?,
            soft_bodies: self.soft_bodies.change_tick()?,
        })
    }

    /// See [`PhysicsWorld::set_delta_baseline`].
    pub fn set_delta_baseline(&mut self, tick: WorldChangeTick) {
        macro_rules! set {
            ($field: ident) => {
                self.$field.set_delta_baseline(tick.$field);
            };
        }
        for_each_component!(set);
    }

    /// See [`PhysicsWorld::delta_since`].
    pub fn delta_since(
        &mut self,
        since: Option<WorldChangeTick>,
    ) -> Result<(WorldDelta, WorldChangeTick), DeltaError> {
        macro_rules! delta {
            ($field: ident) => {
                self.$field.delta_since(since.map(|t| t.$field))?.0
            };
        }
        let delta = WorldDelta {
            gravity: *self.gravity,
            integration_parameters: *self.integration_parameters,
            islands: delta!(islands),
            broad_phase: delta!(broad_phase),
            narrow_phase: delta!(narrow_phase),
            bodies: delta!(bodies),
            colliders: delta!(colliders),
            impulse_joints: delta!(impulse_joints),
            multibody_joints: delta!(multibody_joints),
            soft_bodies: delta!(soft_bodies),
        };
        let tick = delta.target_tick();
        Ok((delta, tick))
    }

    /// See [`PhysicsWorld::apply_delta`].
    pub fn apply_delta(&mut self, delta: WorldDelta) -> Result<WorldChangeTick, DeltaError> {
        let tick = delta.target_tick();
        let WorldDelta {
            gravity,
            integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        } = delta;
        *self.gravity = gravity;
        *self.integration_parameters = integration_parameters;
        self.islands.apply_delta(islands)?;
        self.broad_phase.apply_delta(broad_phase)?;
        self.narrow_phase.apply_delta(narrow_phase)?;
        self.bodies.apply_delta(bodies)?;
        self.colliders.apply_delta(colliders)?;
        self.impulse_joints.apply_delta(impulse_joints)?;
        self.multibody_joints.apply_delta(multibody_joints)?;
        self.soft_bodies.apply_delta(soft_bodies)?;
        Ok(tick)
    }
}

impl PhysicsWorld {
    /// Mutable access to every part of this world that deltas cover.
    pub fn components_mut(&mut self) -> WorldComponentsMut<'_> {
        WorldComponentsMut {
            gravity: &mut self.gravity,
            integration_parameters: &mut self.integration_parameters,
            islands: &mut self.islands,
            broad_phase: &mut self.broad_phase,
            narrow_phase: &mut self.narrow_phase,
            bodies: &mut self.bodies,
            colliders: &mut self.colliders,
            impulse_joints: &mut self.impulse_joints,
            multibody_joints: &mut self.multibody_joints,
            soft_bodies: &mut self.soft_bodies,
        }
    }

    /// Starts stamping the writes of every component with change ticks, for
    /// [`Self::delta_since`]. Restarting it invalidates every tick issued before.
    ///
    /// See [`SnapshotDelta::enable_change_tracking`] about targets without OS randomness.
    pub fn enable_change_tracking(&mut self) {
        self.components_mut().enable_change_tracking();
    }

    /// Like [`Self::enable_change_tracking`], deriving the clock identities from `seed` (pass a
    /// random value that differs between runs).
    pub fn enable_change_tracking_with_seed(&mut self, seed: u64) {
        self.components_mut().enable_change_tracking_with_seed(seed);
    }

    /// Stops stamping writes, and drops the stamps.
    pub fn disable_change_tracking(&mut self) {
        self.components_mut().disable_change_tracking();
    }

    /// Whether writes are currently stamped.
    pub fn is_change_tracking_enabled(&self) -> bool {
        self.bodies.is_change_tracking_enabled()
    }

    /// Issues a tick naming the current state without building a delta: send it with a full
    /// snapshot, and the receiver restores that snapshot then calls
    /// [`Self::set_delta_baseline`] with it.
    pub fn change_tick(&mut self) -> Result<WorldChangeTick, DeltaError> {
        self.components_mut().change_tick()
    }

    /// Declares that this world holds the sender's state at `tick` (e.g. a snapshot taken right
    /// after [`Self::change_tick`]), so deltas from that tick can be applied.
    pub fn set_delta_baseline(&mut self, tick: WorldChangeTick) {
        self.components_mut().set_delta_baseline(tick);
    }

    /// The changes made to the world since `since` (`None`: the whole state), and the tick a
    /// receiver is at once it applied them. Requires [`Self::enable_change_tracking`].
    ///
    /// A receiver that applies every delta from its baseline keeps simulating identically,
    /// like a restored snapshot, and serializes to the same bytes (with `enhanced-determinism`
    /// when composite shapes are in contact; see the [`data::delta`](crate::data::delta)
    /// limitations).
    pub fn delta_since(
        &mut self,
        since: Option<WorldChangeTick>,
    ) -> Result<(WorldDelta, WorldChangeTick), DeltaError> {
        self.components_mut().delta_since(since)
    }

    /// Patches a delta produced by [`Self::delta_since`] into this world. An incremental delta
    /// requires the delta's baseline: the target of the last delta applied (or
    /// [`Self::set_delta_baseline`]), not stepped since. A full delta first resets the world's
    /// components, as a deserialization would. Returns the tick to acknowledge to the sender.
    ///
    /// On error, the components applied before the failing one keep their new state: apply a
    /// full delta to recover.
    pub fn apply_delta(&mut self, delta: WorldDelta) -> Result<WorldChangeTick, DeltaError> {
        self.components_mut().apply_delta(delta)
    }
}
