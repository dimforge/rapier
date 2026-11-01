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

/// The number of world components with deltas.
pub(crate) const NUM_DELTA_COMPONENTS: usize = 8;

/// A delta component with its delta types erased, for the world operations that ignore them.
pub(crate) trait AnyDeltaComponent {
    fn enable_tracking(&mut self, seed: Option<u64>);
    fn disable_tracking(&mut self);
    fn is_tracking_enabled(&self) -> bool;
    fn state_id(&self) -> u32;
    fn clear_baseline(&mut self);
    /// Whether a tick can be issued (see [`SnapshotDelta::change_tick`]).
    fn check_can_issue(&self) -> Result<(), DeltaError>;
}

impl<T: crate::data::delta::DeltaComponent> AnyDeltaComponent for T {
    fn enable_tracking(&mut self, seed: Option<u64>) {
        match seed {
            Some(seed) => self.enable_change_tracking_with_seed(seed),
            None => self.enable_change_tracking(),
        }
    }
    fn disable_tracking(&mut self) {
        self.disable_change_tracking();
    }
    fn is_tracking_enabled(&self) -> bool {
        self.is_change_tracking_enabled()
    }
    fn state_id(&self) -> u32 {
        crate::data::delta::delta_state_id(self)
    }
    fn clear_baseline(&mut self) {
        crate::data::delta::clear_delta_baseline(self);
    }
    fn check_can_issue(&self) -> Result<(), DeltaError> {
        crate::data::delta::check_delta_since(self, None)
    }
}

/// The ids of the components' current states, renewed by every applied delta (the pipeline
/// drops its caches when they change).
pub(crate) fn delta_state_ids(
    components: &[&mut dyn AnyDeltaComponent; NUM_DELTA_COMPONENTS],
) -> [u32; NUM_DELTA_COMPONENTS] {
    core::array::from_fn(|i| components[i].state_id())
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
    /// Every component with deltas (the destructuring makes a new part a compile error here).
    fn each_mut(&mut self) -> [&mut dyn AnyDeltaComponent; NUM_DELTA_COMPONENTS] {
        let Self {
            gravity: _,
            integration_parameters: _,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        } = self;
        [
            &mut **islands,
            &mut **broad_phase,
            &mut **narrow_phase,
            &mut **bodies,
            &mut **colliders,
            &mut **impulse_joints,
            &mut **multibody_joints,
            &mut **soft_bodies,
        ]
    }

    /// See [`PhysicsWorld::enable_change_tracking`].
    pub fn enable_change_tracking(&mut self) {
        for component in self.each_mut() {
            component.enable_tracking(None);
        }
    }

    /// See [`PhysicsWorld::enable_change_tracking_with_seed`].
    pub fn enable_change_tracking_with_seed(&mut self, seed: u64) {
        for component in self.each_mut() {
            component.enable_tracking(Some(seed));
        }
    }

    /// See [`PhysicsWorld::disable_change_tracking`].
    pub fn disable_change_tracking(&mut self) {
        for component in self.each_mut() {
            component.disable_tracking();
        }
    }

    /// Rebuilds the derived data a deserialization or a delta leaves empty until the next step
    /// (replayable solver contacts and manifold normals, multibody jacobians and mass
    /// matrices), so it can be read right away. Nothing serialized changes.
    pub fn rebuild_derived_state(&mut self) {
        self.narrow_phase
            .rebuild_solver_contacts(self.bodies, self.colliders);
        // The solver builds the mass matrices with the substep dt of its highest-cadence group.
        let max_extra = self
            .bodies
            .iter()
            .filter(|(_, rb)| rb.is_dynamic() && !rb.is_sleeping())
            .map(|(_, rb)| rb.additional_solver_iterations().min(0xFFFF))
            .max()
            .unwrap_or(0);
        let params = &self.integration_parameters;
        let substeps = (params.num_solver_iterations + max_extra).max(1);
        let substep_dt = params.dt / substeps as crate::math::Real;
        self.multibody_joints
            .rebuild_derived_data(self.bodies, substep_dt);
    }

    /// See [`PhysicsWorld::is_change_tracking_enabled`].
    pub fn is_change_tracking_enabled(&self) -> bool {
        let Self {
            gravity: _,
            integration_parameters: _,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        } = self;
        let components: [&dyn AnyDeltaComponent; NUM_DELTA_COMPONENTS] = [
            &**islands,
            &**broad_phase,
            &**narrow_phase,
            &**bodies,
            &**colliders,
            &**impulse_joints,
            &**multibody_joints,
            &**soft_bodies,
        ];
        components.iter().all(|c| c.is_tracking_enabled())
    }

    /// See [`PhysicsWorld::change_tick`].
    pub fn change_tick(&mut self) -> Result<WorldChangeTick, DeltaError> {
        // Every clock or none advances.
        for component in self.each_mut() {
            component.check_can_issue()?;
        }
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
        let WorldChangeTick {
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        } = tick;
        self.islands.set_delta_baseline(islands);
        self.broad_phase.set_delta_baseline(broad_phase);
        self.narrow_phase.set_delta_baseline(narrow_phase);
        self.bodies.set_delta_baseline(bodies);
        self.colliders.set_delta_baseline(colliders);
        self.impulse_joints.set_delta_baseline(impulse_joints);
        self.multibody_joints.set_delta_baseline(multibody_joints);
        self.soft_bodies.set_delta_baseline(soft_bodies);
    }

    /// See [`PhysicsWorld::delta_since`].
    pub fn delta_since(
        &mut self,
        since: Option<WorldChangeTick>,
    ) -> Result<(WorldDelta, WorldChangeTick), DeltaError> {
        // Every clock or none advances.
        use crate::data::delta::check_delta_since as check;
        check(&*self.islands, since.map(|t| t.islands))?;
        check(&*self.broad_phase, since.map(|t| t.broad_phase))?;
        check(&*self.narrow_phase, since.map(|t| t.narrow_phase))?;
        check(&*self.bodies, since.map(|t| t.bodies))?;
        check(&*self.colliders, since.map(|t| t.colliders))?;
        check(&*self.impulse_joints, since.map(|t| t.impulse_joints))?;
        check(&*self.multibody_joints, since.map(|t| t.multibody_joints))?;
        check(&*self.soft_bodies, since.map(|t| t.soft_bodies))?;
        let delta = WorldDelta {
            gravity: *self.gravity,
            integration_parameters: *self.integration_parameters,
            islands: self.islands.delta_since(since.map(|t| t.islands))?.0,
            broad_phase: self
                .broad_phase
                .delta_since(since.map(|t| t.broad_phase))?
                .0,
            narrow_phase: self
                .narrow_phase
                .delta_since(since.map(|t| t.narrow_phase))?
                .0,
            bodies: self.bodies.delta_since(since.map(|t| t.bodies))?.0,
            colliders: self.colliders.delta_since(since.map(|t| t.colliders))?.0,
            impulse_joints: self
                .impulse_joints
                .delta_since(since.map(|t| t.impulse_joints))?
                .0,
            multibody_joints: (self.multibody_joints)
                .delta_since(since.map(|t| t.multibody_joints))?
                .0,
            soft_bodies: self
                .soft_bodies
                .delta_since(since.map(|t| t.soft_bodies))?
                .0,
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
        // A stale or duplicate delta changes nothing.
        use crate::data::delta::check_delta_baseline as check;
        check(&*self.islands, &islands)?;
        check(&*self.broad_phase, &broad_phase)?;
        check(&*self.narrow_phase, &narrow_phase)?;
        check(&*self.bodies, &bodies)?;
        check(&*self.colliders, &colliders)?;
        check(&*self.impulse_joints, &impulse_joints)?;
        check(&*self.multibody_joints, &multibody_joints)?;
        check(&*self.soft_bodies, &soft_bodies)?;
        self.colliders.check_delta_shapes(&colliders)?;
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
        self.rebuild_derived_state();
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

    /// Whether writes are currently stamped on every component.
    pub fn is_change_tracking_enabled(&self) -> bool {
        let components: [&dyn AnyDeltaComponent; NUM_DELTA_COMPONENTS] = [
            &self.islands,
            &self.broad_phase,
            &self.narrow_phase,
            &self.bodies,
            &self.colliders,
            &self.impulse_joints,
            &self.multibody_joints,
            &self.soft_bodies,
        ];
        components.iter().all(|c| c.is_tracking_enabled())
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
    /// Baseline mismatches and missing shapes are reported before anything changes. A
    /// malformed delta found while patching leaves the components before it patched: apply a
    /// full delta to recover.
    pub fn apply_delta(&mut self, delta: WorldDelta) -> Result<WorldChangeTick, DeltaError> {
        self.components_mut().apply_delta(delta)
    }
}
