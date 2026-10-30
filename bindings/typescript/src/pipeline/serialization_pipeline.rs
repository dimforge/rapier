use crate::dynamics::{
    RawImpulseJointSet, RawIntegrationParameters, RawIslandManager, RawMultibodyJointSet,
    RawRigidBodySet, RawSoftBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use js_sys::Uint8Array;
use rapier::data::{DeltaError, SnapshotDelta};
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBodySet,
    SoftBodySet,
};
use rapier::geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase};
use rapier::math::Vector;
use rapier::pipeline::{WorldChangeTick, WorldDelta};
use wasm_bindgen::prelude::*;

#[derive(Serialize)]
struct SerializableWorld<'a> {
    gravity: &'a Vector,
    integration_parameters: &'a IntegrationParameters,
    islands: &'a IslandManager,
    broad_phase: &'a DefaultBroadPhase,
    narrow_phase: &'a NarrowPhase,
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    impulse_joints: &'a ImpulseJointSet,
    multibody_joints: &'a MultibodyJointSet,
    soft_bodies: &'a SoftBodySet,
}

#[derive(Deserialize)]
struct DeserializableWorld {
    gravity: Vector,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    #[serde(default)]
    soft_bodies: SoftBodySet,
}

#[wasm_bindgen]
pub struct RawDeserializedWorld {
    gravity: Option<RawVector>,
    integrationParameters: Option<RawIntegrationParameters>,
    islands: Option<RawIslandManager>,
    broadPhase: Option<RawBroadPhase>,
    narrowPhase: Option<RawNarrowPhase>,
    bodies: Option<RawRigidBodySet>,
    colliders: Option<RawColliderSet>,
    impulse_joints: Option<RawImpulseJointSet>,
    multibody_joints: Option<RawMultibodyJointSet>,
    soft_bodies: Option<RawSoftBodySet>,
}

#[wasm_bindgen]
impl RawDeserializedWorld {
    pub fn takeGravity(&mut self) -> Option<RawVector> {
        self.gravity.take()
    }

    pub fn takeIntegrationParameters(&mut self) -> Option<RawIntegrationParameters> {
        self.integrationParameters.take()
    }

    pub fn takeIslandManager(&mut self) -> Option<RawIslandManager> {
        self.islands.take()
    }

    pub fn takeBroadPhase(&mut self) -> Option<RawBroadPhase> {
        self.broadPhase.take()
    }

    pub fn takeNarrowPhase(&mut self) -> Option<RawNarrowPhase> {
        self.narrowPhase.take()
    }

    pub fn takeBodies(&mut self) -> Option<RawRigidBodySet> {
        self.bodies.take()
    }

    pub fn takeColliders(&mut self) -> Option<RawColliderSet> {
        self.colliders.take()
    }

    pub fn takeImpulseJoints(&mut self) -> Option<RawImpulseJointSet> {
        self.impulse_joints.take()
    }

    pub fn takeMultibodyJoints(&mut self) -> Option<RawMultibodyJointSet> {
        self.multibody_joints.take()
    }

    pub fn takeSoftBodies(&mut self) -> Option<RawSoftBodySet> {
        self.soft_bodies.take()
    }
}

/// A world delta and the tick a receiver is at once it applied it, both bincode-serialized.
#[wasm_bindgen]
pub struct RawWorldDelta {
    delta: Vec<u8>,
    tick: Vec<u8>,
}

#[wasm_bindgen]
impl RawWorldDelta {
    /// The bincode bytes of the `WorldDelta`.
    #[wasm_bindgen(getter)]
    pub fn delta(&self) -> Uint8Array {
        Uint8Array::from(&self.delta[..])
    }

    /// The bincode bytes of the `WorldChangeTick` to acknowledge once the delta is applied.
    #[wasm_bindgen(getter)]
    pub fn tick(&self) -> Uint8Array {
        Uint8Array::from(&self.tick[..])
    }
}

fn delta_error(err: DeltaError) -> JsError {
    JsError::new(&format!("snapshot delta: {err}"))
}

#[wasm_bindgen]
pub struct RawSerializationPipeline;

#[wasm_bindgen]
impl RawSerializationPipeline {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawSerializationPipeline
    }

    pub fn serializeAll(
        &self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &RawIslandManager,
        broadPhase: &RawBroadPhase,
        narrowPhase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        soft_bodies: &RawSoftBodySet,
        impulse_joints: &RawImpulseJointSet,
        multibody_joints: &RawMultibodyJointSet,
    ) -> Option<Uint8Array> {
        let to_serialize = SerializableWorld {
            gravity: &gravity.0,
            integration_parameters: &integrationParameters.0,
            islands: &islands.0,
            broad_phase: &broadPhase.0,
            narrow_phase: &narrowPhase.0,
            bodies: &bodies.0,
            colliders: &colliders.0,
            impulse_joints: &impulse_joints.0,
            multibody_joints: &multibody_joints.0,
            soft_bodies: &soft_bodies.0,
        };
        let snap = bincode::serialize(&to_serialize).ok()?;
        Some(Uint8Array::from(&snap[..]))
    }

    pub fn deserializeAll(&self, data: Uint8Array) -> Option<RawDeserializedWorld> {
        let data = data.to_vec();
        let d: DeserializableWorld = bincode::deserialize(&data[..]).ok()?;
        Some(RawDeserializedWorld {
            gravity: Some(RawVector(d.gravity)),
            integrationParameters: Some(RawIntegrationParameters(d.integration_parameters)),
            islands: Some(RawIslandManager(d.islands)),
            broadPhase: Some(RawBroadPhase(d.broad_phase)),
            narrowPhase: Some(RawNarrowPhase(d.narrow_phase)),
            bodies: Some(RawRigidBodySet(d.bodies)),
            colliders: Some(RawColliderSet(d.colliders)),
            impulse_joints: Some(RawImpulseJointSet(d.impulse_joints)),
            multibody_joints: Some(RawMultibodyJointSet(d.multibody_joints)),
            soft_bodies: Some(RawSoftBodySet(d.soft_bodies)),
        })
    }
    /// Starts stamping the writes of every component with change ticks, for `deltaSince`.
    /// Restarting it invalidates every tick issued before.
    pub fn enableChangeTracking(
        &self,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        soft_bodies: &mut RawSoftBodySet,
        impulse_joints: &mut RawImpulseJointSet,
        multibody_joints: &mut RawMultibodyJointSet,
    ) {
        islands.0.enable_change_tracking();
        broadPhase.0.enable_change_tracking();
        narrowPhase.0.enable_change_tracking();
        bodies.0.enable_change_tracking();
        colliders.0.enable_change_tracking();
        soft_bodies.0.enable_change_tracking();
        impulse_joints.0.enable_change_tracking();
        multibody_joints.0.enable_change_tracking();
    }

    /// Stops stamping writes on every component, and drops the stamps.
    pub fn disableChangeTracking(
        &self,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        soft_bodies: &mut RawSoftBodySet,
        impulse_joints: &mut RawImpulseJointSet,
        multibody_joints: &mut RawMultibodyJointSet,
    ) {
        islands.0.disable_change_tracking();
        broadPhase.0.disable_change_tracking();
        narrowPhase.0.disable_change_tracking();
        bodies.0.disable_change_tracking();
        colliders.0.disable_change_tracking();
        soft_bodies.0.disable_change_tracking();
        impulse_joints.0.disable_change_tracking();
        multibody_joints.0.disable_change_tracking();
    }

    /// The changes since the serialized `WorldChangeTick` `since` (`undefined`: the whole
    /// state), as a bincode-serialized `WorldDelta`, with the tick it brings receivers to.
    ///
    /// Throws if change tracking is disabled, or if `since` is malformed or was not issued
    /// by the current change trackers.
    pub fn deltaSince(
        &self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        soft_bodies: &mut RawSoftBodySet,
        impulse_joints: &mut RawImpulseJointSet,
        multibody_joints: &mut RawMultibodyJointSet,
        since: Option<Uint8Array>,
    ) -> Result<RawWorldDelta, JsError> {
        let since: Option<WorldChangeTick> = since
            .map(|bytes| bincode::deserialize(&bytes.to_vec()[..]))
            .transpose()
            .map_err(|e| JsError::new(&format!("malformed snapshot delta tick: {e}")))?;
        let (islands, _) = islands
            .0
            .delta_since(since.map(|t| t.islands))
            .map_err(delta_error)?;
        let (broad_phase, _) = broadPhase
            .0
            .delta_since(since.map(|t| t.broad_phase))
            .map_err(delta_error)?;
        let (narrow_phase, _) = narrowPhase
            .0
            .delta_since(since.map(|t| t.narrow_phase))
            .map_err(delta_error)?;
        let (bodies, _) = bodies
            .0
            .delta_since(since.map(|t| t.bodies))
            .map_err(delta_error)?;
        let (colliders, _) = colliders
            .0
            .delta_since(since.map(|t| t.colliders))
            .map_err(delta_error)?;
        let (impulse_joints, _) = impulse_joints
            .0
            .delta_since(since.map(|t| t.impulse_joints))
            .map_err(delta_error)?;
        let (multibody_joints, _) = multibody_joints
            .0
            .delta_since(since.map(|t| t.multibody_joints))
            .map_err(delta_error)?;
        let (soft_bodies, _) = soft_bodies
            .0
            .delta_since(since.map(|t| t.soft_bodies))
            .map_err(delta_error)?;
        let delta = WorldDelta {
            gravity: gravity.0,
            integration_parameters: integrationParameters.0,
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
        let delta = bincode::serialize(&delta).map_err(|e| JsError::new(&e.to_string()))?;
        let tick = bincode::serialize(&tick).map_err(|e| JsError::new(&e.to_string()))?;
        Ok(RawWorldDelta { delta, tick })
    }

    /// Patches a bincode-serialized `WorldDelta` into the components, gravity and integration
    /// parameters, and returns the serialized `WorldChangeTick` to acknowledge to the sender.
    ///
    /// Throws if the bytes are malformed or the delta's baseline is not the current state; the
    /// components applied before the failing one then keep their new state.
    pub fn applyDelta(
        &self,
        data: Uint8Array,
        gravity: &mut RawVector,
        integrationParameters: &mut RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        soft_bodies: &mut RawSoftBodySet,
        impulse_joints: &mut RawImpulseJointSet,
        multibody_joints: &mut RawMultibodyJointSet,
    ) -> Result<Uint8Array, JsError> {
        let delta: WorldDelta = bincode::deserialize(&data.to_vec()[..])
            .map_err(|e| JsError::new(&format!("malformed snapshot delta: {e}")))?;
        let tick = delta.target_tick();
        gravity.0 = delta.gravity;
        integrationParameters.0 = delta.integration_parameters;
        islands.0.apply_delta(delta.islands).map_err(delta_error)?;
        broadPhase
            .0
            .apply_delta(delta.broad_phase)
            .map_err(delta_error)?;
        narrowPhase
            .0
            .apply_delta(delta.narrow_phase)
            .map_err(delta_error)?;
        bodies.0.apply_delta(delta.bodies).map_err(delta_error)?;
        colliders
            .0
            .apply_delta(delta.colliders)
            .map_err(delta_error)?;
        impulse_joints
            .0
            .apply_delta(delta.impulse_joints)
            .map_err(delta_error)?;
        multibody_joints
            .0
            .apply_delta(delta.multibody_joints)
            .map_err(delta_error)?;
        soft_bodies
            .0
            .apply_delta(delta.soft_bodies)
            .map_err(delta_error)?;
        let tick = bincode::serialize(&tick).map_err(|e| JsError::new(&e.to_string()))?;
        Ok(Uint8Array::from(&tick[..]))
    }
}
