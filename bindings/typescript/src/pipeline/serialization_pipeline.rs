use crate::dynamics::{
    RawImpulseJointSet, RawIntegrationParameters, RawIslandManager, RawMultibodyJointSet,
    RawRigidBodySet, RawSoftBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use bincode::Options;
use js_sys::Uint8Array;
use rapier::data::DeltaError;
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBodySet,
    SoftBodySet,
};
use rapier::geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase};
use rapier::math::Vector;
use rapier::pipeline::{WorldChangeTick, WorldComponentsMut, WorldDelta};
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

/// The largest delta or tick accepted, like the C bindings.
const MAX_DELTA_BYTES: usize = 256 * 1024 * 1024;

/// The bincode options of deltas and ticks: the encoding of `bincode::serialize`, with a size
/// limit and no trailing bytes when decoding.
fn delta_options() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(MAX_DELTA_BYTES as u64)
        .reject_trailing_bytes()
}

/// A JS `Error` whose `kind` property is a machine-readable error category.
fn kinded_error(kind: &str, message: &str) -> JsValue {
    let err = js_sys::Error::new(message);
    // Setting a property on a fresh `Error` object cannot fail.
    let _ = js_sys::Reflect::set(&err, &"kind".into(), &kind.into());
    err.into()
}

fn delta_error(err: DeltaError) -> JsValue {
    let kind = match err {
        DeltaError::TrackingDisabled => "tracking_disabled",
        DeltaError::UnknownBaseline => "unknown_baseline",
        DeltaError::BaselineMismatch => "baseline_mismatch",
        DeltaError::Malformed => "malformed",
        DeltaError::MissingShape => "missing_shape",
    };
    kinded_error(kind, &format!("snapshot delta: {err}"))
}

fn encode<T: serde::Serialize>(value: &T) -> Result<Vec<u8>, JsValue> {
    delta_options()
        .serialize(value)
        .map_err(|e| JsValue::from(js_sys::Error::new(&e.to_string())))
}

fn decode<T: serde::de::DeserializeOwned>(what: &str, bytes: &Uint8Array) -> Result<T, JsValue> {
    if bytes.length() as usize > MAX_DELTA_BYTES {
        return Err(kinded_error(
            "malformed",
            &format!("malformed {what}: exceeds 256 MiB"),
        ));
    }
    delta_options()
        .deserialize(&bytes.to_vec()[..])
        .map_err(|e| kinded_error("malformed", &format!("malformed {what}: {e}")))
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
        let mut d: DeserializableWorld = bincode::deserialize(&data[..]).ok()?;
        // Exposes contact normals and multibody jacobians before the first step.
        WorldComponentsMut {
            gravity: &mut d.gravity,
            integration_parameters: &mut d.integration_parameters,
            islands: &mut d.islands,
            broad_phase: &mut d.broad_phase,
            narrow_phase: &mut d.narrow_phase,
            bodies: &mut d.bodies,
            colliders: &mut d.colliders,
            impulse_joints: &mut d.impulse_joints,
            multibody_joints: &mut d.multibody_joints,
            soft_bodies: &mut d.soft_bodies,
        }
        .rebuild_derived_state();
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

    /// Starts stamping the writes of every component with change ticks, for `takeDelta`.
    /// Restarting it invalidates every tick issued before.
    ///
    /// The tracker identities derive from `seedHi` and `seedLo`, which must be random: std's
    /// default hasher keys are not random on `wasm32-unknown-unknown`.
    pub fn enableChangeTracking(
        &self,
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
        seedHi: u32,
        seedLo: u32,
    ) {
        let seed = ((seedHi as u64) << 32) | seedLo as u64;
        components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .enable_change_tracking_with_seed(seed);
    }

    /// Stops stamping writes on every component, and drops the stamps.
    pub fn disableChangeTracking(
        &self,
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
    ) {
        components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .disable_change_tracking();
    }

    /// The serialized `WorldChangeTick` naming the current state, to send with a snapshot.
    ///
    /// Throws (`kind`: `tracking_disabled`) if change tracking is disabled.
    pub fn changeTick(
        &self,
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
    ) -> Result<Uint8Array, JsValue> {
        let tick = components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .change_tick()
        .map_err(delta_error)?;
        Ok(Uint8Array::from(&encode(&tick)?[..]))
    }

    /// Declares that the components hold the sender's state at the serialized `tick`.
    ///
    /// Throws (`kind`: `malformed`) if the bytes are not a tick.
    pub fn setDeltaBaseline(
        &self,
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
        tick: Uint8Array,
    ) -> Result<(), JsValue> {
        let tick: WorldChangeTick = decode("snapshot delta tick", &tick)?;
        components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .set_delta_baseline(tick);
        Ok(())
    }

    /// The changes since the serialized `WorldChangeTick` `since` (`undefined`: the whole
    /// state), as a serialized `WorldDelta`, with the tick it brings receivers to.
    ///
    /// Throws an `Error` with a `kind` property (see `World.takeDelta`).
    pub fn takeDelta(
        &self,
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
        since: Option<Uint8Array>,
    ) -> Result<RawWorldDelta, JsValue> {
        let since: Option<WorldChangeTick> = since
            .map(|bytes| decode("snapshot delta tick", &bytes))
            .transpose()?;
        let (delta, tick) = components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .delta_since(since)
        .map_err(delta_error)?;
        Ok(RawWorldDelta {
            delta: encode(&delta)?,
            tick: encode(&tick)?,
        })
    }

    /// Patches a serialized `WorldDelta` into the components, gravity and integration
    /// parameters, rebuilds their derived state, and returns the serialized `WorldChangeTick`
    /// to acknowledge to the sender.
    ///
    /// Throws an `Error` with a `kind` property (see `World.applyDelta`). Once the bytes
    /// decoded, the gravity, integration parameters and the components applied before the
    /// failing one keep their new state.
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
    ) -> Result<Uint8Array, JsValue> {
        let delta: WorldDelta = decode("snapshot delta", &data)?;
        let tick = components(
            gravity,
            integrationParameters,
            islands,
            broadPhase,
            narrowPhase,
            bodies,
            colliders,
            soft_bodies,
            impulse_joints,
            multibody_joints,
        )
        .apply_delta(delta)
        .map_err(delta_error)?;
        Ok(Uint8Array::from(&encode(&tick)?[..]))
    }
}

/// Bundles the raw world parts for the world-level delta operations.
fn components<'a>(
    gravity: &'a mut RawVector,
    integrationParameters: &'a mut RawIntegrationParameters,
    islands: &'a mut RawIslandManager,
    broadPhase: &'a mut RawBroadPhase,
    narrowPhase: &'a mut RawNarrowPhase,
    bodies: &'a mut RawRigidBodySet,
    colliders: &'a mut RawColliderSet,
    soft_bodies: &'a mut RawSoftBodySet,
    impulse_joints: &'a mut RawImpulseJointSet,
    multibody_joints: &'a mut RawMultibodyJointSet,
) -> WorldComponentsMut<'a> {
    WorldComponentsMut {
        gravity: &mut gravity.0,
        integration_parameters: &mut integrationParameters.0,
        islands: &mut islands.0,
        broad_phase: &mut broadPhase.0,
        narrow_phase: &mut narrowPhase.0,
        bodies: &mut bodies.0,
        colliders: &mut colliders.0,
        impulse_joints: &mut impulse_joints.0,
        multibody_joints: &mut multibody_joints.0,
        soft_bodies: &mut soft_bodies.0,
    }
}
