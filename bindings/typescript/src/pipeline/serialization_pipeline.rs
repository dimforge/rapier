use crate::dynamics::{
    RawImpulseJointSet, RawIntegrationParameters, RawIslandManager, RawJointType,
    RawMultibodyJointSet, RawRigidBodySet, RawSoftBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use crate::utils::{self, FlatHandle};
use bincode::Options;
use js_sys::Uint8Array;
use rapier::data::{DeltaError, Index};
use rapier::dynamics::{
    GenericJoint, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodySet, SoftBodySet,
};
use rapier::geometry::{Collider, ColliderSet, DefaultBroadPhase, NarrowPhase, SharedShape};
use rapier::math::{Real, Vector};
use rapier::pipeline::{WorldChangeTick, WorldComponentsMut, WorldDelta};
use std::sync::Arc;
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

/// A world delta and the tick a receiver is at once it applied it, both serialized with their
/// header.
#[wasm_bindgen]
pub struct RawWorldDelta {
    delta: Vec<u8>,
    tick: Vec<u8>,
}

#[wasm_bindgen]
impl RawWorldDelta {
    /// The serialized `WorldDelta`.
    #[wasm_bindgen(getter)]
    pub fn delta(&self) -> Uint8Array {
        Uint8Array::from(&self.delta[..])
    }

    /// The serialized `WorldChangeTick` to acknowledge once the delta is applied.
    #[wasm_bindgen(getter)]
    pub fn tick(&self) -> Uint8Array {
        Uint8Array::from(&self.tick[..])
    }
}

/// The handles a set gained and lost through an applied delta.
#[derive(Default)]
struct SetChanges {
    added: Vec<FlatHandle>,
    removed: Vec<FlatHandle>,
}

/// The outcome of `applyDelta`: the tick to acknowledge, or the error that stopped the patch,
/// and what changed for the JS wrappers to follow (also when the patch failed partway).
#[wasm_bindgen]
pub struct RawAppliedDelta {
    tick: Option<Vec<u8>>,
    error: Option<JsValue>,
    bodies: SetChanges,
    colliders: SetChanges,
    impulse_joints: SetChanges,
    multibody_joints: SetChanges,
    soft_bodies: SetChanges,
    colliders_reparented: Vec<FlatHandle>,
    colliders_reshaped: Vec<FlatHandle>,
}

#[wasm_bindgen]
impl RawAppliedDelta {
    /// The serialized tick to acknowledge, or `undefined` if the patch failed.
    #[wasm_bindgen(getter)]
    pub fn tick(&self) -> Option<Uint8Array> {
        self.tick.as_ref().map(|tick| Uint8Array::from(&tick[..]))
    }

    /// The kinded `Error` that stopped the patch, or `undefined`.
    #[wasm_bindgen(getter)]
    pub fn error(&self) -> JsValue {
        self.error.clone().unwrap_or(JsValue::UNDEFINED)
    }

    #[wasm_bindgen(getter)]
    pub fn bodiesAdded(&self) -> Vec<FlatHandle> {
        self.bodies.added.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn bodiesRemoved(&self) -> Vec<FlatHandle> {
        self.bodies.removed.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn collidersAdded(&self) -> Vec<FlatHandle> {
        self.colliders.added.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn collidersRemoved(&self) -> Vec<FlatHandle> {
        self.colliders.removed.clone()
    }

    /// The colliders kept by the patch whose parent changed.
    #[wasm_bindgen(getter)]
    pub fn collidersReparented(&self) -> Vec<FlatHandle> {
        self.colliders_reparented.clone()
    }

    /// The colliders kept by the patch whose shape may have changed (or which hold a deformable
    /// collision mesh).
    #[wasm_bindgen(getter)]
    pub fn collidersReshaped(&self) -> Vec<FlatHandle> {
        self.colliders_reshaped.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn impulseJointsAdded(&self) -> Vec<FlatHandle> {
        self.impulse_joints.added.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn impulseJointsRemoved(&self) -> Vec<FlatHandle> {
        self.impulse_joints.removed.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn multibodyJointsAdded(&self) -> Vec<FlatHandle> {
        self.multibody_joints.added.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn multibodyJointsRemoved(&self) -> Vec<FlatHandle> {
        self.multibody_joints.removed.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesAdded(&self) -> Vec<FlatHandle> {
        self.soft_bodies.added.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesRemoved(&self) -> Vec<FlatHandle> {
        self.soft_bodies.removed.clone()
    }
}

/// An occupied arena slot: index, generation and the data compared across the patch.
type Slot<T> = (u32, u32, T);

/// The occupied slots of a set, sorted by index.
fn slots<T>(iter: impl Iterator<Item = (Index, T)>) -> Vec<Slot<T>> {
    let mut slots: Vec<_> = iter
        .map(|(handle, data)| {
            let (index, generation) = handle.into_raw_parts();
            (index, generation, data)
        })
        .collect();
    slots.sort_unstable_by_key(|slot| slot.0);
    slots
}

fn slot_handle<T>(slot: &Slot<T>) -> FlatHandle {
    utils::flat_handle(Index::from_raw_parts(slot.0, slot.1))
}

/// Merges the sorted slots of a set before and after the patch into `changes`, and calls `kept`
/// on the handles present in both.
fn diff_slots<T>(
    before: &[Slot<T>],
    after: &[Slot<T>],
    changes: &mut SetChanges,
    mut kept: impl FnMut(FlatHandle, &T, &T),
) {
    use core::cmp::Ordering;
    let (mut i, mut j) = (0, 0);
    loop {
        match (before.get(i), after.get(j)) {
            (None, None) => break,
            (Some(b), None) => {
                changes.removed.push(slot_handle(b));
                i += 1;
            }
            (None, Some(a)) => {
                changes.added.push(slot_handle(a));
                j += 1;
            }
            (Some(b), Some(a)) => match b.0.cmp(&a.0) {
                Ordering::Less => {
                    changes.removed.push(slot_handle(b));
                    i += 1;
                }
                Ordering::Greater => {
                    changes.added.push(slot_handle(a));
                    j += 1;
                }
                Ordering::Equal => {
                    if b.1 == a.1 {
                        kept(slot_handle(a), &b.2, &a.2);
                    } else {
                        changes.removed.push(slot_handle(b));
                        changes.added.push(slot_handle(a));
                    }
                    i += 1;
                    j += 1;
                }
            },
        }
    }
}

/// What a patch may change of a collider wrapper: its parent, and its shape.
struct ColliderSlotData {
    parent: Option<Index>,
    /// The shape (`None` for a deformable collider), kept alive across the patch so that equal
    /// pointers mean the same shape. This delays the core's pruning of unused received shapes
    /// by one patch.
    shape: Option<SharedShape>,
}

impl ColliderSlotData {
    fn new(co: &Collider) -> Self {
        ColliderSlotData {
            parent: co.parent().map(|p| p.0),
            // A clone would make the deformable mesh's next in-place update copy it.
            shape: co
                .deformable_mesh_ref()
                .is_none()
                .then(|| co.shared_shape().clone()),
        }
    }

    /// Whether the wrapper's cached shape may be stale once `self` became `after`.
    fn reshaped(&self, after: &Self) -> bool {
        match (&self.shape, &after.shape) {
            (Some(before), Some(after)) => {
                // Patches rebuild the shapes they carry inline behind a new pointer.
                Arc::as_ptr(&before.0) as *const () != Arc::as_ptr(&after.0) as *const ()
                    && !same_content(before, after)
            }
            // A deformable mesh may be rewritten in place, behind the same pointer.
            _ => true,
        }
    }
}

/// Whether two shapes serialize to the same bytes (`false` if either does not serialize).
fn same_content(a: &SharedShape, b: &SharedShape) -> bool {
    /// A writer that checks the bytes written against `expected`, failing at the first mismatch.
    struct Compare<'a>(&'a [u8]);
    impl std::io::Write for Compare<'_> {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            match self.0.strip_prefix(buf) {
                Some(rest) => {
                    self.0 = rest;
                    Ok(buf.len())
                }
                None => Err(std::io::ErrorKind::InvalidData.into()),
            }
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    if a.shape_type() != b.shape_type() {
        return false;
    }
    let Ok(expected) = bincode::serialize(a) else {
        return false;
    };
    let mut compare = Compare(&expected);
    bincode::serialize_into(&mut compare, b).is_ok() && compare.0.is_empty()
}

/// The type of a joint, as `RawJointType` (which the JS wrapper classes follow).
fn joint_type(data: &GenericJoint) -> u32 {
    RawJointType::from(data.locked_axes) as u32
}

/// Diffs joint slots, reporting a joint whose type changed as removed and added.
fn diff_joint_slots(before: &[Slot<u32>], after: &[Slot<u32>], changes: &mut SetChanges) {
    let mut retyped = Vec::new();
    diff_slots(before, after, changes, |handle, b, a| {
        if b != a {
            retyped.push(handle);
        }
    });
    changes.removed.extend_from_slice(&retyped);
    changes.added.extend(retyped);
}

/// The occupied slots of every set, recorded before and after a patch.
struct WorldSlots {
    bodies: Vec<Slot<()>>,
    colliders: Vec<Slot<ColliderSlotData>>,
    /// The joint types (as `RawJointType`): a full delta onto an independently built world can
    /// put a joint of another type behind a handle the receiver already uses.
    impulse_joints: Vec<Slot<u32>>,
    /// The joint types (as `RawJointType`): a multibody joint's handle is its child body's, so
    /// it survives the joint's replacement by one of another type.
    multibody_joints: Vec<Slot<u32>>,
    soft_bodies: Vec<Slot<()>>,
}

impl WorldSlots {
    fn record(
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        impulse_joints: &RawImpulseJointSet,
        multibody_joints: &RawMultibodyJointSet,
        soft_bodies: &RawSoftBodySet,
    ) -> Self {
        WorldSlots {
            bodies: slots(bodies.0.iter().map(|(h, _)| (h.0, ()))),
            colliders: slots(
                colliders
                    .0
                    .iter()
                    .map(|(h, co)| (h.0, ColliderSlotData::new(co))),
            ),
            impulse_joints: slots(
                impulse_joints
                    .0
                    .iter()
                    .map(|(h, joint)| (h.0, joint_type(&joint.data))),
            ),
            multibody_joints: slots(
                multibody_joints
                    .0
                    .iter()
                    .map(|(h, _, _, link)| (h.0, joint_type(&link.joint().data))),
            ),
            soft_bodies: slots(soft_bodies.0.iter().map(|(h, _)| (h.0, ()))),
        }
    }

    /// What changed from `self` (before the patch) to `after`.
    fn changes(&self, after: &WorldSlots, result: Result<Vec<u8>, JsValue>) -> RawAppliedDelta {
        let (tick, error) = match result {
            Ok(tick) => (Some(tick), None),
            Err(err) => (None, Some(err)),
        };
        let mut applied = RawAppliedDelta {
            tick,
            error,
            bodies: SetChanges::default(),
            colliders: SetChanges::default(),
            impulse_joints: SetChanges::default(),
            multibody_joints: SetChanges::default(),
            soft_bodies: SetChanges::default(),
            colliders_reparented: Vec::new(),
            colliders_reshaped: Vec::new(),
        };
        diff_slots(
            &self.bodies,
            &after.bodies,
            &mut applied.bodies,
            |_, _, _| {},
        );
        let (reparented, reshaped) = (
            &mut applied.colliders_reparented,
            &mut applied.colliders_reshaped,
        );
        diff_slots(
            &self.colliders,
            &after.colliders,
            &mut applied.colliders,
            |handle, b, a| {
                if b.parent != a.parent {
                    reparented.push(handle);
                }
                if b.reshaped(a) {
                    reshaped.push(handle);
                }
            },
        );
        // The JS wrapper's class follows the joint type: rebuild it.
        diff_joint_slots(
            &self.impulse_joints,
            &after.impulse_joints,
            &mut applied.impulse_joints,
        );
        diff_joint_slots(
            &self.multibody_joints,
            &after.multibody_joints,
            &mut applied.multibody_joints,
        );
        diff_slots(
            &self.soft_bodies,
            &after.soft_bodies,
            &mut applied.soft_bodies,
            |_, _, _| {},
        );
        applied
    }
}

/// The largest delta or tick accepted, header included, like the C and Python bindings.
const MAX_DELTA_BYTES: usize = 256 * 1024 * 1024;
/// The largest payload encoded or decoded, so that every encoded delta or tick decodes.
const MAX_PAYLOAD_BYTES: usize = MAX_DELTA_BYTES - HEADER_LEN;

/// The magic opening every serialized delta.
const DELTA_MAGIC: [u8; 4] = *b"RJSD";
/// The magic opening every serialized tick.
const TICK_MAGIC: [u8; 4] = *b"RJST";
/// The version of the delta and tick encoding, bumped whenever their layout changes.
const FORMAT_VERSION: u16 = 1;
#[cfg(feature = "dim2")]
const DIMENSION: u8 = 2;
#[cfg(feature = "dim3")]
const DIMENSION: u8 = 3;
const SCALAR_SIZE: u8 = core::mem::size_of::<Real>() as u8;
/// The header: magic, little-endian format version, dimension and scalar size in bytes.
const HEADER_LEN: usize = 8;

fn header(magic: [u8; 4]) -> [u8; HEADER_LEN] {
    let version = FORMAT_VERSION.to_le_bytes();
    [
        magic[0],
        magic[1],
        magic[2],
        magic[3],
        version[0],
        version[1],
        DIMENSION,
        SCALAR_SIZE,
    ]
}

/// Checks the header of a serialized `what` opened by `magic`, and returns the payload.
fn check_header<'a>(what: &str, magic: [u8; 4], bytes: &'a [u8]) -> Result<&'a [u8], JsValue> {
    let malformed =
        |reason: String| kinded_error(INVALID_BYTES, &format!("malformed {what}: {reason}"));
    if bytes.len() < HEADER_LEN {
        return Err(malformed("too short for the rapier.js header".into()));
    }
    let (head, payload) = bytes.split_at(HEADER_LEN);
    if head[..4] != magic {
        let (expected, other, found) = if magic == DELTA_MAGIC {
            ("a delta", TICK_MAGIC, "a tick")
        } else {
            ("a tick", DELTA_MAGIC, "a delta")
        };
        return Err(malformed(if head[..4] == other {
            format!("expected {expected}, got {found}")
        } else {
            "not produced by rapier.js (bad magic)".into()
        }));
    }
    let version = u16::from_le_bytes([head[4], head[5]]);
    if version != FORMAT_VERSION {
        return Err(malformed(format!(
            "format version {version}, this rapier.js build reads version {FORMAT_VERSION}"
        )));
    }
    if head[6] != DIMENSION {
        return Err(malformed(format!(
            "produced by a {}D build, this is a {DIMENSION}D build",
            head[6]
        )));
    }
    if head[7] != SCALAR_SIZE {
        return Err(malformed(format!(
            "produced by a build with {}-byte scalars, this build uses {SCALAR_SIZE}-byte scalars",
            head[7]
        )));
    }
    Ok(payload)
}

/// The bincode options of deltas and ticks: the encoding of `bincode::serialize`, with a size
/// limit and no trailing bytes when decoding.
fn delta_options() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(MAX_PAYLOAD_BYTES as u64)
        .reject_trailing_bytes()
}

/// The error kind of bytes that are not a delta or tick of this build, rejected before any write.
const INVALID_BYTES: &str = "invalid_bytes";

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

/// Serializes `value` behind the `magic` header, failing (`kind`: `invalid_bytes`) past 256 MiB.
fn encode<T: serde::Serialize>(what: &str, magic: [u8; 4], value: &T) -> Result<Vec<u8>, JsValue> {
    let mut bytes = header(magic).to_vec();
    delta_options()
        .serialize_into(&mut bytes, value)
        .map_err(|e| {
            let reason = match *e {
                bincode::ErrorKind::SizeLimit => "exceeds 256 MiB".to_string(),
                e => e.to_string(),
            };
            kinded_error(
                INVALID_BYTES,
                &format!("cannot encode the {what}: {reason}"),
            )
        })?;
    Ok(bytes)
}

fn decode<T: serde::de::DeserializeOwned>(
    what: &str,
    magic: [u8; 4],
    bytes: &Uint8Array,
) -> Result<T, JsValue> {
    if bytes.length() as usize > MAX_DELTA_BYTES {
        return Err(kinded_error(
            INVALID_BYTES,
            &format!("malformed {what}: exceeds 256 MiB"),
        ));
    }
    let bytes = bytes.to_vec();
    let payload = check_header(what, magic, &bytes)?;
    delta_options()
        .deserialize(payload)
        .map_err(|e| kinded_error(INVALID_BYTES, &format!("malformed {what}: {e}")))
}

fn decode_tick(bytes: &Uint8Array) -> Result<WorldChangeTick, JsValue> {
    decode("snapshot delta tick", TICK_MAGIC, bytes)
}

fn encode_tick(tick: &WorldChangeTick) -> Result<Vec<u8>, JsValue> {
    encode("snapshot delta tick", TICK_MAGIC, tick)
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
        Ok(Uint8Array::from(&encode_tick(&tick)?[..]))
    }

    /// Declares that the components hold the sender's state at the serialized `tick`.
    ///
    /// Throws (`kind`: `invalid_bytes`) if the bytes are not a tick.
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
        let tick = decode_tick(&tick)?;
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

    /// The changes since the serialized `WorldChangeTick` `since` (`undefined` or empty: the
    /// whole state), as a serialized `WorldDelta`, with the tick it brings receivers to.
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
        // An empty baseline is no baseline (a full delta), as in the C and Python bindings.
        let since = since
            .filter(|bytes| bytes.length() > 0)
            .map(|bytes| decode_tick(&bytes))
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
            delta: encode("snapshot delta", DELTA_MAGIC, &delta)?,
            tick: encode_tick(&tick)?,
        })
    }

    /// Patches a serialized `WorldDelta` into the components, gravity and integration
    /// parameters, rebuilds their derived state, and returns the serialized `WorldChangeTick`
    /// to acknowledge to the sender with the handles the patch added, removed or modified.
    ///
    /// Throws an `Error` with a `kind` property (see `World.applyDelta`) if the bytes do not
    /// decode, target another baseline or reference a missing shape, leaving the world
    /// untouched. A patch failing afterwards is reported by the result's `error`, with what the
    /// components applied before the failing one changed.
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
    ) -> Result<RawAppliedDelta, JsValue> {
        let delta: WorldDelta = decode("snapshot delta", DELTA_MAGIC, &data)?;
        let before = WorldSlots::record(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        );
        let result = components(
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
        .apply_delta(delta);
        // The core checks every baseline and shape before writing anything: nothing changed.
        if let Err(err @ (DeltaError::BaselineMismatch | DeltaError::MissingShape)) = result {
            return Err(delta_error(err));
        }
        let result = result
            .map_err(delta_error)
            .and_then(|tick| encode_tick(&tick));
        let after = WorldSlots::record(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
        );
        Ok(before.changes(&after, result))
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
