use crate::dynamics::{
    RawImpulseJointSet, RawIntegrationParameters, RawIslandManager, RawMultibodyJointSet,
    RawRigidBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use js_sys::Uint8Array;
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBodySet,
};
use rapier::geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase};
use rapier::math::Vector;
use wasm_bindgen::prelude::*;

#[derive(Serialize)]
struct SerializableWorld<'a> {
    gravity: &'a Vector<f32>,
    integration_parameters: &'a IntegrationParameters,
    islands: &'a IslandManager,
    broad_phase: &'a DefaultBroadPhase,
    narrow_phase: &'a NarrowPhase,
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    impulse_joints: &'a ImpulseJointSet,
    multibody_joints: &'a MultibodyJointSet,
}

#[derive(Deserialize)]
struct DeserializableWorld {
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
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
        })
    }
}
