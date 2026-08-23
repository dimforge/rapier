//! Workspace and context of the contact assembly: the per-body and per-mesh contact state kept across steps, the step-constant inputs and the small helpers the passes share.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;
use parry::utils::hashmap::HashMap;

use super::super::soft_contact::SoftContact;
use crate::dynamics::soft_body::{
    SoftCollisionMesh, SoftEdgeContact, SoftMeshId, SoftMeshRef, SoftVertexContact,
};
use crate::dynamics::{IntegrationParameters, RigidBodySet, SoftBody, SoftBodyHandle, SoftBodySet};
use crate::geometry::{Collider, ColliderHandle, ColliderSet, NarrowPhase};
use crate::dynamics::soft_body::{SoftOverlapState, SoftVolumeContact};
use crate::math::{AngVector, Real, Rotation, Vector};

/// The collision mesh a soft body's collider carries.
pub(super) fn mesh_of(sb: &SoftBody, collider: ColliderHandle) -> Option<&SoftCollisionMesh> {
    sb.mesh_of(collider)
}

pub(super) fn mesh_ref(co: &Collider) -> SoftMeshRef {
    co.deformable_mesh_ref.expect("not a deformable collider")
}

/// Scratch of the contact assembly kept across steps.
#[derive(Default)]
pub(crate) struct EdgeContactWorkspace {
    /// Awake index of every awake soft body, by handle.
    pub(super) awake_of: HashMap<SoftBodyHandle, usize>,
    /// The rows and new edge contacts of every awake body (assembled in parallel, appended in
    /// awake order).
    pub(super) per_body: Vec<BodyContacts>,
}

/// `SoftContactSource::manifold` of the rows without a manifold point: an edge-vs-edge row (its
/// warm start lives in the owner's `edge_contacts`) or a vertex-vs-surface row (`vertex_contacts`).
pub(super) const SOURCE_EDGE_CONTACT: u32 = u32::MAX;
pub(super) const SOURCE_VERTEX_CONTACT: u32 = u32::MAX - 1;
/// The contact rows of one awake soft body, and its new edge-vs-edge and vertex-vs-surface
/// contacts (the rows' `point` indices are positions in those lists).
#[derive(Default)]
pub(super) struct BodyContacts {
    pub(super) contacts: Vec<SoftContact>,
    /// The contact state of every mesh assembled this step, in the order the body's meshes were
    /// visited (a row's `point` indexes its own mesh's lists).
    pub(super) meshes: Vec<MeshContacts>,
    /// The mesh being assembled.
    pub(super) current_mesh: usize,
    /// Largest normal approach speed of the rigid bodies met (the impact-adaptive substeps).
    pub(super) max_approach_speed: Real,
    /// The owner's edge contacts of the last step, by (other mesh, edge, other edge).
    pub(super) previous: HashMap<(SoftMeshRef, u32, u32), (Real, Vector)>,
    /// The owner's vertex contacts of the last step with the current other body (and
    /// vertex pass), by (vertex, element).
    pub(super) previous_vertex: HashMap<(u32, u32), (Real, Vector)>,
}
impl BodyContacts {
}
/// The step-constant inputs of the per-body contact assembly.
pub(super) struct AssemblyCtx<'a> {
    pub(super) island_id: usize,
    /// The first group's substep parameters (the constraints' softness).
    pub(super) params: &'a IntegrationParameters,
    pub(super) narrow_phase: &'a NarrowPhase,
    pub(super) colliders: &'a ColliderSet,
    pub(super) bodies: &'a RigidBodySet,
    pub(super) soft_bodies: &'a SoftBodySet,
    /// Dynamic and static contact softness `(erp_inv_dt, cfm_factor)`.
    pub(super) dyn_soft: (Real, Real),
    pub(super) static_soft: (Real, Real),
}
