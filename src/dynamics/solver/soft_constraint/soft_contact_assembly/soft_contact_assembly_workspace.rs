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

/// The collision mesh a soft body's collider holds.
pub(super) fn mesh_of(sb: &SoftBody, collider: ColliderHandle) -> Option<&SoftCollisionMesh> {
    sb.mesh_of(collider)
}

/// The mesh a deformable collider holds.
pub(super) fn mesh_ref(co: &Collider) -> SoftMeshRef {
    co.deformable_mesh_ref.expect("not a deformable collider")
}

/// Workspace of the contact assembly kept across steps.
#[derive(Default)]
pub(crate) struct EdgeContactWorkspace {
    /// Awake index of every awake soft body, by handle.
    pub(super) awake_of: HashMap<SoftBodyHandle, usize>,
    /// The constraints and new edge contacts of every awake body (assembled in parallel, appended in
    /// awake order).
    pub(super) per_body: Vec<BodyContacts>,
}

/// `SoftContactSource::manifold` of the constraints without a manifold point: an edge-vs-edge constraint (its
/// warm start lives in the owner's `edge_contacts`) or a vertex-vs-surface constraint (`vertex_contacts`).
pub(super) const SOURCE_EDGE_CONTACT: u32 = u32::MAX;
pub(super) const SOURCE_VERTEX_CONTACT: u32 = u32::MAX - 1;
/// `SoftContactSource::manifold` of a soft-rigid pair's predictive vertex constraint (its warm start
/// lives in the pair's `SoftRigidContacts::vertices`).
pub(super) const SOURCE_RIGID_VERTEX: u32 = u32::MAX - 2;
/// The corrective pace deep recovery is allowed (demoted intruders, the volume constraints):
/// fast enough to visibly heal, slow enough that a correction cannot pump the material.
pub(super) fn material_pace(params: &IntegrationParameters) -> Real {
    params.soft_bodies.recovery.recovery_pace * params.length_unit
}

/// The contact constraints of one awake soft body, and its new edge-vs-edge and vertex-vs-surface
/// contacts (the constraints' `point` indices are positions in those lists).
#[derive(Default)]
pub(super) struct BodyContacts {
    pub(super) contacts: Vec<SoftContact>,
    /// The contact state of every mesh assembled this step, in the order the body's meshes were
    /// visited (a constraint's `point` indexes its own mesh's lists).
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
    /// Elements (and vertices) of the mesh being assembled whose self contacts stand down
    /// this step: backed by inverted cells, or part of a self-crossing of the surface (empty
    /// while the surface is healthy). Copied from the narrow phase's self detection.
    pub(super) tangled_elements: Vec<bool>,
    pub(super) tangled_vertices: Vec<bool>,
    /// The crossing element pairs the narrow phase's sweep found on the mesh being assembled.
    pub(super) crossings: Vec<(u32, u32)>,
    /// Elements of the current pair's surface side, and vertices of its vertex side, in a crossing
    /// between the two surfaces (empty while they do not cross); constraints touching them may
    /// only expel, never hold at skin distance, so nothing freezes the crossing.
    pub(super) cross_tangled_elements: Vec<bool>,
    pub(super) cross_tangled_vertices: Vec<bool>,
    /// Element-granularity crossing flags on the vertex-side mesh, and the recorded
    /// (element-side, vertex-side) crossing element pairs.
    pub(super) cross_tangled_vb_elements: Vec<bool>,
    pub(super) cross_pairs: Vec<(u32, u32)>,
    /// The intersection-volume constraints this body assembled (see `SoftOverlapConstraint`).
    pub(super) overlap_constraints: Vec<OverlapConstraintWorkspace>,
    /// Per grid cell, the center and normal (element side into vertex side) guiding the current
    /// pair's crossing repulsion (see `crossing_repulsion_guide`); for a self pair, the push
    /// direction of the fold's vertices, the facing surface's vertices taking the opposite.
    pub(super) repel_guides: Vec<(Vector, Vector)>,
    /// For a self pair, whether each vertex lies in its own body's self-intersection region
    /// (see `classify_inside_self`).
    pub(super) repel_inside: Vec<bool>,
    /// For the current pair, whether each vertex of the vertex side lies in the other
    /// side's volume patch (see `overlap_patch_constraints`).
    pub(super) patch_inside_vb: Vec<bool>,
    /// The rigid pairs of the current mesh with a volume constraint (see
    /// `overlap_patch_constraints`): the collider, the surface vertices' depths in its patch
    /// (`NEG_INFINITY` outside), and the constraint's cells (center, normal).
    pub(super) rigid_patches: Vec<(ColliderHandle, Vec<Real>, Vec<(Vector, Vector)>)>,
}

/// An intersection-volume constraint assembled by one body (see `SoftOverlapConstraint`): the
/// gradient triples of every simulated particle of the pair, the rigid side if any, and the
/// right-hand side (the signed volume estimate when the correction runs on it, else the slack).
pub(super) struct OverlapConstraintWorkspace {
    pub(super) grads: Vec<(u32, Real, Vector, Vector)>,
    /// The `(side, particle)` of every gradient entry (see `SoftOverlapConstraint::warm`).
    pub(super) particles: Vec<(u8, u32)>,
    pub(super) rigid: Option<(u32, Vector, AngVector)>,
    pub(super) rigid_pose0: (Vector, Rotation),
    pub(super) rhs: Real,
    /// A contact in its own right (see `SoftOverlapConstraint::hard`).
    pub(super) hard: bool,
    /// The warm impulses of a hard constraint (see `SoftOverlapWarm`): the other collider, the
    /// fitted multiplier, the warm impulse per gradient entry, and the rigid side's.
    pub(super) warm: Option<(ColliderHandle, Real)>,
    pub(super) warm_impulses: Vec<Vector>,
    pub(super) warm_rigid: (Vector, AngVector),
    pub(super) max_bias_velocity: Real,
    /// The pair's contact slot of the constraint's bin (see `SoftOverlapConstraint::report`).
    pub(super) report: (ColliderHandle, ColliderHandle, u32),
}

/// The contact state one collision mesh owns across steps: what the next step warm-starts from.
#[derive(Default)]
pub(super) struct MeshContacts {
    pub(super) id: SoftMeshId,
    pub(super) edge_contacts: Vec<SoftEdgeContact>,
    pub(super) vertex_contacts: Vec<SoftVertexContact>,
    /// The pairs' overlap progress states (the owner's `overlap_states` of the next step).
    pub(super) overlap_states: Vec<SoftOverlapState>,
    /// The volume constraints assembled this step (the owner's `volume_contacts`).
    pub(super) volume_contacts: Vec<SoftVolumeContact>,
    /// Surface travel since the last self-crossing sweep (the owner's `crossing_sweep_travel` of the
    /// next step).
    pub(super) crossing_sweep_travel: Real,
    /// The surfaces whose pair with this mesh crosses this step (the owner's
    /// `crossed_partners`).
    pub(super) crossed_partners: Vec<crate::geometry::ColliderHandle>,
}

impl BodyContacts {
    /// Starts assembling the mesh `id`: its contacts go to a list of its own.
    pub(super) fn begin_mesh(&mut self, id: SoftMeshId) {
        self.current_mesh = self.meshes.len();
        self.rigid_patches.clear();
        self.meshes.push(MeshContacts {
            id,
            ..Default::default()
        });
    }

    /// The contact state of the mesh being assembled.
    pub(super) fn mesh(&mut self) -> &mut MeshContacts {
        &mut self.meshes[self.current_mesh]
    }

    /// Records the self pair as recovery-owned when this mesh has self tangles.
    pub(super) fn exempt_self_tangles(&mut self, own_surface: crate::geometry::ColliderHandle) {
        let mc = &mut self.meshes[self.current_mesh];
        if (!self.tangled_elements.is_empty() || !self.tangled_vertices.is_empty())
            && !mc.crossed_partners.contains(&own_surface)
        {
            mc.crossed_partners.push(own_surface);
        }
    }

    /// Records the current pair as recovery-owned when the two boundaries cross.
    pub(super) fn exempt_cross_tangles(&mut self, other_surface: crate::geometry::ColliderHandle) {
        if self.cross_tangled_elements.is_empty() {
            return;
        }
        let mc = &mut self.meshes[self.current_mesh];
        if !mc.crossed_partners.contains(&other_surface) {
            mc.crossed_partners.push(other_surface);
        }
    }

    pub(super) fn clear(&mut self) {
        self.contacts.clear();
        self.meshes.clear();
        self.current_mesh = 0;
        self.max_approach_speed = 0.0;
        self.previous.clear();
        self.previous_vertex.clear();
        self.tangled_elements.clear();
        self.tangled_vertices.clear();
        self.crossings.clear();
        self.cross_tangled_elements.clear();
        self.cross_tangled_vertices.clear();
        self.cross_tangled_vb_elements.clear();
        self.cross_pairs.clear();
        self.overlap_constraints.clear();
    }
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
