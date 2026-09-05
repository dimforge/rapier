//! Vertex-vs-surface contact constraints (self contacts and surface-vs-surface pairs) and the crossing-repulsion constraint.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;

use super::soft_contact_assembly_volume_constraints::{emit_volume_bins, mirrored_bins};
use super::{AssemblyCtx, BodyContacts, SOURCE_VERTEX_CONTACT, mesh_of, mesh_ref};
use super::super::soft_constraints_set::SoftConstraintsSet;
use super::super::soft_contact::CONTACT_ANCHORS;
use super::super::soft_contact::{SoftContact, SoftContactElement, SoftContactSource};
use crate::dynamics::soft_body::SoftPatchConstraints;
use crate::dynamics::soft_body::{SoftCollisionMesh, SoftVertexContact};
use crate::dynamics::SoftBody;
use crate::geometry::soft_contacts::{SoftVertexPass, SoftVolumePatch, VolumeBin, element_plane};
use crate::geometry::{Collider, ColliderHandle};
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;

/// The crossing-repulsion constraint of a vertex against an element: pushes along the element's
/// plane normal, signed toward the vertex's one-ring centroid, to skin distance on that side
/// (inactive once there). `None` when the one-ring straddles the plane or the element is a wire.
fn repel_constraint(
    (eb, eb_mesh, element): (&SoftBody, &SoftCollisionMesh, usize),
    (vb, vb_mesh, vertex_pos, neighbors): (&SoftBody, &SoftCollisionMesh, Vector, &[u32]),
    skins: Real,
) -> Option<(Vector, Real)> {
    let (p0, n) = element_plane(eb_mesh, element, |v| eb_mesh.vertex(eb, v))?;
    if neighbors.is_empty() {
        return None;
    }
    let mut centroid = Vector::ZERO;
    for &u in neighbors {
        centroid += vb_mesh.vertex(vb, u as usize);
    }
    centroid /= neighbors.len() as Real;
    let side = (centroid - p0).dot(n);
    if side.abs() < 0.25 * skins {
        return None;
    }
    // `dir` is the force on the surface; the vertex is pushed along its opposite.
    let push = n * side.signum();
    Some((-push, (vertex_pos - p0).dot(push) - skins))
}

impl SoftConstraintsSet {
    /// Vertex-vs-surface contact constraints: the surface vertices of `other` (`other_ai` its awake
    /// index, `None` when fully pinned; itself for self contacts) against awake body `ai`, one per
    /// (vertex, element) in `detected` (`flipped`: this body's vertices against a non-awake other).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn assemble_vertex_contacts(
        &self,
        ctx: &AssemblyCtx,
        ai: usize,
        other_ai: Option<usize>,
        other: &SoftBody,
        (surface_handle, surface_co): (ColliderHandle, &Collider),
        (other_surface_handle, other_co): (ColliderHandle, &Collider),
        flipped: bool,
        detected: &SoftVertexPass,
        volume: Option<&SoftVolumePatch>,
        regions: &[Vec<VolumeBin>],
        out: &mut BodyContacts,
    ) {
        let AssemblyCtx {
            params,
            dyn_soft: (dyn_erp, dyn_cfm),
            static_soft: (static_erp, static_cfm),
            ..
        } = *ctx;
        let awake = &self.awake[ai];
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let other_handle = mesh_ref(other_co);
        let current_mesh = out.current_mesh;
        let is_self = other_ai == Some(ai);
        let self_frozen = awake.frozen;
        let own_slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let other_slots: Option<&[u32]> = other_ai.map(|bi| {
            let a = &self.awake[bi];
            &self.slots[a.slot_start..a.slot_start + a.num_particles]
        });
        // The meshes the two sides collide through.
        let (Some(mesh), Some(other_mesh)) = (
            mesh_of(sb, surface_handle),
            mesh_of(other, other_surface_handle),
        ) else {
            return;
        };
        // The vertex side and the element side.
        let (vb, vb_mesh, vb_slots, vb_co, eb, eb_mesh, eb_slots, eb_co) = if flipped {
            (sb, mesh, Some(own_slots), surface_co, other, other_mesh, other_slots, other_co)
        } else {
            (other, other_mesh, other_slots, other_co, sb, mesh, Some(own_slots), surface_co)
        };
        // Crossings between the two surfaces (see `soft_contacts`): the pair's keep-apart
        // constraints are wrong-sided there and would freeze the crossing (or push material through
        // the gates the self stand-down opened), so they stand down around it like self ones.
        out.cross_tangled_elements
            .clone_from(&detected.cross_tangled_elements);
        out.cross_tangled_vertices
            .clone_from(&detected.cross_tangled_vertices);
        out.cross_tangled_vb_elements
            .clone_from(&detected.cross_tangled_vb_elements);
        out.cross_pairs.clone_from(&detected.cross_pairs);
        if !is_self && !flipped {
            out.exempt_cross_tangles(other_surface_handle);
        }
        let friction = crate::dynamics::CoefficientCombineRule::combine(
            surface_co.friction(),
            other_co.friction(),
            surface_co.friction_combine_rule(),
            other_co.friction_combine_rule(),
        );
        let hard = recovery.overlap_skin_volume
            || recovery.overlap_patch_constraints != SoftPatchConstraints::Keep;
        if let Some(volume) = volume.filter(|_| {
            !flipped
                && (!recovery.overlap_skip_self_tangled || out.tangled_elements.is_empty())
        out.previous_vertex.clear();
        for c in &mesh.vertex_contacts {
            if c.other == other_handle && c.flipped == flipped {
                out.previous_vertex
                    .insert((c.vertex, c.element), (c.impulse, c.tangent_impulse));
            }
        }
        // Crossing repulsion (see `repel_constraint`): each crossing pair also gets constraints of
        // its own (piercing element's vertices vs the pierced element), so an edge-first crossing
        // with no vertex within reach is repelled too. `targets[f]` lists the elements `f` crosses.
        let repel = params.soft_bodies.recovery.crossing_repulsion
            && !vb_mesh.is_wire()
            && !eb_mesh.is_wire()
            && if is_self {
                !out.tangled_vertices.is_empty() || !out.tangled_elements.is_empty()
            } else {
                !out.cross_tangled_vertices.is_empty() || !out.cross_tangled_elements.is_empty()
            };
        // Narrow-phase results: the guiding volume normals (see `crossing_repulsion_guide`), the
        // vertices inside the other side's volume patch (see `overlap_patch_constraints`) and, for
        // a self pair, the vertices in their own body's self-intersection region.
            // boundary, the particles of the cell carrying it when it collides through a skin.
                // A candidate the contact-modification hook disabled gets no constraint.
                // Crossing repulsion (see `repel_constraint`): a flagged feature's constraint repels toward
                // would pin the intruder inside; the volume rows and the elasticity
                // A row touching a boundary crossing between the two surfaces may only
                // expel, never hold: a keep-apart row there is wrong-sided by construction
        }
    }
}
