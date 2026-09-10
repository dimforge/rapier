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
    guide: Option<Vector>,
) -> Option<(Vector, Real)> {
    let (p0, n) = element_plane(eb_mesh, element, |v| eb_mesh.vertex(eb, v))?;
    // Guided (see `crossing_repulsion_guide`): the vertex is pushed along the pair's volume
    // normal, its separation measured along it from the element's plane point.
    if let Some(push) = guide {
        return Some((-push, (vertex_pos - p0).dot(push) - skins));
    }
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
        // Intersection-volume contact (`SoftOverlapConstraint`, Allard et al. 2010): one coupled
        // constraint per crossed closed pair over both intruding patches (none if self-tangled),
        // built once per pair (lower surface handle, or this body if the other is not simulated).
        let recovery = &params.soft_bodies.recovery;
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
                && (other_ai.is_none()
                    || surface_handle.into_raw_parts() < other_surface_handle.into_raw_parts())
        }) {
            // The patches are binned from the pair's lower surface; consumed from the other
            // side (an owner that is not simulated), their bins are mirrored.
            let mirrored = (volume.own != surface_handle).then(|| mirrored_bins(&volume.bins));
            let bins = mirrored.as_deref().unwrap_or(&volume.bins);
            emit_volume_bins(
                params,
                out,
                mesh,
                other_surface_handle,
                (eb, eb_slots),
                Some((vb, vb_slots)),
                None,
                hard,
                bins,
                volume.slot_offset,
            );
        }
        // Self-overlaps between distinct regions of one closed surface (see
        // `overlap_self_regions`, detected by the narrow phase): each region gets a coupled
        // constraint with the vertices of the elements it faces, like a pair of bodies would.
        if is_self {
            for bins in regions {
                emit_volume_bins(
                    params,
                    out,
                    mesh,
                    surface_handle,
                    (vb, vb_slots),
                    Some((vb, vb_slots)),
                    None,
                    hard,
                    bins,
                    u32::MAX,
                );
            }
        }
        // The two contact skins (a soft surface's is its vertices' thickness).
        let skins = vb_co.contact_skin() + eb_co.contact_skin();

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
        let patch_policy = params.soft_bodies.recovery.overlap_patch_constraints;
        out.repel_guides.clone_from(&detected.repel_guides);
        out.patch_inside_vb.clone_from(&detected.patch_inside_vb);
        out.repel_inside.clone_from(&detected.repel_inside);
        // The surface vertices with an element within reach (see `soft_contacts`).
        for hit in &detected.hits {
            let v = hit.vertex as usize;
            let candidates = detected.candidates_of(hit);
            // A vertex inside the other side's volume patch (see `overlap_patch_constraints`): its
            // constraints disagreeing with the constraint are stood down or bent along its normal.
            let in_patch = out.patch_inside_vb.get(v).copied().unwrap_or(false)
                && patch_policy != SoftPatchConstraints::Keep;
            let vertex_pos = vb_mesh.vertex(vb, v);
            // The vertex side: its own solver body when the body collides through its cells'
            // boundary, the particles of the cell holding it when it collides through a skin.
            let (vertex_anchors, vertex_weights) = vb_mesh.vertex_anchors(vb, v);
            let vertex_element = vb_mesh.is_skinned().then(|| {
                let particles: [u32; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                    if vertex_anchors[k] == u32::MAX {
                        u32::MAX
                    } else {
                        vb_slots
                            .map(|s| s[vertex_anchors[k] as usize])
                            .unwrap_or(u32::MAX)
                    }
                });
                SoftContactElement {
                    particles,
                    weights: vertex_weights,
                    im_particles: core::array::from_fn(|k| {
                        if particles[k] == u32::MAX {
                            0.0
                        } else {
                            vb.particles[vertex_anchors[k] as usize].inv_mass
                        }
                    }),
                    frozen_pos: core::array::from_fn(|k| {
                        if vertex_anchors[k] == u32::MAX {
                            Vector::ZERO
                        } else {
                            vb.particles[vertex_anchors[k] as usize].position
                        }
                    }),
                }
            });
            let vslot = match &vertex_element {
                Some(_) => u32::MAX,
                None => vb_slots.map(|s| s[v]).unwrap_or(u32::MAX),
            };
            // Whether the vertex side can move at all: its slot, or any of its cell's particles.
            let vertex_free = match &vertex_element {
                Some(e) => e.im_particles.iter().any(|im| *im > 0.0),
                None => vslot != u32::MAX,
            };
            // A frozen surface against a vertex that is not simulated: no DOF on either side.
            if self_frozen && !vertex_free {
                continue;
            }

            // A foreign vertex that crossed a closed surface (see `soft_contacts`) hands its
            // pair to the recovery (the pair is crossed).
            if hit.crossed {
                let mc = &mut out.meshes[current_mesh];
                if !mc.crossed_partners.contains(&other_surface_handle) {
                    mc.crossed_partners.push(other_surface_handle);
                }
            }
            // The vertex's solver body (its contact point is tracked in that body's CoM frame).
            // The vertex's solver body sits at the particle: the contact point is the origin of
            // its frame.
            let (body_local_point, body_arm) = (
                if vslot != u32::MAX {
                    Vector::ZERO
                } else {
                    vertex_pos
                },
                Vector::ZERO,
            );
            let (erp_inv_dt, cfm_factor) = if !vertex_free {
                (static_erp, static_cfm)
            } else {
                (dyn_erp, dyn_cfm)
            };

            for (ci, c) in candidates.iter().enumerate() {
                // A candidate the contact-modification hook disabled gets no constraint.
                if !c.enabled {
                    continue;
                }
                // The pair's contact slot of this candidate (none for a self contact).
                let slot = if detected.slot_offset == u32::MAX {
                    u32::MAX
                } else {
                    detected.slot_offset + hit.candidates.start + ci as u32
                };
                // Crossing repulsion (see `repel_constraint`): a flagged feature's constraint repels toward
                // the vertex's neighborhood side instead of standing down or being dropped,
                // and drops when the neighborhood straddles the element.
                let flagged = if is_self {
                    out.tangled_vertices.get(v).copied().unwrap_or(false)
                        || out.tangled_elements.get(c.element as usize).copied().unwrap_or(false)
                } else {
                    out.cross_tangled_vertices.get(v).copied().unwrap_or(false)
                        || out
                            .cross_tangled_elements
                            .get(c.element as usize)
                            .copied()
                            .unwrap_or(false)
                };
                // The nearest guiding cell's normal (the vertex's push direction), if any.
                let patch_guide = if in_patch {
                    out.repel_guides
                        .iter()
                        .min_by(|a, b| {
                            let da = (a.0 - vertex_pos).length_squared();
                            let db = (b.0 - vertex_pos).length_squared();
                            da.partial_cmp(&db).unwrap_or(core::cmp::Ordering::Equal)
                        })
                        .map(|g| g.1)
                        // Disagreeing: the constraint pushes the vertex (along minus its `dir`)
                        // against the normal.
                        .filter(|g| c.dir.gdot(*g) > 0.0)
                } else {
                    None
                };
                if patch_guide.is_some() && patch_policy == SoftPatchConstraints::StandDown {
                    continue;
                }
                let along_patch_normal =
                    patch_guide.is_some() && patch_policy == SoftPatchConstraints::AlongNormal;
                let repelled = if repel && flagged || along_patch_normal {
                    // The nearest guiding cell's normal, if the guide is on.
                    let guide = out
                        .repel_guides
                        .iter()
                        .min_by(|a, b| {
                            let da = (a.0 - vertex_pos).length_squared();
                            let db = (b.0 - vertex_pos).length_squared();
                            da.partial_cmp(&db).unwrap_or(core::cmp::Ordering::Equal)
                        })
                        .map(|g| {
                            // A self pair: the fold's vertices go one way, the facing
                            // surface's the other.
                            if is_self && !out.repel_inside.get(v).copied().unwrap_or(false) {
                                -g.1
                            } else {
                                g.1
                            }
                        });
                    if guide.is_none() && !(repel && flagged) {
                        // A patch vertex with no cell to follow keeps its constraint as is.
                        None
                    } else {
                        match repel_constraint(
                            (eb, eb_mesh, c.element as usize),
                            (
                                vb,
                                vb_mesh,
                                vertex_pos,
                                &vb_mesh.ring[vb_mesh.ring_offsets[v] as usize
                                    ..vb_mesh.ring_offsets[v + 1] as usize],
                            ),
                            skins,
                            guide,
                        ) {
                            Some((d, dd)) => Some((d, dd, true)),
                            None => continue,
                        }
                    }
                } else {
                    None
                };
                // Reversed interior contacts of a closed surface are dropped: holding them
                // would pin the intruder inside; the volume constraints and the elasticity
                // resolve it.
                let (dir, dist, expelling) = match c.outward {
                    _ if repelled.is_some() => repelled.unwrap(),
                    Some(outward) if c.interior && c.dir.gdot(outward) > 0.5 => continue,
                    _ => (c.dir, c.dist, false),
                };
                // A constraint touching a boundary crossing between the surfaces may only expel,
                // never hold: a keep-apart constraint there is wrong-sided and freezes it.
                if !expelling
                    && !is_self
                    && params.soft_bodies.recovery.cross_body_expel_gate
                    && (out.cross_tangled_vertices.get(v).copied().unwrap_or(false)
                        || out
                            .cross_tangled_elements
                            .get(c.element as usize)
                            .copied()
                            .unwrap_or(false))
                {
                    continue;
                }
                let element = eb_mesh.element(c.element as usize);
                let mut surface_point0 = Vector::ZERO;
                for (k, v) in element.iter().enumerate() {
                    surface_point0 += eb_mesh.vertex(eb, *v as usize) * c.weights[k];
                }
                let (anchors, anchor_weights, tracked) =
                    eb_mesh.contact_anchors(eb, element, &c.weights, surface_point0);
                let surface_point0 = tracked;
                let particles: [u32; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                    if anchors[k] == u32::MAX {
                        u32::MAX
                    } else {
                        eb_slots.map(|s| s[anchors[k] as usize]).unwrap_or(u32::MAX)
                    }
                });
                let im_particles: [Real; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                    if particles[k] == u32::MAX {
                        0.0
                    } else {
                        eb.particles[anchors[k] as usize].inv_mass
                    }
                });
                let frozen_pos: [Vector; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                    if anchors[k] == u32::MAX {
                        Vector::ZERO
                    } else {
                        eb.particles[anchors[k] as usize].position
                    }
                });
                let tangents = SoftContact::tangent_basis(dir);
                let (warm_normal, warm_tangent_world) = out
                    .previous_vertex
                    .get(&(v as u32, c.element))
                    .copied()
                    .unwrap_or((0.0, Vector::ZERO));
                let warm_tangent: [Real; DIM - 1] =
                    core::array::from_fn(|k| warm_tangent_world.gdot(tangents[k]));
                let assembled = &mut out.meshes[current_mesh];
                let point = assembled.vertex_contacts.len() as u32;
                assembled.vertex_contacts.push(SoftVertexContact {
                    other: other_handle,
                    vertex: v as u32,
                    element: c.element,
                    flipped,
                    impulse: warm_normal,
                    tangent_impulse: warm_tangent_world,
                });
                out.contacts.push(SoftContact {
                    source: SoftContactSource {
                        collider1: surface_handle,
                        collider2: other_surface_handle,
                        manifold: SOURCE_VERTEX_CONTACT,
                        point,
                        slot,
                    },
                    support_body: ai as u32,
                    support_particle: anchors,
                    particles,
                    weights: anchor_weights,
                    im_particles,
                    frozen_pos,
                    body: vslot,
                    other_body: other_ai.map_or(u32::MAX, |bi| bi as u32),
                    element: vertex_element,
                    body_im: Vector::ZERO,
                    body_ii: Default::default(),
                    body_local_point,
                    body_arm,
                    surface_point0,
                    body_point0: vertex_pos,
                    dir,
                    tangents,
                    dist0: dist,
                    friction,
                    soft_other: true,
                    fem: [None, None],
                    erp_inv_dt,
                    cfm_factor,
                    max_bias: Real::MAX,
                    torque_dir: Default::default(),
                    ii_torque_dir: Default::default(),
                    r_normal: 0.0,
                    rhs_normal: 0.0,
                    cfm_normal: 1.0,
                    impulse_normal: warm_normal,
                    impulse_normal_acc: -warm_normal,
                    torque_tangent: [Default::default(); DIM - 1],
                    ii_torque_tangent: [Default::default(); DIM - 1],
                    r_tangent: [0.0; DIM - 1],
                    rhs_tangent: [0.0; DIM - 1],
                    impulse_tangent: warm_tangent,
                    impulse_tangent_acc: core::array::from_fn(|k| -warm_tangent[k]),
                });
            }
        }
    }
}
