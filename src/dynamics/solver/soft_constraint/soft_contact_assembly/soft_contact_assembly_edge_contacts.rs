//! Edge-vs-edge contact constraints between two soft surfaces (or a surface and itself).

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};


use super::{BodyContacts, SOURCE_EDGE_CONTACT, mesh_of, mesh_ref};
use super::super::soft_constraints_set::SoftConstraintsSet;
use super::super::soft_contact::CONTACT_ANCHORS;
use super::super::soft_contact::{SoftContact, SoftContactElement, SoftContactSource};
use crate::dynamics::soft_body::{SoftCollisionMesh, SoftEdgeContact};
use crate::dynamics::{IntegrationParameters, SoftBody};
use crate::geometry::soft_contacts::SoftEdgePass;
use crate::geometry::{Collider, ColliderHandle};
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;

impl SoftConstraintsSet {
    /// Edge-vs-edge constraints between the surfaces of awake body `ai` and `other` (itself for
    /// self contacts; `other_ai` is its awake index, `None` when fully pinned and frozen), from the
    /// candidates `detected` (`None` without an edge pass); complements the vertex-vs-surface pass.
    #[allow(clippy::too_many_arguments)]
    #[cfg_attr(feature = "dim2", allow(unused_variables))]
    pub(super) fn assemble_edge_contacts(
        &self,
        params: &IntegrationParameters,
        ai: usize,
        other_ai: Option<usize>,
        other: &SoftBody,
        (surface_handle, surface_co): (ColliderHandle, &Collider),
        (other_surface_handle, other_co): (ColliderHandle, &Collider),
        (erp_inv_dt, cfm_factor): (Real, Real),
        detected: Option<&SoftEdgePass>,
        out: &mut BodyContacts,
    ) {
        let awake = &self.awake[ai];
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let (Some(mesh), Some(other_mesh)) = (
            mesh_of(sb, surface_handle),
            mesh_of(other, other_surface_handle),
        ) else {
            return;
        };
        let is_self = other_ai == Some(ai);
        // In 2D vertex constraints alone complete a first contact between closed surfaces, and
        // crossing edges at dented corners would only add oblique constraints fighting them; in 3D
        // edge-leading bodies (bars crossing corner-first) need them, filtered below.
        #[cfg(feature = "dim2")]
        if mesh.is_closed() && other_mesh.is_closed() {
            return;
        }
        #[cfg(feature = "dim3")]
        if !params.soft_bodies.recovery.edge_speculation && mesh.is_closed() && other_mesh.is_closed() {
            return;
        }
        // While a volume constraint owns a crossed closed-closed pair, its edge constraints stand down
        // entirely: inside the overlap they are wrong-sided and freeze it (two overlapping
        // balloons never de-overlapped through them).
        #[cfg(feature = "dim3")]
        if !is_self && mesh.is_closed() && other_mesh.is_closed() {
            let crossed = out
                .meshes
                .get(out.current_mesh)
                .is_some_and(|m| m.crossed_partners.contains(&other_surface_handle));
            let recovery = &params.soft_bodies.recovery;
            if recovery.overlap_constraints && recovery.overlap_edge_stand_down && crossed {
                return;
            }
        }
        let Some(detected) = detected else {
            return;
        };
        let other_handle = mesh_ref(other_co);
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let other_slots = other_ai.map(|bi| {
            let a = &self.awake[bi];
            &self.slots[a.slot_start..a.slot_start + a.num_particles]
        });
        let friction = crate::dynamics::CoefficientCombineRule::combine(
            surface_co.friction(),
            other_co.friction(),
            surface_co.friction_combine_rule(),
            other_co.friction_combine_rule(),
        );

        let current_mesh = out.current_mesh;
        let workspace = out;
        workspace.previous.clear();
        for c in &mesh.edge_contacts {
            if c.other == other_handle {
                workspace.previous.insert(
                    (c.other, c.edge, c.other_edge),
                    (c.impulse, c.tangent_impulse),
                );
            }
        }
        if detected.crossed {
            // The pair is recovery-owned: the crossing guard leaves it alone.
            let mc = &mut workspace.meshes[current_mesh];
            if !mc.crossed_partners.contains(&other_surface_handle) {
                mc.crossed_partners.push(other_surface_handle);
            }
        }

        let inv_mass_of = |body: &SoftBody, slots: Option<&[u32]>, v: u32| -> (u32, Real) {
            match slots {
                Some(slots) if slots[v as usize] != u32::MAX => {
                    (slots[v as usize], body.particles[v as usize].inv_mass)
                }
                _ => (u32::MAX, 0.0),
            }
        };

        // The pass is oriented from its owner (see `soft_contacts`); consumed from the other
        // side (an owner that is not simulated), its candidates are mirrored.
        let mirrored = detected.own != surface_handle;
        for (ci, c) in detected.candidates.iter().enumerate() {
            // A candidate the contact-modification hook disabled gets no constraint.
            if !c.enabled {
                continue;
            }
            // The pair's contact slot of this candidate (none for a self contact).
            let slot = if detected.slot_offset == u32::MAX {
                u32::MAX
            } else {
                detected.slot_offset + ci as u32
            };
            let (ea, eb, ba, bb, point_a, point_b, dir) = if mirrored {
                (
                    c.other_edge,
                    c.edge,
                    c.other_bcoords,
                    c.bcoords,
                    c.other_point,
                    c.point,
                    -c.dir,
                )
            } else {
                (c.edge, c.other_edge, c.bcoords, c.other_bcoords, c.point, c.other_point, c.dir)
            };
            let dist = c.dist;
            let (va, vb) = (mesh.edge_vertices(ea), other_mesh.edge_vertices(eb));
            {
                {
                    let tangents = SoftContact::tangent_basis(dir);
                    let (warm_normal, warm_tangent_world) = workspace
                        .previous
                        .get(&(other_handle, ea, eb))
                        .copied()
                        .unwrap_or((0.0, Vector::ZERO));
                    let warm_tangent: [Real; DIM - 1] =
                        core::array::from_fn(|k| warm_tangent_world.gdot(tangents[k]));

                    // Each side's contact point acts through its particles: the
                    // edge's own two vertices, or the cell holding them under a skin.
                    let side = |body: &SoftBody,
                                body_mesh: &SoftCollisionMesh,
                                body_slots: Option<&[u32]>,
                                vertices: [u32; 2],
                                bcoords: [Real; 2],
                                point: Vector| {
                        let (anchors, weights, _) =
                            body_mesh.contact_anchors(body, &vertices, &bcoords, point);
                        let particles: [u32; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                            if anchors[k] == u32::MAX {
                                u32::MAX
                            } else {
                                inv_mass_of(body, body_slots, anchors[k]).0
                            }
                        });
                        let im_particles: [Real; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                            if anchors[k] == u32::MAX {
                                0.0
                            } else {
                                inv_mass_of(body, body_slots, anchors[k]).1
                            }
                        });
                        let frozen_pos: [Vector; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                            if anchors[k] == u32::MAX {
                                Vector::ZERO
                            } else {
                                body.particles[anchors[k] as usize].position
                            }
                        });
                        (anchors, particles, weights, im_particles, frozen_pos)
                    };
                    let (support_particle, particles, weights, im_particles, frozen_pos) =
                        side(sb, mesh, Some(slots), va, ba, point_a);
                    let (_, particles2, weights2, im_particles2, frozen_pos2) =
                        side(other, other_mesh, other_slots, vb, bb, point_b);
                    let assembled = &mut workspace.meshes[current_mesh];
                    let point = assembled.edge_contacts.len() as u32;
                    assembled.edge_contacts.push(SoftEdgeContact {
                        other: other_handle,
                        edge: ea,
                        other_edge: eb,
                        impulse: warm_normal,
                        tangent_impulse: warm_tangent_world,
                    });
                    let contact = SoftContact {
                        source: SoftContactSource {
                            collider1: surface_handle,
                            collider2: other_surface_handle,
                            manifold: SOURCE_EDGE_CONTACT,
                            point,
                            slot,
                        },
                        support_body: ai as u32,
                        support_particle,
                        particles,
                        weights,
                        im_particles,
                        frozen_pos,
                        body: u32::MAX,
                        element: Some(SoftContactElement {
                            particles: particles2,
                            weights: weights2,
                            im_particles: im_particles2,
                            frozen_pos: frozen_pos2,
                        }),
                        other_body: other_ai.map_or(u32::MAX, |bi| bi as u32),
                        body_im: Vector::ZERO,
                        body_ii: Default::default(),
                        body_local_point: point_b,
                        body_arm: Vector::ZERO,
                        surface_point0: point_a,
                        body_point0: point_b,
                        dir,
                        tangents,
                        dist0: dist,
                        friction,
                        soft_other: true,
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
                    };
                    workspace.contacts.push(contact);
                }
            }
        }
    }
}
