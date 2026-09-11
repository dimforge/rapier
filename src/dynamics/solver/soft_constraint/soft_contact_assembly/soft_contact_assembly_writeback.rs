//! Write-back of the soft contact constraints' impulses to their narrow-phase manifold points and volume warm starts.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;

use super::soft_contact_assembly_workspace::SOURCE_RIGID_VERTEX;
use super::{SOURCE_EDGE_CONTACT, SOURCE_VERTEX_CONTACT};
use super::super::soft_constraints_set::SoftConstraintsSet;
use super::super::soft_contact::SoftContact;
use crate::dynamics::soft_body::SoftOverlapWarm;
use crate::geometry::NarrowPhase;
use crate::math::{AngVector, DIM, Vector};

/// Adds a constraint's impulses to its manifold point (the point's impulses are cleared
/// before its pair's constraints report; a point split into endpoint constraints sums them):
/// the step's total for the events, the last substep's for the warm start.
fn report_impulses(data: &mut crate::geometry::ContactData, c: &SoftContact) {
    #[cfg(feature = "dim2")]
    {
        data.tangent_impulse[0] = crate::utils::canonicalize_zero(
            data.tangent_impulse[0] + c.impulse_tangent_acc[0] + c.impulse_tangent[0],
        );
        data.warmstart_tangent_impulse[0] = crate::utils::canonicalize_zero(
            data.warmstart_tangent_impulse[0] + c.impulse_tangent[0],
        );
    }
    #[cfg(feature = "dim3")]
    {
        for j in 0..2 {
            data.tangent_impulse[j] = crate::utils::canonicalize_zero(
                data.tangent_impulse[j] + c.impulse_tangent_acc[j] + c.impulse_tangent[j],
            );
            data.warmstart_tangent_impulse[j] = crate::utils::canonicalize_zero(
                data.warmstart_tangent_impulse[j] + c.impulse_tangent[j],
            );
        }
        data.warmstart_tangent_world = crate::utils::canonicalize_zero(
                + c.tangents[0] * c.impulse_tangent[0]
                + c.tangents[1] * c.impulse_tangent[1],
        );
    }
}

impl SoftConstraintsSet {
    /// Writes the soft contacts' impulses back to their narrow-phase manifold points (total
    /// impulse for the contact events, warm-start impulses for the next step).
    pub fn writeback_contacts(&self, narrow_phase: &mut NarrowPhase) {
        // The volume contacts' warm impulses (see `SoftOverlapWarm`): each entry's last-substep
        // normal and friction impulse, accumulated per pair; a bin's contact slot reports the
        // impulse as the force put on the own side (the multiplier along the own gradients).
        for constraint in &self.overlap_constraints {
            let (c1, c2, slot) = constraint.report;
            if slot != u32::MAX {
                let mut own_grad = Vector::ZERO;
                for (&(_, _, g, _), &(side, _)) in self.overlap_grads[constraint.grads.clone()]
                    .iter()
                    .zip(&self.overlap_particles[constraint.grads.clone()])
                {
                    if side == 0 {
                        own_grad += g;
                    }
                }
                if let Some(soft) = narrow_phase
                    .contact_pair_mut(c1, c2)
                    .and_then(|pair| pair.soft_mut())
                {
                    soft.report_impulse(
                        slot,
                        crate::utils::canonicalize_zero(constraint.impulse * own_grad.length()),
                        Vector::ZERO,
                    );
                }
            }
            let Some((ai, mesh_id, other)) = constraint.warm else {
                continue;
            };
            // SAFETY: the writeback stage is serial and owns the soft bodies.
            let ptr = self.awake[ai as usize].ptr;
            let sb = unsafe { &mut *ptr };
            let Some(mesh) = sb.mesh_mut(mesh_id) else {
                continue;
            };
            let idx = match mesh.overlap_warm.iter().position(|w| w.other == other) {
                Some(i) => i,
                None => {
                    mesh.overlap_warm.push(SoftOverlapWarm {
                        other,
                        own: Vec::new(),
                        other_soft: Vec::new(),
                        rigid: (Vector::ZERO, AngVector::default()),
                    });
                    mesh.overlap_warm.len() - 1
                }
            };
            let record = &mut mesh.overlap_warm[idx];
            for (&(_, _, g, _), &(side, p)) in self.overlap_grads[constraint.grads.clone()]
                .iter()
                .zip(&self.overlap_particles[constraint.grads.clone()])
            {
                let impulse = -g * constraint.impulse;
                let list = if side == 0 {
                    &mut record.own
                } else {
                    &mut record.other_soft
                };
                match list.iter_mut().find(|e| e.0 == p) {
                    Some(e) => e.1 += impulse,
                    None => list.push((p, impulse)),
                }
            }
            if let Some((_, g_lin, g_ang)) = constraint.rigid {
                record.rigid.0 += -g_lin * constraint.impulse;
                record.rigid.1 += -g_ang * constraint.impulse;
            }
        }
        // The constraints of a pair are contiguous: on entering a new pair, clear the reported impulses
        // of all its points first (points without a constraint this step, e.g. deduplicated vertex
        // contacts, must not keep reporting a stale force).
        let mut prev_pair = None;
        for c in &self.contacts {
            {
                // The load borne by the surface (its extra substeps under a heavy pile).
                // SAFETY: the writeback stage is serial and owns the soft bodies.
                let ptr = self.awake[c.support_body as usize].ptr;
                let sb = unsafe { &mut *ptr };
                sb.contact_load += (c.impulse_normal_acc + c.impulse_normal).abs();
            }
            if c.source.manifold == SOURCE_EDGE_CONTACT
                || c.source.manifold == SOURCE_VERTEX_CONTACT
            {
                // An edge-vs-edge or vertex-vs-surface constraint: its warm start lives on the owner
                // soft body.
                // SAFETY: the writeback stage is serial and owns the soft bodies.
                let ptr = self.awake[c.support_body as usize].ptr;
                let sb = unsafe { &mut *ptr };
                // The constraint's first collider is the mesh that owns its warm start.
                let Some(mesh) = sb.mesh_of_mut(c.source.collider1) else {
                    continue;
                };
                let normal = crate::utils::canonicalize_zero(c.impulse_normal);
                let mut tangent = Vector::ZERO;
                for k in 0..DIM - 1 {
                    tangent += c.tangents[k] * c.impulse_tangent[k];
                }
                let tangent = crate::utils::canonicalize_zero(tangent);
                if c.source.manifold == SOURCE_EDGE_CONTACT {
                    if let Some(edge) = mesh.edge_contacts.get_mut(c.source.point as usize) {
                        edge.impulse = normal;
                        edge.tangent_impulse = tangent;
                    }
                } else if let Some(vertex) = mesh.vertex_contacts.get_mut(c.source.point as usize) {
                    vertex.impulse = normal;
                    vertex.tangent_impulse = tangent;
                }
                // The pair's contact slot of the candidate (rebuilt from scratch by every
                // narrow-phase update, so nothing stale to clear) reports the impulse.
                if c.source.slot != u32::MAX {
                    if let Some(soft) = narrow_phase
                        .contact_pair_mut(c.source.collider1, c.source.collider2)
                        .and_then(|pair| pair.soft_mut())
                    {
                        soft.report_impulse(
                            c.source.slot,
                            crate::utils::canonicalize_zero(
                                c.impulse_normal_acc + c.impulse_normal,
                            ),
                            crate::utils::canonicalize_zero(
                                c.impulse_tangent_acc
                                    .iter()
                                    .zip(&c.impulse_tangent)
                                    .enumerate()
                                    .map(|(k, (a, i))| c.tangents[k] * (a + i))
                                    .sum::<Vector>(),
                            ),
                        );
                    }
                }
                continue;
            }
            let Some(rigid) = narrow_phase
                .contact_pair_mut(c.source.collider1, c.source.collider2)
                .and_then(|pair| pair.rigid_mut())
            else {
                continue;
            };
            let key = (c.source.collider1, c.source.collider2);
            if prev_pair != Some(key) {
                prev_pair = Some(key);
                for manifold in &mut rigid.manifolds {
                    for point in &mut manifold.points {
                        point.data.impulse = 0.0;
                        point.data.tangent_impulse = Default::default();
                    }
                }
                if let Some(soft) = rigid.soft.as_deref_mut() {
                    for vertex in &mut soft.vertices {
                        vertex.impulse = 0.0;
                        vertex.tangent_impulse = Vector::ZERO;
                    }
                }
            }
            if c.source.manifold == SOURCE_RIGID_VERTEX {
                // A predictive vertex constraint: its warm start lives on the pair's vertex contact.
                if let Some(vertex) = rigid
                    .soft
                    .as_deref_mut()
                    .and_then(|soft| soft.vertices.get_mut(c.source.point as usize))
                {
                    let mut tangent = Vector::ZERO;
                    for k in 0..DIM - 1 {
                        tangent += c.tangents[k] * c.impulse_tangent[k];
                    }
                    vertex.impulse = crate::utils::canonicalize_zero(c.impulse_normal);
                    vertex.tangent_impulse = crate::utils::canonicalize_zero(tangent);
                }
                continue;
            }
            let Some(manifold) = rigid.manifolds.get_mut(c.source.manifold as usize) else {
                continue;
            };
            let Some(point) = manifold.points.get_mut(c.source.point as usize) else {
                continue;
            };
            report_impulses(&mut point.data, c);
        }
    }
}
