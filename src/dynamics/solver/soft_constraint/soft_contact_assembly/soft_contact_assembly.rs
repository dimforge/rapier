//! The `assemble_contacts` entry point: runs the per-body assembly and appends the constraints to the substep groups.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::soft_constraints_set::{SoftConstraintsSet, SoftOverlapConstraint};
use super::super::soft_contact::SoftContact;
use super::{AssemblyCtx, BodyContacts};
use crate::dynamics::{IntegrationParameters, RigidBodySet, SoftBodySet, SpringCoefficients};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::math::{Real, Vector};

impl SoftConstraintsSet {
    /// Builds the contact constraints of the awake soft bodies' surface colliders from the
    /// narrow-phase pairs and the vertex-vs-surface and edge-vs-edge candidates. Group-major: runs
    /// after [`Self::assemble`], appending to each group's `contacts` range (`group_dt`: softness).
    #[allow(clippy::too_many_arguments)]
    pub fn assemble_contacts(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        narrow_phase: &NarrowPhase,
        colliders: &ColliderSet,
        bodies: &RigidBodySet,
        soft_bodies: &SoftBodySet,
        group_of_slot: impl Fn(u32) -> usize,
        group_dt: impl Fn(usize) -> Real,
    ) {
        if self.awake.is_empty() {
            return;
        }
        // The contact softness is normalized by the constraint's effective mass, i.e. by a few particles'
        // mass here, while the load comes from the whole soft body through them: at the rigid
        // defaults the constraints would sink under a body's weight, so they run stiffer.
        let stiffen = |c: &SpringCoefficients<Real>| SpringCoefficients {
            natural_frequency: c.natural_frequency * params.soft_bodies.contact_stiffening,
            damping_ratio: c.damping_ratio,
        };
        let dyn_soft = stiffen(&params.contact_softness);
        let static_soft = stiffen(&params.static_contact_softness);
        let ctx = AssemblyCtx {
            island_id,
            params,
            narrow_phase,
            colliders,
            bodies,
            soft_bodies,
            dyn_soft: (
                dyn_soft.erp_inv_dt(params.dt),
                dyn_soft.cfm_factor(params.dt),
            ),
            static_soft: (
                static_soft.erp_inv_dt(params.dt),
                static_soft.cfm_factor(params.dt),
            ),
        };
        let mut per_body = core::mem::take(&mut self.edge_workspace.per_body);
        per_body.resize_with(self.awake.len(), Default::default);
        {
            let workspace = &mut self.edge_workspace;
            workspace.awake_of.clear();
            for (ai, awake) in self.awake.iter().enumerate() {
                workspace.awake_of.insert(awake.handle, ai);
            }
        }

        // Every awake body's constraints only read the shared sets: assembled in parallel, appended in
        // awake order (deterministic).
        let this = &*self;
        let assemble_body = |(ai, out): (usize, &mut BodyContacts)| {
            out.clear();
            this.assemble_body_contacts(ai, &ctx, out);
        };
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            per_body.par_iter_mut().enumerate().for_each(assemble_body);
        }
        #[cfg(not(feature = "parallel"))]
        per_body.iter_mut().enumerate().for_each(assemble_body);

        for (ai, body) in per_body.iter().enumerate() {
            // A frozen body has no particle to substep for; its request would only land on a
            // pinned particle's kinematic body.
            self.awake[ai].contact_approach_speed = (!body.contacts.is_empty()
                && !self.awake[ai].frozen)
                .then_some(body.max_approach_speed);
        }
        // A frozen body's constraint is solved with the group of the body it holds (its own particles
        // are not solver DOFs): the other side's slot, or the other element's first live particle.
        let constraint_group = |c: &SoftContact, own: u16| -> usize {
            if c.body != u32::MAX {
                return group_of_slot(c.body);
            }
            let live = c
                .element
                .as_ref()
                .and_then(|e| e.particles.iter().copied().find(|&s| s != u32::MAX));
            live.map(&group_of_slot).unwrap_or(own as usize)
        };
        for gi in 0..self.groups.len() {
            let start = self.contacts.len();
            for ai in self.groups[gi].awake.clone() {
                if !self.awake[ai].frozen {
                    self.contacts.append(&mut per_body[ai].contacts);
                }
            }
            for (ai, awake) in self.awake.iter().enumerate() {
                if awake.frozen {
                    self.contacts.extend(
                        per_body[ai]
                            .contacts
                            .iter()
                            .filter(|c| constraint_group(c, awake.group) == gi),
                    );
                }
            }
            // The softness of a constraint follows its group's substep length (the constraints were built
            // with the first group's).
            let dyn_soft = (
                dyn_soft.erp_inv_dt(group_dt(gi)),
                dyn_soft.cfm_factor(group_dt(gi)),
            );
            let static_soft = (
                static_soft.erp_inv_dt(group_dt(gi)),
                static_soft.cfm_factor(group_dt(gi)),
            );
            for c in &mut self.contacts[start..] {
                let world_fixed = c.element.is_none() && c.body == u32::MAX;
                (c.erp_inv_dt, c.cfm_factor) = if world_fixed { static_soft } else { dyn_soft };
            }
            self.groups[gi].contacts = start..self.contacts.len();
            // The intersection-volume constraints of the group's bodies, with the contact softness.
            let ostart = self.overlap_constraints.len();
            for ai in self.groups[gi].awake.clone() {
                let mesh_id = per_body[ai]
                    .meshes
                    .get(per_body[ai].current_mesh)
                    .map(|m| m.id);
                for constraint in per_body[ai].overlap_constraints.drain(..) {
                    let gs = self.overlap_grads.len();
                    let n = constraint.grads.len();
                    self.overlap_grads.extend(constraint.grads);
                    self.overlap_particles.extend(constraint.particles);
                    if constraint.warm_impulses.len() == n {
                        self.overlap_warm_impulses.extend(constraint.warm_impulses);
                    } else {
                        self.overlap_warm_impulses
                            .extend(core::iter::repeat_n(Vector::ZERO, n));
                    }
                    let (impulse, warm) = match (constraint.warm, mesh_id) {
                        (Some((other, impulse)), Some(mesh_id)) => {
                            (impulse, Some((ai as u32, mesh_id, other)))
                        }
                        _ => (0.0, None),
                    };
                    self.overlap_constraints.push(SoftOverlapConstraint {
                        grads: gs..self.overlap_grads.len(),
                        rhs: constraint.rhs,
                        rhs0: constraint.rhs,
                        rigid_pose0: constraint.rigid_pose0,
                        warm,
                        warm_pending: warm.is_some(),
                        warm_rigid: constraint.warm_rigid,
                        erp_inv_dt: stiffen(&params.contact_softness).erp_inv_dt(group_dt(gi)),
                        cfm_coeff: stiffen(&params.contact_softness).cfm_coeff(group_dt(gi)),
                        max_bias_velocity: constraint.max_bias_velocity,
                        impulse,
                        // A hard constraint's slack is speculative: consumed within the substep,
                        // never past it.
                        speculative_inv_dt: if constraint.hard {
                            1.0 / group_dt(gi)
                        } else {
                            0.0
                        },
                        rigid: constraint.rigid,
                        hard: constraint.hard,
                        fem_sides: 0..0,
                        report: constraint.report,
                    });
                }
            }
            self.groups[gi].overlap_constraints = ostart..self.overlap_constraints.len();
        }
        // Hand the new edge and vertex contacts to their owner bodies.
        // SAFETY: the assembly holds no other reference into the soft bodies any more.
        for (ai, awake) in self.awake.iter().enumerate() {
            let sb = unsafe { &mut *awake.ptr };
            for assembled in &mut per_body[ai].meshes {
                let Some(mesh) = sb.mesh_mut(assembled.id) else {
                    continue;
                };
                core::mem::swap(&mut mesh.edge_contacts, &mut assembled.edge_contacts);
                core::mem::swap(&mut mesh.vertex_contacts, &mut assembled.vertex_contacts);
                core::mem::swap(&mut mesh.overlap_states, &mut assembled.overlap_states);
                core::mem::swap(&mut mesh.volume_contacts, &mut assembled.volume_contacts);
                mesh.overlap_warm.clear();
                mesh.crossing_sweep_travel = assembled.crossing_sweep_travel;
                core::mem::swap(&mut mesh.crossed_partners, &mut assembled.crossed_partners);
            }
        }
        self.edge_workspace.per_body = per_body;
        self.chunk_contacts();
    }
}
