//! The `assemble_contacts` entry point: runs the per-body assembly and appends the constraints to the substep groups.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::{AssemblyCtx, BodyContacts};
use super::super::soft_constraints_set::{SoftConstraintsSet, SoftOverlapConstraint};
use super::super::soft_contact::SoftContact;
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
        self.edge_workspace.per_body = per_body;
        self.chunk_contacts();
    }
}
