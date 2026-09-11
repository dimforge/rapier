//! Moving a soft body's particles from outside: pinning, attachments to rigid bodies, user forces and impulses.
use super::SoftBody;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::math::{Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// A particle of a soft body attached to a rigid body (see [`SoftBody::attach_particle`]): a
/// point-to-point constraint between the particle and a point of the body.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftParticleAttachment {
    /// The attached particle.
    pub particle: u32,
    /// The rigid body it is attached to.
    pub body: RigidBodyHandle,
    /// The attachment point in the rigid body's local frame.
    pub local_anchor: Vector,
    /// Accumulated impulse of the last step (warm-start state), one per world axis.
    pub(crate) impulse: Vector,
}

impl SoftParticleAttachment {
    /// The impulse this attachment applied during the last substep (world axes, positive when
    /// pulling the particle back toward the anchor).
    pub fn impulse(&self) -> Vector {
        self.impulse
    }
}

impl SoftBody {
    /// Pins (`pinned = true`) or releases (`pinned = false`) the `i`-th particle: a pinned one
    /// is kinematic, holding its position or following [`Self::set_particle_kinematic_target`] or
    /// [`Self::set_particle_velocity`]; release restores its nominal mass and keeps its velocity.
    pub fn set_particle_pinned(&mut self, i: usize, pinned: bool) {
        let particle = &mut self.particles[i];
        if pinned {
            // Held where it is until given a target or a velocity.
            particle.inv_mass = 0.0;
            particle.velocity = Vector::ZERO;
            particle.next_position = Some(particle.position);
        } else {
            particle.inv_mass = crate::utils::inv(particle.mass);
            particle.next_position = None;
        }
        self.modified = true;
    }

    /// Attaches the `i`-th particle to a rigid body by a two-way point-to-point constraint (unlike
    /// pinning), undone by [`Self::detach_particle`]; the anchor is its current position in
    /// `body`'s local frame, and a twice-attached particle keeps both attachments.
    pub fn attach_particle(&mut self, i: usize, body: RigidBodyHandle, bodies: &RigidBodySet) {
        let particle = &self.particles[i];
        let local_anchor = bodies
            .get(body)
            .map(|rb| rb.position().inverse_transform_point(particle.position))
            .unwrap_or(particle.position);
        self.attachments.push(SoftParticleAttachment {
            particle: i as u32,
            body,
            local_anchor,
            impulse: Vector::ZERO,
        });
        self.attachments_modified = true;
        self.modified = true;
    }

    /// Detaches the `i`-th particle from every rigid body it was attached to with
    /// [`Self::attach_particle`]. Returns whether it was attached at all.
    pub fn detach_particle(&mut self, i: usize) -> bool {
        let before = self.attachments.len();
        self.attachments.retain(|a| a.particle != i as u32);
        let detached = self.attachments.len() != before;
        if detached {
            self.attachments_modified = true;
            self.modified = true;
        }
        detached
    }

    /// The particles of this soft body attached to rigid bodies.
    pub fn particle_attachments(&self) -> &[SoftParticleAttachment] {
        &self.attachments
    }

    /// Adds `force` to the user force of every particle (persistent until
    /// [`Self::reset_forces`], like a rigid body's forces). Pinned particles ignore it.
    pub fn add_force(&mut self, force: Vector, wake_up: bool) {
        for p in &mut self.particles {
            p.force += force;
        }
        if wake_up {
            self.wake_up();
        }
    }

    /// Adds `force` to the user force of the `i`-th particle (persistent until
    /// [`Self::reset_forces`]).
    pub fn add_particle_force(&mut self, i: usize, force: Vector, wake_up: bool) {
        self.particles[i].force += force;
        if wake_up {
            self.wake_up();
        }
    }

    /// Clears the user forces of every particle.
    pub fn reset_forces(&mut self, wake_up: bool) {
        for p in &mut self.particles {
            p.force = Vector::ZERO;
        }
        if wake_up {
            self.wake_up();
        }
    }

    /// Applies a linear velocity change (`impulse` per unit of mass) to every free particle of
    /// this soft body: the whole body is kicked at the same velocity.
    pub fn apply_impulse(&mut self, impulse: Vector, wake_up: bool) {
        for p in &mut self.particles {
            if p.inv_mass > 0.0 {
                p.velocity += impulse;
            }
        }
        if wake_up {
            self.wake_up();
        }
    }

    /// Applies an impulse to the `i`-th particle (its velocity changes by `impulse / mass`;
    /// pinned particles ignore it).
    pub fn apply_particle_impulse(&mut self, i: usize, impulse: Vector, wake_up: bool) {
        let p = &mut self.particles[i];
        p.velocity += impulse * p.inv_mass;
        if wake_up {
            self.wake_up();
        }
    }

    /// Applies `impulse` to every free particle within `falloff_radius` of world point `point`,
    /// scaled linearly to zero at that radius and divided by the particle's mass; a
    /// `falloff_radius` of zero or less gives every particle the whole impulse.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector,
        point: Vector,
        falloff_radius: Real,
        wake_up: bool,
    ) {
        for p in &mut self.particles {
            if p.inv_mass == 0.0 {
                continue;
            }
            let scale = if falloff_radius > 0.0 {
                1.0 - (p.position - point).length() / falloff_radius
            } else {
                1.0
            };
            if scale > 0.0 {
                p.velocity += impulse * (scale * p.inv_mass);
            }
        }
        if wake_up {
            self.wake_up();
        }
    }

    /// Applies an impulse of `magnitude` pointing away from world point `center` to every free
    /// particle within `falloff_radius`, scaled linearly to zero at that radius (a particle on the
    /// center gets nothing); a `falloff_radius` of zero or less pushes every particle fully.
    pub fn apply_radial_impulse(
        &mut self,
        center: Vector,
        magnitude: Real,
        falloff_radius: Real,
        wake_up: bool,
    ) {
        for p in &mut self.particles {
            if p.inv_mass == 0.0 {
                continue;
            }
            let offset = p.position - center;
            let distance = offset.length();
            if distance <= Real::EPSILON {
                continue;
            }
            let scale = if falloff_radius > 0.0 {
                1.0 - distance / falloff_radius
            } else {
                1.0
            };
            if scale > 0.0 {
                p.velocity += offset * (magnitude * scale * p.inv_mass / distance);
            }
        }
        if wake_up {
            self.wake_up();
        }
    }
}
