//! Contact constraints between a soft-body surface element (`DIM` particles, barycentric weights)
//! and a solver body or another surface element (surface-vs-surface and self contacts). Same model
//! as the rigid kernel: speculation, softness bias, Coulomb friction, warm start, total impulses.

use crate::dynamics::IntegrationParameters;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::geometry::ColliderHandle;
use crate::math::{AngVector, AngularInertia, DIM, Real, Vector};
use crate::utils::{AngularInertiaOps, ComponentMul, CrossProduct, DotProduct, OrthonormalBasis};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Where a soft contact writes its impulses back: a narrow-phase manifold point, or (`manifold`
/// set to an assembly sentinel) an entry of the owner body's edge-vs-edge or vertex-vs-surface
/// list, plus the pair's contact `slot` reporting the impulse (`u32::MAX` for a self contact).
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftContactSource {
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle,
    pub manifold: u32,
    pub point: u32,
    pub slot: u32,
}

/// The other side of a soft contact when it is a surface element too: its particles' solver
/// slots (`u32::MAX`: frozen at `frozen_pos`), barycentric weights, inverse masses.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftContactElement {
    pub particles: [u32; CONTACT_ANCHORS],
    pub weights: [Real; CONTACT_ANCHORS],
    pub im_particles: [Real; CONTACT_ANCHORS],
    pub frozen_pos: [Vector; CONTACT_ANCHORS],
}

/// Number of particles the soft side of a contact is spread over: a surface element's, or a
/// cell's when the body collides through a skin the cells carry.
pub(crate) const CONTACT_ANCHORS: usize = DIM + 1;

/// One contact point between a soft surface element and a solver body.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftContact {
    pub source: SoftContactSource,
    /// Index of the surface's soft body in the solver's awake list, and the element's particle
    /// indices in it (the vertex support rows write their warm start back to the particle).
    pub support_body: u32,
    pub support_particle: [u32; CONTACT_ANCHORS],
    /// Solver slots of the particles carrying the contact point (`u32::MAX`: frozen at
    /// `frozen_pos`, or an unused anchor when its weight is zero).
    pub particles: [u32; CONTACT_ANCHORS],
    pub weights: [Real; CONTACT_ANCHORS],
    pub im_particles: [Real; CONTACT_ANCHORS],
    pub frozen_pos: [Vector; CONTACT_ANCHORS],
    /// The other side: solver slot (`u32::MAX`: world-attached at `body_point0`, or a surface
    /// element when `element` is set).
    pub body: u32,
    pub element: Option<SoftContactElement>,
    /// Index of the other side's soft body in the solver's awake list when it is a particle
    /// or an element of one (`u32::MAX`: a rigid body or the world), for the chunking of the
    /// constraints by soft-body pair.
    pub other_body: u32,
    pub body_im: Vector,
    pub body_ii: AngularInertia,
    /// The other side's contact point in its CoM-local frame, and frozen world lever arm.
    pub body_local_point: Vector,
    pub body_arm: Vector,
    /// World contact points at build time (surface side and other side).
    pub surface_point0: Vector,
    pub body_point0: Vector,
    /// Force direction on the surface (minus the contact normal from surface to body).
    pub dir: Vector,
    pub tangents: [Vector; DIM - 1],
    /// Separation at build time (skins baked in), and its live value.
    pub dist0: Real,
    pub friction: Real,
    /// The other side is a soft-body particle: the row is solved in the biased pass only, like
    /// the springs driving both sides.
    pub soft_other: bool,
    /// Static or dynamic contact softness.
    pub erp_inv_dt: Real,
    pub cfm_factor: Real,
    /// Cap of the penetration-recovery velocity (`Real::MAX`: the world's
    /// `max_corrective_velocity`); a demoted expulsion holds its intruder quietly.
    pub max_bias: Real,
    // Solve state.
    pub torque_dir: AngVector,
    pub ii_torque_dir: AngVector,
    pub r_normal: Real,
    pub rhs_normal: Real,
    pub cfm_normal: Real,
    pub impulse_normal: Real,
    pub impulse_normal_acc: Real,
    pub torque_tangent: [AngVector; DIM - 1],
    pub ii_torque_tangent: [AngVector; DIM - 1],
    pub r_tangent: [Real; DIM - 1],
    pub rhs_tangent: [Real; DIM - 1],
    pub impulse_tangent: [Real; DIM - 1],
    pub impulse_tangent_acc: [Real; DIM - 1],
}

impl SoftContact {
    /// Current world position of the surface point (barycentric blend of the particles).
    #[inline]
    fn surface_point(&self, bodies: &SolverBodies) -> Vector {
        let mut p = Vector::ZERO;
        for k in 0..CONTACT_ANCHORS {
            let id = self.particles[k];
            let x = if id == u32::MAX {
                self.frozen_pos[k]
            } else {
                bodies.get_pose(id).translation
            };
            p += x * self.weights[k];
        }
        p
    }

    /// Current world position of the other side's contact point.
    #[inline]
    fn body_point(&self, bodies: &SolverBodies) -> Vector {
        if let Some(e) = &self.element {
            let mut p = Vector::ZERO;
            for k in 0..CONTACT_ANCHORS {
                let id = e.particles[k];
                let x = if id == u32::MAX {
                    e.frozen_pos[k]
                } else {
                    bodies.get_pose(id).translation
                };
                p += x * e.weights[k];
            }
            p
        } else if self.body == u32::MAX {
            self.body_point0
        } else {
            bodies.get_pose(self.body).pose() * self.body_local_point
        }
    }

    /// Updates the separation, rhs and effective masses from the current solver state.
    /// `wo_bias`: relax pass (speculative term only, no softness, no tangent drift bias).
    pub fn update(&mut self, bodies: &SolverBodies, params: &IntegrationParameters, wo_bias: bool) {
        let inv_dt = params.inv_dt();
        let ps = self.surface_point(bodies);
        let po = self.body_point(bodies);
        let delta = (ps - self.surface_point0) - (po - self.body_point0);
        let dist = self.dist0 + delta.gdot(self.dir);

        // Effective masses (the other side's mass properties are step-constant, read once the
        // solver bodies are filled: this runs after the body-init stage).
        if self.body != u32::MAX {
            let pose = bodies.get_pose(self.body);
            self.body_im = pose.im;
            self.body_ii = pose.ii;
        }
        // A side acting through pinned particles but for a sliver of weight on a free one is a
        // lever (the free particle would need a huge impulse): it is frozen for the constraint.
        fn freeze_levers<const N: usize>(weights: &[Real; N], im: &mut [Real; N]) {
            let mut w2 = 0.0;
            let mut im_max: Real = 0.0;
            for k in 0..N {
                w2 += weights[k] * weights[k] * im[k];
                im_max = im_max.max(im[k]);
            }
            if im_max > 0.0 && w2 < 0.04 * im_max {
                *im = [0.0; N];
            }
        }
        freeze_levers(&self.weights, &mut self.im_particles);
        if let Some(e) = &mut self.element {
            freeze_levers(&e.weights, &mut e.im_particles);
        }
        let mut w_particles = 0.0;
        for k in 0..CONTACT_ANCHORS {
            w_particles += self.weights[k] * self.weights[k] * self.im_particles[k];
        }
        if let Some(e) = &self.element {
            for k in 0..CONTACT_ANCHORS {
                w_particles += e.weights[k] * e.weights[k] * e.im_particles[k];
            }
        }
        self.torque_dir = self.body_arm.gcross(-self.dir);
        self.ii_torque_dir = self.body_ii.transform_vector(self.torque_dir);
        self.r_normal = crate::utils::inv(
            w_particles
                + self.dir.gdot(self.body_im.component_mul(&self.dir))
                + self.ii_torque_dir.gdot(self.torque_dir),
        );
        for j in 0..DIM - 1 {
            let t = self.tangents[j];
            self.torque_tangent[j] = self.body_arm.gcross(-t);
            self.ii_torque_tangent[j] = self.body_ii.transform_vector(self.torque_tangent[j]);
            self.r_tangent[j] = crate::utils::inv(
                w_particles
                    + t.gdot(self.body_im.component_mul(&t))
                    + self.ii_torque_tangent[j].gdot(self.torque_tangent[j]),
            );
        }

        let rhs_wo_bias = dist.max(0.0) * inv_dt;
        if wo_bias {
            self.rhs_normal = rhs_wo_bias;
            self.cfm_normal = 1.0;
            for j in 0..DIM - 1 {
                self.rhs_tangent[j] = 0.0;
            }
        } else {
            let max_corrective_velocity = params.max_corrective_velocity().min(self.max_bias);
            let bias = (dist * self.erp_inv_dt).clamp(-max_corrective_velocity, 0.0);
            self.rhs_normal = rhs_wo_bias + bias;
            self.cfm_normal = if dist <= 0.0 { self.cfm_factor } else { 1.0 };
            for j in 0..DIM - 1 {
                self.rhs_tangent[j] = delta.gdot(self.tangents[j]) * inv_dt;
            }
            // Bank the previous substep's impulses (they were applied in full) before the
            // warm-start scaling of the new substep.
            self.impulse_normal_acc += self.impulse_normal;
            self.impulse_normal *= params.warmstart_coefficient;
            for j in 0..DIM - 1 {
                self.impulse_tangent_acc[j] += self.impulse_tangent[j];
                self.impulse_tangent[j] *= params.warmstart_coefficient;
            }
        }
    }

    /// Which sides belong to a FEM body: `(surface side, other side)`.
    #[inline]
    fn apply(&self, bodies: &mut SolverBodies, dir: Vector, ii_torque: AngVector, lambda: Real) {
        for k in 0..CONTACT_ANCHORS {
            let id = self.particles[k];
            if id != u32::MAX && (id as usize) < bodies.len() {
                bodies.vels[id as usize].linear +=
                    dir * (self.weights[k] * self.im_particles[k] * lambda);
            for k in 0..CONTACT_ANCHORS {
                let id = e.particles[k];
                if id != u32::MAX && (id as usize) < bodies.len() {
                    bodies.vels[id as usize].linear -=
                        dir * (e.weights[k] * e.im_particles[k] * lambda);
                }
            }
        } else if self.body != u32::MAX && (self.body as usize) < bodies.len() {
    }

    /// Relative velocity of the surface point w.r.t. the other side's contact point, dotted
    /// with `dir` (its torque term given).
    #[inline]
    fn relative_velocity(&self, bodies: &SolverBodies, dir: Vector, torque: AngVector) -> Real {
        let mut vs = Vector::ZERO;
        for k in 0..CONTACT_ANCHORS {
            let id = self.particles[k];
            if id != u32::MAX {
                vs += bodies.get_vel(id).linear * self.weights[k];
            }
        }
        let mut dvel = dir.gdot(vs);
        if let Some(e) = &self.element {
            let mut vo = Vector::ZERO;
            for k in 0..CONTACT_ANCHORS {
                let id = e.particles[k];
                if id != u32::MAX {
                    vo += bodies.get_vel(id).linear * e.weights[k];
                }
            }
            dvel -= dir.gdot(vo);
        } else if self.body != u32::MAX {
            let vo = bodies.get_vel(self.body);
            dvel += -dir.gdot(vo.linear) + torque.gdot(vo.angular);
        }
        dvel
    }

    /// Applies the accumulated (warm-start) impulses.
    pub fn warmstart(&self, bodies: &mut SolverBodies) {
        self.apply(bodies, self.dir, self.ii_torque_dir, self.impulse_normal);
        for j in 0..DIM - 1 {
            self.apply(
                bodies,
                self.tangents[j],
                self.ii_torque_tangent[j],
                self.impulse_tangent[j],
            );
        }
    }

    /// One Gauss-Seidel iteration: normal part then Coulomb friction.
    pub fn solve(&mut self, bodies: &mut SolverBodies) {
        // Normal.
        let dvel = self.relative_velocity(bodies, self.dir, self.torque_dir) + self.rhs_normal;
        let new_impulse = (self.cfm_normal * (self.impulse_normal - self.r_normal * dvel)).max(0.0);
        let delta = new_impulse - self.impulse_normal;
        self.impulse_normal = new_impulse;
        self.apply(bodies, self.dir, self.ii_torque_dir, delta);

        // Friction: per-tangent update, then a clamp to the disc of radius μλ.
        let limit = self.friction * self.impulse_normal;
        let mut new_tangent = self.impulse_tangent;
        for j in 0..DIM - 1 {
            let dvel = self.relative_velocity(bodies, self.tangents[j], self.torque_tangent[j])
                + self.rhs_tangent[j];
            new_tangent[j] -= self.r_tangent[j] * dvel;
        }
        #[cfg(feature = "dim2")]
        {
            new_tangent[0] = new_tangent[0].clamp(-limit, limit);
        }
        #[cfg(feature = "dim3")]
        {
            let norm = (new_tangent[0] * new_tangent[0] + new_tangent[1] * new_tangent[1]).sqrt();
            if norm > limit {
                let scale = if norm > 0.0 { limit / norm } else { 0.0 };
                new_tangent[0] *= scale;
                new_tangent[1] *= scale;
            }
        }
        for j in 0..DIM - 1 {
            let delta = new_tangent[j] - self.impulse_tangent[j];
            self.impulse_tangent[j] = new_tangent[j];
            self.apply(bodies, self.tangents[j], self.ii_torque_tangent[j], delta);
        }
    }

    /// The tangent basis of a force direction.
    pub fn tangent_basis(dir: Vector) -> [Vector; DIM - 1] {
        dir.orthonormal_basis()
    }
}
