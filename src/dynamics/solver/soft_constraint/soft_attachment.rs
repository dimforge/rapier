//! Point-to-point constraints attaching a soft-body particle to a rigid body
//! (`SoftBody::attach_particle`): `DIM` coupled bilateral rows keeping the particle on a point of
//! the body, solved like a locked joint (both passes, no bias in relax, softness, warm start).

use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{AngVector, AngularInertia, Matrix, Real, Vector};
use crate::utils::{AngularInertiaOps, ComponentMul, CrossProduct};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// One particle attachment, as solved by the staged solver.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftAttachmentConstraint {
    /// Index of the soft body in the solver's awake list, and of the attachment in it.
    pub soft_body: u32,
    pub attachment: u32,
    /// The particle's solver slot and inverse mass.
    pub particle: u32,
    pub im_particle: Real,
    /// The rigid body's solver slot (`u32::MAX`: fixed or not simulated, the anchor stays at
    /// `anchor0`) and its attachment point in its CoM-local frame.
    pub body: u32,
    pub body_local_point: Vector,
    pub anchor0: Vector,
    /// Joint softness.
    pub erp_inv_dt: Real,
    pub cfm_coeff: Real,
    // Solve state.
    pub body_im: Vector,
    pub body_ii: AngularInertia,
    /// World lever arm of the anchor about the body's center of mass.
    pub arm: Vector,
    /// The rows' effective-mass matrix and its (softened) inverse.
    pub lhs: Matrix,
    pub inv_lhs: Matrix,
    pub rhs: Vector,
    pub impulse: Vector,
}

/// The matrix `-[r]× ii [r]×` (3D) or `ii perp(r) perp(r)ᵀ` (2D): the velocity change of the
/// point at lever arm `r` per unit impulse applied there, through the body's rotation.
fn rotational_effective_mass(arm: Vector, ii: AngularInertia) -> Matrix {
    #[cfg(feature = "dim2")]
    {
        let p = Vector::new(-arm.y, arm.x);
        Matrix::from_cols(p * (p.x * ii), p * (p.y * ii))
    }
    #[cfg(feature = "dim3")]
    {
        let rx = Matrix::from_cols(
            Vector::new(0.0, arm.z, -arm.y),
            Vector::new(-arm.z, 0.0, arm.x),
            Vector::new(arm.y, -arm.x, 0.0),
        );
        -(rx * ii.into_matrix() * rx)
    }
}

impl SoftAttachmentConstraint {
    /// Current world position of the body's attachment point.
    #[inline]
    fn anchor(&self, bodies: &SolverBodies) -> Vector {
        if self.body == u32::MAX {
            self.anchor0
        } else {
            bodies.get_pose(self.body).pose() * self.body_local_point
        }
    }

    /// Updates the position error, effective mass and bias from the current solver state.
    /// `wo_bias`: relax pass (no position correction).
    pub fn update(&mut self, bodies: &SolverBodies, wo_bias: bool) {
        let particle = bodies.get_pose(self.particle).translation;
        let anchor = self.anchor(bodies);
        let mut lhs = Matrix::IDENTITY * self.im_particle;
        if self.body != u32::MAX {
            let pose = bodies.get_pose(self.body);
            self.body_im = pose.im;
            self.body_ii = pose.ii;
            self.arm = anchor - pose.translation;
            lhs += Matrix::from_diagonal(pose.im) + rotational_effective_mass(self.arm, pose.ii);
        }
        self.lhs = lhs;
        let softened = lhs * (1.0 + self.cfm_coeff);
        // No degree of freedom on either side (a pinned particle on a fixed body): inert constraint.
        self.inv_lhs = if softened.determinant().abs() > Real::EPSILON * Real::EPSILON {
            softened.inverse()
        } else {
            Matrix::ZERO
        };
        // Uncapped, like a joint's locked axes: the attachment must win over the contact constraints
        // solved after it (a cloth corner attached to a bar its surface also touches).
        self.rhs = if wo_bias {
            Vector::ZERO
        } else {
            (particle - anchor) * self.erp_inv_dt
        };
    }

    /// Applies `impulse` (positive along the constraint: the particle is pulled back, the body pulled
    /// toward it).
    #[inline]
    fn apply(&self, bodies: &mut SolverBodies, impulse: Vector) {
        if (self.particle as usize) < bodies.len() {
            bodies.vels[self.particle as usize].linear -= impulse * self.im_particle;
        }
        if self.body != u32::MAX && (self.body as usize) < bodies.len() {
            let v = &mut bodies.vels[self.body as usize];
            v.linear += self.body_im.component_mul(&impulse);
            let torque: AngVector = self.arm.gcross(impulse);
            v.angular += self.body_ii.transform_vector(torque);
        }
    }

    #[inline]
    pub fn warmstart(&mut self, bodies: &mut SolverBodies, coeff: Real) {
        self.impulse *= coeff;
        self.apply(bodies, self.impulse);
    }

    #[inline]
    pub fn solve(&mut self, bodies: &mut SolverBodies) {
        let vp = bodies.get_vel(self.particle).linear;
        let va = if self.body == u32::MAX {
            Vector::ZERO
        } else {
            let v = bodies.get_vel(self.body);
            v.linear + v.angular.gcross(self.arm)
        };
        let dv = vp - va;
        let total = self.impulse
            + self.inv_lhs * (dv + self.rhs - (self.lhs * self.impulse) * self.cfm_coeff);
        let delta = total - self.impulse;
        self.impulse = total;
        self.apply(bodies, delta);
    }
}
