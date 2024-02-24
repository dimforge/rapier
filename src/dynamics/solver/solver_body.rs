use crate::dynamics::{RigidBody, RigidBodyVelocity};
use crate::math::{AngularInertia, Isometry, Point, Real, Vector};
use crate::prelude::RigidBodyDamping;

#[cfg(feature = "dim2")]
use crate::num::Zero;

#[derive(Copy, Clone, Debug)]
pub(crate) struct SolverBody {
    pub position: Isometry<Real>,
    pub integrated_vels: RigidBodyVelocity,
    pub im: Vector<Real>,
    pub sqrt_ii: AngularInertia<Real>,
    pub world_com: Point<Real>,
    pub ccd_thickness: Real,
    pub damping: RigidBodyDamping,
    pub local_com: Point<Real>,
}

impl Default for SolverBody {
    fn default() -> Self {
        Self {
            position: Isometry::identity(),
            integrated_vels: RigidBodyVelocity::zero(),
            im: na::zero(),
            sqrt_ii: AngularInertia::zero(),
            world_com: Point::origin(),
            ccd_thickness: 0.0,
            damping: RigidBodyDamping::default(),
            local_com: Point::origin(),
        }
    }
}

impl SolverBody {
    pub fn from(rb: &RigidBody) -> Self {
        Self {
            position: rb.pos.position,
            integrated_vels: RigidBodyVelocity::zero(),
            im: rb.mprops.effective_inv_mass,
            sqrt_ii: rb.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb.mprops.world_com,
            ccd_thickness: rb.ccd.ccd_thickness,
            damping: rb.damping,
            local_com: rb.mprops.local_mprops.local_com,
        }
    }

    pub fn copy_from(&mut self, rb: &RigidBody) {
        self.position = rb.pos.position;
        self.integrated_vels = RigidBodyVelocity::zero();
        self.im = rb.mprops.effective_inv_mass;
        self.sqrt_ii = rb.mprops.effective_world_inv_inertia_sqrt;
        self.world_com = rb.mprops.world_com;
        self.ccd_thickness = rb.ccd.ccd_thickness;
        self.damping = rb.damping;
        self.local_com = rb.mprops.local_mprops.local_com;
    }
}
