use crate::dynamics::{RigidBody, Velocity};
use crate::math::*;
use crate::prelude::Damping;

#[cfg(feature = "dim2")]
use crate::num::Zero;

#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct SolverBody {
    pub position: Isometry,
    pub integrated_vels: Velocity,
    pub im: Vector,
    pub sqrt_ii: AngularInertia<Real>,
    pub world_com: Point,
    pub ccd_thickness: Real,
    pub damping: Damping,
    pub local_com: Point,
}

impl SolverBody {
    pub fn from(rb: &RigidBody) -> Self {
        Self {
            position: rb.pos.position,
            integrated_vels: Velocity::zero(),
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
        self.integrated_vels = Velocity::zero();
        self.im = rb.mprops.effective_inv_mass;
        self.sqrt_ii = rb.mprops.effective_world_inv_inertia_sqrt;
        self.world_com = rb.mprops.world_com;
        self.ccd_thickness = rb.ccd.ccd_thickness;
        self.damping = rb.damping;
        self.local_com = rb.mprops.local_mprops.local_com;
    }
}
