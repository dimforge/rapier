use crate::math::Real;
use rapier::parry::shape::Cylinder;

/// Read-only access to the properties of a cylinder.
#[derive(Copy, Clone)]
pub struct CylinderView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Cylinder,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-height of the cylinder.
            pub fn half_height(&self) -> Real {
                self.raw.half_height
            }

            /// The base radius of the cylinder.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(CylinderView);

/// Read-write access to the properties of a cylinder.
pub struct CylinderViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Cylinder,
}

impl_ref_methods!(CylinderViewMut);

impl CylinderViewMut<'_> {
    /// Set the half-height of the cylinder.
    pub fn set_half_height(&mut self, half_height: Real) {
        self.raw.half_height = half_height;
    }

    /// Set the radius of the basis of the cylinder.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
