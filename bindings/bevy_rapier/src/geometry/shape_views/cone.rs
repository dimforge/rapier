use crate::math::Real;
use rapier::parry::shape::Cone;

/// Read-only access to the properties of a cone.
#[derive(Copy, Clone)]
pub struct ConeView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Cone,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-height of the cone.
            pub fn half_height(&self) -> Real {
                self.raw.half_height
            }

            /// The base radius of the cone.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(ConeView);

/// Read-write access to the properties of a cone.
pub struct ConeViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Cone,
}

impl_ref_methods!(ConeViewMut);

impl ConeViewMut<'_> {
    /// Set the half-height of the cone.
    pub fn set_half_height(&mut self, half_height: Real) {
        self.raw.half_height = half_height;
    }

    /// Set the radius of the basis of the cone.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
