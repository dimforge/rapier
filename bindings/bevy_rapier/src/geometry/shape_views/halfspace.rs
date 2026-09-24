use crate::math::Vect;
use rapier::parry::shape::HalfSpace;

/// Read-only access to the properties of a half-space.
#[derive(Copy, Clone)]
pub struct HalfSpaceView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a HalfSpace,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The halfspace planar boundary's outward normal.
            pub fn normal(&self) -> Vect {
                self.raw.normal
            }
        }
    }
);

impl_ref_methods!(HalfSpaceView);

/// Read-write access to the properties of a half-space.
pub struct HalfSpaceViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut HalfSpace,
}

impl_ref_methods!(HalfSpaceViewMut);

impl HalfSpaceViewMut<'_> {
    /// Set the normal of the half-space.
    pub fn set_normal(&mut self, normal: Vect) {
        let normal: rapier::math::Vector = normal;
        if normal.length_squared() >= 1.0e-12 {
            self.raw.normal = normal.normalize();
        }
    }
}
