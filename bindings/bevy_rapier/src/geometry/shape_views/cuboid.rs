use crate::math::Vect;
use rapier::geometry::Cuboid;

/// Read-only access to the properties of a cuboid.
#[derive(Copy, Clone)]
pub struct CuboidView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Cuboid,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-extents of the cuboid.
            pub fn half_extents(&self) -> Vect {
                self.raw.half_extents.into()
            }
        }
    }
);

impl_ref_methods!(CuboidView);

/// Read-write access to the properties of a cuboid.
pub struct CuboidViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Cuboid,
}

impl_ref_methods!(CuboidViewMut);

impl CuboidViewMut<'_> {
    /// Set the half-extents of the cuboid.
    pub fn set_half_extents(&mut self, half_extents: Vect) {
        self.raw.half_extents = half_extents;
    }
}
