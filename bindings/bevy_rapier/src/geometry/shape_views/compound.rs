use super::ColliderView;
use crate::math::{Real, Rot, Vect};
use rapier::parry::shape::{Compound, CompoundFlags};

/// Read-only access to the properties of a compound shape.
#[derive(Copy, Clone)]
pub struct CompoundView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Compound,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The shapes of this compound shape.
            #[inline]
            pub fn shapes(&self) -> impl ExactSizeIterator<Item = (Vect, Rot, ColliderView<'_>)> {
                self.raw.shapes().iter().map(|(pos, shape)| {
                    #[cfg(feature = "dim2")]
                    let rot = pos.rotation.angle();
                    #[cfg(feature = "dim3")]
                    let rot = pos.rotation;
                    (pos.translation, rot, shape.as_typed_shape().into())
                })
            }

            /// The flags controlling the optional data associated to this compound shape.
            pub fn flags(&self) -> CompoundFlags {
                self.raw.flags()
            }
        }
    }
);

impl_ref_methods!(CompoundView);

/// Read-write access to the properties of a compound shape.
///
/// The parts of the compound shape are shared and cannot be modified through this view.
pub struct CompoundViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Compound,
}

impl_ref_methods!(CompoundViewMut);

impl CompoundViewMut<'_> {
    /// Sets the flags of this compound shape, computing or discarding its optional
    /// associated data.
    ///
    /// `weld_tolerance` is the distance (in ULPs) under which two corners of different parts
    /// are considered equal when detecting internal edges; `None` selects parry's default.
    pub fn set_flags(&mut self, flags: CompoundFlags, weld_tolerance: Option<Real>) {
        self.raw.set_flags(flags, weld_tolerance)
    }
}
