use super::SegmentView;
use crate::math::{Real, Rot, Vect};
use rapier::parry::shape::Capsule;

/// Read-only access to the properties of a capsule.
#[derive(Copy, Clone)]
pub struct CapsuleView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Capsule,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The axis and endpoint of the capsule.
            pub fn segment(&self) -> SegmentView<'_> {
                SegmentView {
                    raw: &self.raw.segment,
                }
            }

            /// The radius of the capsule.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }

            /// The height of this capsule.
            pub fn height(&self) -> Real {
                self.raw.height()
            }

            /// The half-height of this capsule.
            pub fn half_height(&self) -> Real {
                self.raw.half_height()
            }

            /// The center of this capsule.
            pub fn center(&self) -> Vect {
                self.raw.center().into()
            }

            /// The transformation such that `t * Y` is collinear with `b - a` and `t * origin` equals
            /// the capsule's center.
            pub fn canonical_transform(&self) -> (Vect, Rot) {
                let pose = self.raw.canonical_transform();
                #[cfg(feature = "dim2")]
                return (pose.translation, pose.rotation.angle());
                #[cfg(feature = "dim3")]
                return (pose.translation, pose.rotation);
            }

            /// The rotation `r` such that `r * Y` is collinear with `b - a`.
            #[cfg(feature = "dim2")]
            pub fn rotation_wrt_y(&self) -> Rot {
                self.raw.rotation_wrt_y().angle()
            }

            /// The rotation `r` such that `r * Y` is collinear with `b - a`.
            #[cfg(feature = "dim3")]
            pub fn rotation_wrt_y(&self) -> Rot {
                self.raw.rotation_wrt_y()
            }

            /// The transform `t` such that `t * Y` is collinear with `b - a` and such that `t * origin = (b + a) / 2.0`.
            pub fn transform_wrt_y(&self) -> (Vect, Rot) {
                let pose = self.raw.transform_wrt_y();
                #[cfg(feature = "dim2")]
                return (pose.translation, pose.rotation.angle());
                #[cfg(feature = "dim3")]
                return (pose.translation, pose.rotation);
            }
        }
    }
);

impl_ref_methods!(CapsuleView);

/// Read-write access to the properties of a capsule.
pub struct CapsuleViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Capsule,
}

impl_ref_methods!(CapsuleViewMut);

impl CapsuleViewMut<'_> {
    /// Set the segment of this capsule.
    pub fn set_segment(&mut self, start: Vect, end: Vect) {
        self.raw.segment.a = start;
        self.raw.segment.b = end;
    }

    /// Set the radius of this capsule.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
