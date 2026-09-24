use crate::math::Real;
use rapier::parry::shape::Ball;

/// Read-only access to the properties of a ball.
#[derive(Copy, Clone)]
pub struct BallView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Ball,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The radius of the ball.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(BallView);

/// Read-write access to the properties of a ball.
pub struct BallViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Ball,
}

impl_ref_methods!(BallViewMut);

impl BallViewMut<'_> {
    /// Set the radius of the ball.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
