use crate::dynamics::MassProperties;
#[cfg(feature = "dim3")]
use crate::math::Vector;
use crate::math::{Point, PrincipalAngularInertia};

impl MassProperties {
    pub(crate) fn ball_volume_unit_angular_inertia(
        radius: f32,
    ) -> (f32, PrincipalAngularInertia<f32>) {
        #[cfg(feature = "dim2")]
        {
            let volume = std::f32::consts::PI * radius * radius;
            let i = radius * radius / 2.0;
            (volume, i)
        }
        #[cfg(feature = "dim3")]
        {
            let volume = std::f32::consts::PI * radius * radius * radius * 4.0 / 3.0;
            let i = radius * radius * 2.0 / 5.0;

            (volume, Vector::repeat(i))
        }
    }

    pub(crate) fn from_ball(density: f32, radius: f32) -> Self {
        let (vol, unit_i) = Self::ball_volume_unit_angular_inertia(radius);
        let mass = vol * density;
        Self::new(Point::origin(), mass, unit_i * mass)
    }
}
