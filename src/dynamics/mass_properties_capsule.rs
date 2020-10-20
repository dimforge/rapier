use crate::dynamics::MassProperties;
#[cfg(feature = "dim3")]
use crate::geometry::Capsule;
use crate::math::{Point, PrincipalAngularInertia, Rotation, Vector};

impl MassProperties {
    pub(crate) fn from_capsule(density: f32, half_height: f32, radius: f32) -> Self {
        let (cyl_vol, cyl_unit_i) = Self::cylinder_y_volume_unit_inertia(half_height, radius);
        let (ball_vol, ball_unit_i) = Self::ball_volume_unit_angular_inertia(radius);
        let cap_vol = cyl_vol + ball_vol;
        let cap_mass = cap_vol * density;
        let mut cap_unit_i = cyl_unit_i + ball_unit_i;

        #[cfg(feature = "dim2")]
        {
            let h = half_height * 2.0;
            let extra = h * h * 0.5 + h * radius * 3.0 / 8.0;
            cap_unit_i += extra;
            Self::new(Point::origin(), cap_mass, cap_unit_i * cap_mass)
        }

        #[cfg(feature = "dim3")]
        {
            let h = half_height * 2.0;
            let extra = h * h * 0.5 + h * radius * 3.0 / 8.0;
            cap_unit_i.x += extra;
            cap_unit_i.z += extra;
            Self::with_principal_inertia_frame(
                Point::origin(),
                cap_mass,
                cap_unit_i * cap_mass,
                Rotation::identity(),
            )
        }
    }
}
