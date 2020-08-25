use crate::dynamics::MassProperties;
#[cfg(feature = "dim3")]
use crate::geometry::Capsule;
use crate::math::{Point, PrincipalAngularInertia, Vector};

impl MassProperties {
    fn cylinder_y_volume_unit_inertia(
        half_height: f32,
        radius: f32,
    ) -> (f32, PrincipalAngularInertia<f32>) {
        #[cfg(feature = "dim2")]
        {
            Self::cuboid_volume_unit_inertia(Vector::new(radius, half_height))
        }

        #[cfg(feature = "dim3")]
        {
            let volume = half_height * radius * radius * std::f32::consts::PI * 2.0;
            let sq_radius = radius * radius;
            let sq_height = half_height * half_height * 4.0;
            let off_principal = (sq_radius * 3.0 + sq_height) / 12.0;

            let inertia = Vector::new(off_principal, sq_radius / 2.0, off_principal);
            (volume, inertia)
        }
    }

    pub(crate) fn from_capsule(density: f32, a: Point<f32>, b: Point<f32>, radius: f32) -> Self {
        let half_height = (b - a).norm() / 2.0;
        let (cyl_vol, cyl_unit_i) = Self::cylinder_y_volume_unit_inertia(half_height, radius);
        let (ball_vol, ball_unit_i) = Self::ball_volume_unit_angular_inertia(radius);
        let cap_vol = cyl_vol + ball_vol;
        let cap_mass = cap_vol * density;
        let mut cap_unit_i = cyl_unit_i + ball_unit_i;
        let local_com = na::center(&a, &b);

        #[cfg(feature = "dim2")]
        {
            let h = half_height * 2.0;
            let extra = h * h * 0.5 + h * radius * 3.0 / 8.0;
            cap_unit_i += extra;
            Self::new(local_com, cap_mass, cap_unit_i * cap_mass)
        }

        #[cfg(feature = "dim3")]
        {
            let h = half_height * 2.0;
            let extra = h * h * 0.5 + h * radius * 3.0 / 8.0;
            cap_unit_i.x += extra;
            cap_unit_i.z += extra;
            let local_frame = Capsule::new(a, b, radius).rotation_wrt_y();
            Self::with_principal_inertia_frame(
                local_com,
                cap_mass,
                cap_unit_i * cap_mass,
                local_frame,
            )
        }
    }
}
