use crate::dynamics::MassProperties;
use crate::math::{PrincipalAngularInertia, Vector};
#[cfg(feature = "dim3")]
use {
    crate::geometry::Capsule,
    crate::math::{Point, Rotation},
};

impl MassProperties {
    pub(crate) fn cylinder_y_volume_unit_inertia(
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

    #[cfg(feature = "dim3")]
    pub(crate) fn from_cylinder(density: f32, half_height: f32, radius: f32) -> Self {
        let (cyl_vol, cyl_unit_i) = Self::cylinder_y_volume_unit_inertia(half_height, radius);
        let cyl_mass = cyl_vol * density;

        Self::with_principal_inertia_frame(
            Point::origin(),
            cyl_mass,
            cyl_unit_i * cyl_mass,
            Rotation::identity(),
        )
    }
}
