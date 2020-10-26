use crate::dynamics::MassProperties;
use crate::math::{Point, PrincipalAngularInertia, Rotation, Vector};

impl MassProperties {
    pub(crate) fn cone_y_volume_unit_inertia(
        half_height: f32,
        radius: f32,
    ) -> (f32, PrincipalAngularInertia<f32>) {
        let volume = radius * radius * std::f32::consts::PI * half_height * 2.0 / 3.0;
        let sq_radius = radius * radius;
        let sq_height = half_height * half_height * 4.0;
        let off_principal = sq_radius * 3.0 / 20.0 + sq_height * 3.0 / 5.0;
        let principal = sq_radius * 3.0 / 10.0;

        (volume, Vector::new(off_principal, principal, off_principal))
    }

    pub(crate) fn from_cone(density: f32, half_height: f32, radius: f32) -> Self {
        let (cyl_vol, cyl_unit_i) = Self::cone_y_volume_unit_inertia(half_height, radius);
        let cyl_mass = cyl_vol * density;

        Self::with_principal_inertia_frame(
            Point::new(0.0, -half_height / 2.0, 0.0),
            cyl_mass,
            cyl_unit_i * cyl_mass,
            Rotation::identity(),
        )
    }
}
