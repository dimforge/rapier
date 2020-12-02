use crate::math::{AngVector, AngularInertia, Isometry, Point, Rotation, Vector};
use crate::utils;
use num::Zero;
use std::ops::{Add, AddAssign, Sub, SubAssign};
#[cfg(feature = "dim3")]
use {na::Matrix3, std::ops::MulAssign};

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The local mass properties of a rigid-body.
pub struct MassProperties {
    /// The center of mass of a rigid-body expressed in its local-space.
    pub local_com: Point<f32>,
    /// The inverse of the mass of a rigid-body.
    ///
    /// If this is zero, the rigid-body is assumed to have infinite mass.
    pub inv_mass: f32,
    /// The inverse of the principal angular inertia of the rigid-body.
    ///
    /// Components set to zero are assumed to be infinite along the corresponding principal axis.
    pub inv_principal_inertia_sqrt: AngVector<f32>,
    #[cfg(feature = "dim3")]
    /// The principal vectors of the local angular inertia tensor of the rigid-body.
    pub principal_inertia_local_frame: Rotation<f32>,
}

impl MassProperties {
    /// Initializes the mass properties with the given center-of-mass, mass, and angular inertia.
    ///
    /// The center-of-mass is specified in the local-space of the rigid-body.
    #[cfg(feature = "dim2")]
    pub fn new(local_com: Point<f32>, mass: f32, principal_inertia: f32) -> Self {
        let inv_mass = utils::inv(mass);
        let inv_principal_inertia_sqrt = utils::inv(principal_inertia.sqrt());
        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
        }
    }

    /// Initializes the mass properties from the given center-of-mass, mass, and principal angular inertia.
    ///
    /// The center-of-mass is specified in the local-space of the rigid-body.
    /// The principal angular inertia are the angular inertia along the coordinate axes in the local-space
    /// of the rigid-body.
    #[cfg(feature = "dim3")]
    pub fn new(local_com: Point<f32>, mass: f32, principal_inertia: AngVector<f32>) -> Self {
        Self::with_principal_inertia_frame(local_com, mass, principal_inertia, Rotation::identity())
    }

    /// Initializes the mass properties from the given center-of-mass, mass, and principal angular inertia.
    ///
    /// The center-of-mass is specified in the local-space of the rigid-body.
    /// The principal angular inertia are the angular inertia along the coordinate axes defined by
    /// the `principal_inertia_local_frame` expressed in the local-space of the rigid-body.
    #[cfg(feature = "dim3")]
    pub fn with_principal_inertia_frame(
        local_com: Point<f32>,
        mass: f32,
        principal_inertia: AngVector<f32>,
        principal_inertia_local_frame: Rotation<f32>,
    ) -> Self {
        let inv_mass = utils::inv(mass);
        let inv_principal_inertia_sqrt = principal_inertia.map(|e| utils::inv(e.sqrt()));
        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
            principal_inertia_local_frame,
        }
    }

    /// The world-space center of mass of the rigid-body.
    pub fn world_com(&self, pos: &Isometry<f32>) -> Point<f32> {
        pos * self.local_com
    }

    #[cfg(feature = "dim2")]
    /// The world-space inverse angular inertia tensor of the rigid-body.
    pub fn world_inv_inertia_sqrt(&self, _rot: &Rotation<f32>) -> AngularInertia<f32> {
        self.inv_principal_inertia_sqrt
    }

    #[cfg(feature = "dim3")]
    /// The world-space inverse angular inertia tensor of the rigid-body.
    pub fn world_inv_inertia_sqrt(&self, rot: &Rotation<f32>) -> AngularInertia<f32> {
        if !self.inv_principal_inertia_sqrt.is_zero() {
            let mut lhs = (rot * self.principal_inertia_local_frame)
                .to_rotation_matrix()
                .into_inner();
            let rhs = lhs.transpose();
            lhs.column_mut(0)
                .mul_assign(self.inv_principal_inertia_sqrt.x);
            lhs.column_mut(1)
                .mul_assign(self.inv_principal_inertia_sqrt.y);
            lhs.column_mut(2)
                .mul_assign(self.inv_principal_inertia_sqrt.z);
            let inertia = lhs * rhs;
            AngularInertia::from_sdp_matrix(inertia)
        } else {
            AngularInertia::zero()
        }
    }

    #[cfg(feature = "dim3")]
    /// Reconstructs the inverse angular inertia tensor of the rigid body from its principal inertia values and axes.
    pub fn reconstruct_inverse_inertia_matrix(&self) -> Matrix3<f32> {
        let inv_principal_inertia = self.inv_principal_inertia_sqrt.map(|e| e * e);
        self.principal_inertia_local_frame.to_rotation_matrix()
            * Matrix3::from_diagonal(&inv_principal_inertia)
            * self
                .principal_inertia_local_frame
                .inverse()
                .to_rotation_matrix()
    }

    #[cfg(feature = "dim3")]
    /// Reconstructs the angular inertia tensor of the rigid body from its principal inertia values and axes.
    pub fn reconstruct_inertia_matrix(&self) -> Matrix3<f32> {
        let principal_inertia = self.inv_principal_inertia_sqrt.map(|e| utils::inv(e * e));
        self.principal_inertia_local_frame.to_rotation_matrix()
            * Matrix3::from_diagonal(&principal_inertia)
            * self
                .principal_inertia_local_frame
                .inverse()
                .to_rotation_matrix()
    }

    #[cfg(feature = "dim2")]
    pub(crate) fn construct_shifted_inertia_matrix(&self, shift: Vector<f32>) -> f32 {
        let i = utils::inv(self.inv_principal_inertia_sqrt * self.inv_principal_inertia_sqrt);

        if self.inv_mass != 0.0 {
            let mass = 1.0 / self.inv_mass;
            i + shift.norm_squared() * mass
        } else {
            i
        }
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn construct_shifted_inertia_matrix(&self, shift: Vector<f32>) -> Matrix3<f32> {
        let matrix = self.reconstruct_inertia_matrix();

        if self.inv_mass != 0.0 {
            let mass = 1.0 / self.inv_mass;
            let diag = shift.norm_squared();
            let diagm = Matrix3::from_diagonal_element(diag);
            matrix + (diagm + shift * shift.transpose()) * mass
        } else {
            matrix
        }
    }

    /// Transform each element of the mass properties.
    pub fn transform_by(&self, m: &Isometry<f32>) -> Self {
        // NOTE: we don't apply the parallel axis theorem here
        // because the center of mass is also transformed.
        Self {
            local_com: m * self.local_com,
            inv_mass: self.inv_mass,
            inv_principal_inertia_sqrt: self.inv_principal_inertia_sqrt,
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: m.rotation * self.principal_inertia_local_frame,
        }
    }
}

impl Zero for MassProperties {
    fn zero() -> Self {
        Self {
            inv_mass: 0.0,
            inv_principal_inertia_sqrt: na::zero(),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: Rotation::identity(),
            local_com: Point::origin(),
        }
    }

    fn is_zero(&self) -> bool {
        *self == Self::zero()
    }
}

impl Sub<MassProperties> for MassProperties {
    type Output = Self;

    #[cfg(feature = "dim2")]
    fn sub(self, other: MassProperties) -> Self {
        if self.is_zero() || other.is_zero() {
            return self;
        }

        let m1 = utils::inv(self.inv_mass);
        let m2 = utils::inv(other.inv_mass);
        let inv_mass = utils::inv(m1 - m2);

        let local_com = (self.local_com * m1 - other.local_com.coords * m2) * inv_mass;
        let i1 = self.construct_shifted_inertia_matrix(local_com - self.local_com);
        let i2 = other.construct_shifted_inertia_matrix(local_com - other.local_com);
        let inertia = i1 - i2;
        // NOTE: we drop the negative eigenvalues that may result from subtraction rounding errors.
        let inv_principal_inertia_sqrt = utils::inv(inertia.max(0.0).sqrt());

        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
        }
    }

    #[cfg(feature = "dim3")]
    fn sub(self, other: MassProperties) -> Self {
        if self.is_zero() || other.is_zero() {
            return self;
        }

        let m1 = utils::inv(self.inv_mass);
        let m2 = utils::inv(other.inv_mass);
        let inv_mass = utils::inv(m1 - m2);
        let local_com = (self.local_com * m1 - other.local_com.coords * m2) * inv_mass;
        let i1 = self.construct_shifted_inertia_matrix(local_com - self.local_com);
        let i2 = other.construct_shifted_inertia_matrix(local_com - other.local_com);
        let inertia = i1 - i2;
        let eigen = inertia.symmetric_eigen();
        let principal_inertia_local_frame =
            Rotation::from_matrix_eps(&eigen.eigenvectors, 1.0e-6, 10, na::one());
        let principal_inertia = eigen.eigenvalues;
        // NOTE: we drop the negative eigenvalues that may result from subtraction rounding errors.
        let inv_principal_inertia_sqrt = principal_inertia.map(|e| utils::inv(e.max(0.0).sqrt()));

        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
            principal_inertia_local_frame,
        }
    }
}

impl SubAssign<MassProperties> for MassProperties {
    fn sub_assign(&mut self, rhs: MassProperties) {
        *self = *self - rhs
    }
}

impl Add<MassProperties> for MassProperties {
    type Output = Self;

    #[cfg(feature = "dim2")]
    fn add(self, other: MassProperties) -> Self {
        if self.is_zero() {
            return other;
        } else if other.is_zero() {
            return self;
        }

        let m1 = utils::inv(self.inv_mass);
        let m2 = utils::inv(other.inv_mass);
        let inv_mass = utils::inv(m1 + m2);
        let local_com = (self.local_com * m1 + other.local_com.coords * m2) * inv_mass;
        let i1 = self.construct_shifted_inertia_matrix(local_com - self.local_com);
        let i2 = other.construct_shifted_inertia_matrix(local_com - other.local_com);
        let inertia = i1 + i2;
        let inv_principal_inertia_sqrt = utils::inv(inertia.sqrt());

        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
        }
    }

    #[cfg(feature = "dim3")]
    fn add(self, other: MassProperties) -> Self {
        if self.is_zero() {
            return other;
        } else if other.is_zero() {
            return self;
        }

        let m1 = utils::inv(self.inv_mass);
        let m2 = utils::inv(other.inv_mass);
        let inv_mass = utils::inv(m1 + m2);
        let local_com = (self.local_com * m1 + other.local_com.coords * m2) * inv_mass;
        let i1 = self.construct_shifted_inertia_matrix(local_com - self.local_com);
        let i2 = other.construct_shifted_inertia_matrix(local_com - other.local_com);
        let inertia = i1 + i2;
        let eigen = inertia.symmetric_eigen();
        let principal_inertia_local_frame =
            Rotation::from_matrix_eps(&eigen.eigenvectors, 1.0e-6, 10, na::one());
        let principal_inertia = eigen.eigenvalues;
        let inv_principal_inertia_sqrt = principal_inertia.map(|e| utils::inv(e.sqrt()));

        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
            principal_inertia_local_frame,
        }
    }
}

impl AddAssign<MassProperties> for MassProperties {
    fn add_assign(&mut self, rhs: MassProperties) {
        *self = *self + rhs
    }
}

impl approx::AbsDiffEq for MassProperties {
    type Epsilon = f32;
    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        #[cfg(feature = "dim2")]
        let inertia_is_ok = self
            .inv_principal_inertia_sqrt
            .abs_diff_eq(&other.inv_principal_inertia_sqrt, epsilon);

        #[cfg(feature = "dim3")]
        let inertia_is_ok = self
            .reconstruct_inverse_inertia_matrix()
            .abs_diff_eq(&other.reconstruct_inverse_inertia_matrix(), epsilon);

        inertia_is_ok
            && self.local_com.abs_diff_eq(&other.local_com, epsilon)
            && self.inv_mass.abs_diff_eq(&other.inv_mass, epsilon)
            && self
                .inv_principal_inertia_sqrt
                .abs_diff_eq(&other.inv_principal_inertia_sqrt, epsilon)
    }
}

impl approx::RelativeEq for MassProperties {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        #[cfg(feature = "dim2")]
        let inertia_is_ok = self.inv_principal_inertia_sqrt.relative_eq(
            &other.inv_principal_inertia_sqrt,
            epsilon,
            max_relative,
        );

        #[cfg(feature = "dim3")]
        let inertia_is_ok = self.reconstruct_inverse_inertia_matrix().relative_eq(
            &other.reconstruct_inverse_inertia_matrix(),
            epsilon,
            max_relative,
        );

        inertia_is_ok
            && self
                .local_com
                .relative_eq(&other.local_com, epsilon, max_relative)
            && self
                .inv_mass
                .relative_eq(&other.inv_mass, epsilon, max_relative)
    }
}

#[cfg(test)]
mod test {
    use super::MassProperties;
    use crate::geometry::ColliderBuilder;
    use crate::math::{Point, Rotation, Vector};
    use approx::assert_relative_eq;
    use num::Zero;

    #[test]
    fn mass_properties_add_partial_zero() {
        let m1 = MassProperties {
            local_com: Point::origin(),
            inv_mass: 2.0,
            inv_principal_inertia_sqrt: na::zero(),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: Rotation::identity(),
        };
        let m2 = MassProperties {
            local_com: Point::origin(),
            inv_mass: 0.0,
            #[cfg(feature = "dim2")]
            inv_principal_inertia_sqrt: 1.0,
            #[cfg(feature = "dim3")]
            inv_principal_inertia_sqrt: Vector::new(1.0, 2.0, 3.0),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: Rotation::identity(),
        };
        let result = MassProperties {
            local_com: Point::origin(),
            inv_mass: 2.0,
            #[cfg(feature = "dim2")]
            inv_principal_inertia_sqrt: 1.0,
            #[cfg(feature = "dim3")]
            inv_principal_inertia_sqrt: Vector::new(1.0, 2.0, 3.0),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: Rotation::identity(),
        };

        assert_eq!(m1 + m2, result);
        assert_eq!(m2 + m1, result);
    }

    #[test]
    fn mass_properties_add_sub() {
        // Check that addition and subtraction of mass properties behave as expected.
        let c1 = ColliderBuilder::capsule_x(1.0, 2.0).build();
        let c2 = ColliderBuilder::capsule_y(3.0, 4.0).build();
        let c3 = ColliderBuilder::ball(5.0).build();

        let m1 = c1.mass_properties();
        let m2 = c2.mass_properties();
        let m3 = c3.mass_properties();
        let m1m2m3 = m1 + m2 + m3;

        assert_relative_eq!(m1 + m2, m2 + m1, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m1, m2 + m3, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m2, m1 + m3, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m3, m1 + m2, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - (m1 + m2), m3, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - (m1 + m3), m2, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - (m2 + m3), m1, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m1 - m2, m3, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m1 - m3, m2, epsilon = 1.0e-6);
        assert_relative_eq!(m1m2m3 - m2 - m3, m1, epsilon = 1.0e-6);
        assert_relative_eq!(
            m1m2m3 - m1 - m2 - m3,
            MassProperties::zero(),
            epsilon = 1.0e-6
        );
    }
}
