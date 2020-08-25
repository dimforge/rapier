use crate::math::{AngVector, AngularInertia, Isometry, Point, Rotation, Vector};
use crate::utils;
use num::Zero;
use std::ops::{Add, AddAssign};
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
    #[cfg(feature = "dim2")]
    pub(crate) fn new(local_com: Point<f32>, mass: f32, principal_inertia: f32) -> Self {
        let inv_mass = utils::inv(mass);
        let inv_principal_inertia_sqrt = utils::inv(principal_inertia.sqrt());
        Self {
            local_com,
            inv_mass,
            inv_principal_inertia_sqrt,
        }
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn new(local_com: Point<f32>, mass: f32, principal_inertia: AngVector<f32>) -> Self {
        Self::with_principal_inertia_frame(local_com, mass, principal_inertia, Rotation::identity())
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn with_principal_inertia_frame(
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
    /// Reconstructs the angular inertia tensor of the rigid body from its principal inertia values and axii.
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
        if self.inv_mass != 0.0 {
            let mass = 1.0 / self.inv_mass;
            let i = utils::inv(self.inv_principal_inertia_sqrt * self.inv_principal_inertia_sqrt);
            i + shift.norm_squared() * mass
        } else {
            0.0
        }
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn construct_shifted_inertia_matrix(&self, shift: Vector<f32>) -> Matrix3<f32> {
        if self.inv_mass != 0.0 {
            let mass = 1.0 / self.inv_mass;
            let matrix = self.reconstruct_inertia_matrix();
            let diag = shift.norm_squared();
            let diagm = Matrix3::from_diagonal_element(diag);
            matrix + (diagm + shift * shift.transpose()) * mass
        } else {
            Matrix3::zeros()
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
        let principal_inertia_local_frame = Rotation::from_matrix(&eigen.eigenvectors);
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
