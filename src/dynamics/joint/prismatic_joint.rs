use crate::dynamics::SpringModel;
use crate::math::{Isometry, Point, Real, Vector, DIM};
use crate::utils::WBasis;
use na::Unit;
#[cfg(feature = "dim2")]
use na::Vector2;
#[cfg(feature = "dim3")]
use na::Vector5;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that removes all relative motion between two bodies, except for the translations along one axis.
pub struct PrismaticJoint {
    /// Where the prismatic joint is attached on the first body, expressed in the local space of the first attached body.
    pub local_anchor1: Point<Real>,
    /// Where the prismatic joint is attached on the second body, expressed in the local space of the second attached body.
    pub local_anchor2: Point<Real>,
    pub(crate) local_axis1: Unit<Vector<Real>>,
    pub(crate) local_axis2: Unit<Vector<Real>>,
    pub(crate) basis1: [Vector<Real>; DIM - 1],
    pub(crate) basis2: [Vector<Real>; DIM - 1],
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    #[cfg(feature = "dim3")]
    pub impulse: Vector5<Real>,
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    #[cfg(feature = "dim2")]
    pub impulse: Vector2<Real>,
    /// Whether or not this joint should enforce translational limits along its axis.
    pub limits_enabled: bool,
    /// The min an max relative position of the attached bodies along this joint's axis.
    pub limits: [Real; 2],
    /// The impulse applied by this joint on the first body to enforce the position limit along this joint's axis.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    pub limits_impulse: Real,

    /// The target relative angular velocity the motor will attempt to reach.
    pub motor_target_vel: Real,
    /// The target relative angle along the joint axis the motor will attempt to reach.
    pub motor_target_pos: Real,
    /// The motor's stiffness.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_stiffness: Real,
    /// The motor's damping.
    /// See the documentation of `SpringModel` for more information on this parameter.
    pub motor_damping: Real,
    /// The maximal impulse the motor is able to deliver.
    pub motor_max_impulse: Real,
    /// The angular impulse applied by the motor.
    pub motor_impulse: Real,
    /// The spring-like model used by the motor to reach the target velocity and .
    pub motor_model: SpringModel,
}

impl PrismaticJoint {
    /// Creates a new prismatic joint with the given point of applications and axis, all expressed
    /// in the local-space of the affected bodies.
    #[cfg(feature = "dim2")]
    pub fn new(
        local_anchor1: Point<Real>,
        local_axis1: Unit<Vector<Real>>,
        local_anchor2: Point<Real>,
        local_axis2: Unit<Vector<Real>>,
    ) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            local_axis1,
            local_axis2,
            basis1: local_axis1.orthonormal_basis(),
            basis2: local_axis2.orthonormal_basis(),
            impulse: na::zero(),
            limits_enabled: false,
            limits: [-Real::MAX, Real::MAX],
            limits_impulse: 0.0,
            motor_target_vel: 0.0,
            motor_target_pos: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_max_impulse: Real::MAX,
            motor_impulse: 0.0,
            motor_model: SpringModel::VelocityBased,
        }
    }

    /// Creates a new prismatic joint with the given point of applications and axis, all expressed
    /// in the local-space of the affected bodies.
    ///
    /// The local tangent are vector orthogonal to the local axis. It is used to compute a basis orthonormal
    /// to the joint's axis. If this tangent is set to zero, te orthonormal basis will be automatically
    /// computed arbitrarily.
    #[cfg(feature = "dim3")]
    pub fn new(
        local_anchor1: Point<Real>,
        local_axis1: Unit<Vector<Real>>,
        local_tangent1: Vector<Real>,
        local_anchor2: Point<Real>,
        local_axis2: Unit<Vector<Real>>,
        local_tangent2: Vector<Real>,
    ) -> Self {
        let basis1 = if let Some(local_bitangent1) =
            Unit::try_new(local_axis1.cross(&local_tangent1), 1.0e-3)
        {
            [
                local_bitangent1.cross(&local_axis1),
                local_bitangent1.into_inner(),
            ]
        } else {
            local_axis1.orthonormal_basis()
        };

        let basis2 = if let Some(local_bitangent2) =
            Unit::try_new(local_axis2.cross(&local_tangent2), 2.0e-3)
        {
            [
                local_bitangent2.cross(&local_axis2),
                local_bitangent2.into_inner(),
            ]
        } else {
            local_axis2.orthonormal_basis()
        };

        Self {
            local_anchor1,
            local_anchor2,
            local_axis1,
            local_axis2,
            basis1,
            basis2,
            impulse: na::zero(),
            limits_enabled: false,
            limits: [-Real::MAX, Real::MAX],
            limits_impulse: 0.0,
            motor_target_vel: 0.0,
            motor_target_pos: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_max_impulse: Real::MAX,
            motor_impulse: 0.0,
            motor_model: SpringModel::VelocityBased,
        }
    }

    /// The local axis of this joint, expressed in the local-space of the first attached body.
    pub fn local_axis1(&self) -> Unit<Vector<Real>> {
        self.local_axis1
    }

    /// The local axis of this joint, expressed in the local-space of the second attached body.
    pub fn local_axis2(&self) -> Unit<Vector<Real>> {
        self.local_axis2
    }

    /// Can a SIMD constraint be used for resolving this joint?
    pub fn supports_simd_constraints(&self) -> bool {
        // SIMD revolute constraints don't support motors right now.
        self.motor_max_impulse == 0.0 || (self.motor_stiffness == 0.0 && self.motor_damping == 0.0)
    }

    // FIXME: precompute this?
    #[cfg(feature = "dim2")]
    pub(crate) fn local_frame1(&self) -> Isometry<Real> {
        use na::{Matrix2, Rotation2, UnitComplex};

        let mat = Matrix2::from_columns(&[self.local_axis1.into_inner(), self.basis1[0]]);
        let rotmat = Rotation2::from_matrix_unchecked(mat);
        let rotation = UnitComplex::from_rotation_matrix(&rotmat);
        let translation = self.local_anchor1.coords.into();
        Isometry::from_parts(translation, rotation)
    }

    // FIXME: precompute this?
    #[cfg(feature = "dim2")]
    pub(crate) fn local_frame2(&self) -> Isometry<Real> {
        use na::{Matrix2, Rotation2, UnitComplex};

        let mat = Matrix2::from_columns(&[self.local_axis2.into_inner(), self.basis2[0]]);
        let rotmat = Rotation2::from_matrix_unchecked(mat);
        let rotation = UnitComplex::from_rotation_matrix(&rotmat);
        let translation = self.local_anchor2.coords.into();
        Isometry::from_parts(translation, rotation)
    }

    // FIXME: precompute this?
    #[cfg(feature = "dim3")]
    pub(crate) fn local_frame1(&self) -> Isometry<Real> {
        use na::{Matrix3, Rotation3, UnitQuaternion};

        let mat = Matrix3::from_columns(&[
            self.local_axis1.into_inner(),
            self.basis1[0],
            self.basis1[1],
        ]);
        let rotmat = Rotation3::from_matrix_unchecked(mat);
        let rotation = UnitQuaternion::from_rotation_matrix(&rotmat);
        let translation = self.local_anchor1.coords.into();
        Isometry::from_parts(translation, rotation)
    }

    // FIXME: precompute this?
    #[cfg(feature = "dim3")]
    pub(crate) fn local_frame2(&self) -> Isometry<Real> {
        use na::{Matrix3, Rotation3, UnitQuaternion};

        let mat = Matrix3::from_columns(&[
            self.local_axis2.into_inner(),
            self.basis2[0],
            self.basis2[1],
        ]);
        let rotmat = Rotation3::from_matrix_unchecked(mat);
        let rotation = UnitQuaternion::from_rotation_matrix(&rotmat);
        let translation = self.local_anchor2.coords.into();
        Isometry::from_parts(translation, rotation)
    }

    pub fn configure_motor_model(&mut self, model: SpringModel) {
        self.motor_model = model;
    }

    pub fn configure_motor_velocity(&mut self, target_vel: Real, factor: Real) {
        self.configure_motor(self.motor_target_pos, target_vel, 0.0, factor)
    }

    pub fn configure_motor_position(&mut self, target_pos: Real, stiffness: Real, damping: Real) {
        self.configure_motor(target_pos, 0.0, stiffness, damping)
    }

    pub fn configure_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) {
        self.motor_target_vel = target_vel;
        self.motor_target_pos = target_pos;
        self.motor_stiffness = stiffness;
        self.motor_damping = damping;
    }
}
