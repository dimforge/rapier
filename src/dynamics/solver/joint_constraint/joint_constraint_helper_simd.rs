pub(self) mod math {
    pub use crate::math::*;

    pub use crate::math::SimdAngVector as AngVector;
    pub use crate::math::SimdIsometry as Isometry;
    pub use crate::math::SimdMatrix as Matrix;
    pub use crate::math::SimdPoint as Point;
    pub use crate::math::SimdReal as Real;
    pub use crate::math::SimdRotation as Rotation;
    pub use crate::math::SimdVector as Vector;

    pub const LANES: usize = SIMD_WIDTH;
    pub type RealConst = crate::math::Real;
}

#[path = "./joint_constraint_helper_template.rs"]
mod joint_constraint_helper_template;

pub use joint_constraint_helper_template::JointConstraintHelperTemplate as JointConstraintHelperSimd;
