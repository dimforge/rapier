pub(crate) use contact_constraint_element::*;
pub(crate) use contact_constraints_set::{
    ContactConstraintsSet, joint_data_num_constraints, joint_num_constraints,
};
pub(crate) use contact_with_coulomb_friction::*;
pub(crate) use generic_contact_constraint::*;
pub(crate) use generic_contact_constraint_element::*;

#[cfg(feature = "dim3")]
pub(crate) use contact_with_twist_friction::*;

mod contact_constraint_element;
mod contact_constraints_set;
mod contact_with_coulomb_friction;
mod generic_contact_constraint;
mod generic_contact_constraint_element;

#[cfg(feature = "dim3")]
mod contact_with_twist_friction;

#[cfg(feature = "dim3")]
use crate::utils::ScalarType;
#[cfg(feature = "dim3")]
use crate::{math::DIM, utils::OrthonormalBasis};

#[inline]
#[cfg(feature = "dim3")]
pub(crate) fn compute_tangent_contact_directions<N: ScalarType>(
    force_dir1: &N::Vector,
    linvel1: &N::Vector,
    linvel2: &N::Vector,
) -> [N::Vector; DIM - 1] {
    use crate::utils::CrossProduct;
    use OrthonormalBasis;

    let _ = (linvel1, linvel2);

    // A DETERMINISTIC basis from the (frozen while recycled) contact normal.
    // It must NOT follow the relative velocity: near quiescence that direction
    // is pure noise, and re-applying warm-started friction in a randomly spun basis (a tangential
    // kick every step) pumps the friction-only mode of large stacks; the exact 2×2 tangent solve
    // makes the in-plane orientation accuracy-neutral.
    let tangent1 = force_dir1.orthonormal_vector();
    let bitangent1 = force_dir1.gcross(tangent1);

    [tangent1, bitangent1]
}
