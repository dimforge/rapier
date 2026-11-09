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

type SimdVector = <crate::math::SimdReal as crate::utils::ScalarType>::Vector;
type SimdBool = <crate::math::SimdReal as simba::simd::SimdValue>::SimdBool;

/// Per-lane 0/1 masks of the sides that are centred balls
/// ([`crate::geometry::ContactManifoldData::centered_ball`]) attached to a solver body.
#[inline]
pub(crate) fn centered_ball_lanes(
    manifolds: &[&crate::geometry::ContactManifold; crate::math::SIMD_WIDTH],
    ids1: [u32; crate::math::SIMD_WIDTH],
    ids2: [u32; crate::math::SIMD_WIDTH],
) -> [crate::math::SimdReal; 2] {
    use crate::math::{Real, SIMD_WIDTH, SimdReal};
    let lane = |ii: usize, side: usize, id: u32| {
        (manifolds[ii].data.centered_ball[side] && id != u32::MAX) as u32 as Real
    };
    [
        SimdReal::from(array![|ii| lane(ii, 0, ids1[ii])]),
        SimdReal::from(array![|ii| lane(ii, 1, ids2[ii])]),
    ]
}

/// The two world points whose difference along `force_dir1` is a solver contact's
/// separation. `p1`/`p2` are its anchors resolved through the current poses. A centred-ball
/// side instead rebuilds its point from the body centre and the frozen world arm, so that it
/// translates with the body without rotating with it: the frozen arms reach the midpoint of
/// the two surface points, which are `dist` apart along the normal.
#[inline]
#[allow(clippy::too_many_arguments)]
pub(crate) fn separation_points(
    p1: SimdVector,
    p2: SimdVector,
    world_com1: SimdVector,
    world_com2: SimdVector,
    dp1: SimdVector,
    dp2: SimdVector,
    dist: crate::math::SimdReal,
    force_dir1: SimdVector,
    balls: [SimdBool; 2],
) -> (SimdVector, SimdVector) {
    use simba::simd::SimdValue;
    let half = force_dir1 * (dist * crate::math::SimdReal::splat(0.5));
    (
        (world_com1 + dp1 + half).select(balls[0], p1),
        (world_com2 + dp2 - half).select(balls[1], p2),
    )
}
