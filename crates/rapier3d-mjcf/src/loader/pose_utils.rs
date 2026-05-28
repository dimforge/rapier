//! Pose / quaternion helpers used throughout the loader.

use mjcf_rs::pose::{Pose as MPose, quat_apply};
use rapier3d::math::{Pose, Real, Rotation, Vector};

pub(super) fn scale_pose(p: MPose, s: f64) -> MPose {
    MPose {
        pos: [p.pos[0] * s, p.pos[1] * s, p.pos[2] * s],
        quat: p.quat,
    }
}

pub(super) fn invert_pose(p: MPose) -> MPose {
    let q_inv = [p.quat[0], -p.quat[1], -p.quat[2], -p.quat[3]];
    let neg_pos = [-p.pos[0], -p.pos[1], -p.pos[2]];
    let pos = quat_apply(q_inv, neg_pos);
    MPose { pos, quat: q_inv }
}

pub(super) fn mpose_to_rapier(p: MPose) -> Pose {
    Pose::from_parts(
        Vector::new(p.pos[0] as Real, p.pos[1] as Real, p.pos[2] as Real),
        Rotation::from_xyzw(
            p.quat[1] as Real,
            p.quat[2] as Real,
            p.quat[3] as Real,
            p.quat[0] as Real,
        ),
    )
}

pub(super) fn mpose_to_rapier_with_shift(p: MPose, shift: Pose) -> Pose {
    shift * mpose_to_rapier(p)
}
