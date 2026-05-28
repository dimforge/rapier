//! Pose representation: 3D translation + rotation in canonical (`w, x, y, z`)
//! quaternion form.
//!
//! MJCF allows several rotation specifications (`quat`, `axisangle`, `euler`,
//! `xyaxes`, `zaxis`). The parser collapses all of them into a unit quaternion
//! so the loader doesn't have to care which form was used.

/// A pose: 3-vector position + unit quaternion (w, x, y, z).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pose {
    /// Position (m).
    pub pos: [f64; 3],
    /// Unit quaternion (w, x, y, z).
    pub quat: [f64; 4],
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose {
    /// The identity pose.
    pub const IDENTITY: Self = Self {
        pos: [0.0; 3],
        quat: [1.0, 0.0, 0.0, 0.0],
    };

    /// Build from translation only.
    pub const fn translation(pos: [f64; 3]) -> Self {
        Self {
            pos,
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// Multiplication of two unit quaternions (w, x, y, z).
pub fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// Apply a unit quaternion to a 3-vector (rotation only).
pub fn quat_apply(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let [vx, vy, vz] = v;
    // Rodrigues' rotation: v' = v + 2 q.xyz × (q.xyz × v + w v)
    let qx_qy_qz = (x, y, z);
    let cross1 = (
        qx_qy_qz.1 * vz - qx_qy_qz.2 * vy,
        qx_qy_qz.2 * vx - qx_qy_qz.0 * vz,
        qx_qy_qz.0 * vy - qx_qy_qz.1 * vx,
    );
    let t = (cross1.0 + w * vx, cross1.1 + w * vy, cross1.2 + w * vz);
    let cross2 = (
        qx_qy_qz.1 * t.2 - qx_qy_qz.2 * t.1,
        qx_qy_qz.2 * t.0 - qx_qy_qz.0 * t.2,
        qx_qy_qz.0 * t.1 - qx_qy_qz.1 * t.0,
    );
    [
        vx + 2.0 * cross2.0,
        vy + 2.0 * cross2.1,
        vz + 2.0 * cross2.2,
    ]
}

/// Compose two poses: `parent ∘ child`. Position is rotated then offset; rotation is composed.
pub fn pose_mul(parent: Pose, child: Pose) -> Pose {
    let rotated = quat_apply(parent.quat, child.pos);
    Pose {
        pos: [
            parent.pos[0] + rotated[0],
            parent.pos[1] + rotated[1],
            parent.pos[2] + rotated[2],
        ],
        quat: normalize(quat_mul(parent.quat, child.quat)),
    }
}

/// Normalize a quaternion. If the input is the zero vector, returns identity.
pub fn normalize(q: [f64; 4]) -> [f64; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n <= 1e-30 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

/// Build a quaternion from an axis (auto-normalized) and an angle in radians.
pub fn quat_from_axis_angle(axis: [f64; 3], angle_rad: f64) -> [f64; 4] {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n <= 1e-30 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let s = (angle_rad * 0.5).sin() / n;
    [
        (angle_rad * 0.5).cos(),
        axis[0] * s,
        axis[1] * s,
        axis[2] * s,
    ]
}

/// Build a quaternion from Euler angles using the given axis sequence.
///
/// `seq` is a 3-character string drawn from `{x, y, z, X, Y, Z}` that names
/// the axes in *intrinsic* application order (MJCF default `"xyz"`).
/// Lowercase = intrinsic (rotating frame), uppercase = extrinsic (fixed
/// frame). MJCF uses intrinsic by default.
pub fn quat_from_euler(angles: [f64; 3], seq: &str) -> [f64; 4] {
    let mut q = [1.0, 0.0, 0.0, 0.0];
    for (i, ch) in seq.chars().enumerate() {
        let angle = angles[i];
        let intrinsic = ch.is_ascii_lowercase();
        let axis = match ch.to_ascii_lowercase() {
            'x' => [1.0, 0.0, 0.0],
            'y' => [0.0, 1.0, 0.0],
            'z' => [0.0, 0.0, 1.0],
            _ => continue,
        };
        let r = quat_from_axis_angle(axis, angle);
        q = if intrinsic {
            quat_mul(q, r)
        } else {
            quat_mul(r, q)
        };
    }
    normalize(q)
}

/// Build a rotation that maps the world Z axis to `axis`. Used for `zaxis="..."`.
pub fn quat_from_z_axis(axis: [f64; 3]) -> [f64; 4] {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n <= 1e-30 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let z = [axis[0] / n, axis[1] / n, axis[2] / n];
    // Rotation that sends [0,0,1] to z is axis-angle around (Z × z).
    let cross = [-z[1], z[0], 0.0];
    let cos = z[2].clamp(-1.0, 1.0);
    let angle = cos.acos();
    let cn = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    if cn <= 1e-30 {
        if z[2] > 0.0 {
            [1.0, 0.0, 0.0, 0.0]
        } else {
            // 180° about X.
            [0.0, 1.0, 0.0, 0.0]
        }
    } else {
        quat_from_axis_angle(cross, angle)
    }
}

/// Build a rotation from `xyaxes="x1 x2 x3 y1 y2 y3"`. The Z axis is X × Y.
/// The Y axis is then re-orthonormalized (Z × X) to ensure orthonormality.
pub fn quat_from_xy_axes(x: [f64; 3], y: [f64; 3]) -> [f64; 4] {
    let mut x = x;
    let nx = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
    if nx <= 1e-30 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    x = [x[0] / nx, x[1] / nx, x[2] / nx];
    let mut z = [
        x[1] * y[2] - x[2] * y[1],
        x[2] * y[0] - x[0] * y[2],
        x[0] * y[1] - x[1] * y[0],
    ];
    let nz = (z[0] * z[0] + z[1] * z[1] + z[2] * z[2]).sqrt();
    if nz <= 1e-30 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    z = [z[0] / nz, z[1] / nz, z[2] / nz];
    let y_orth = [
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0],
    ];
    quat_from_basis(x, y_orth, z)
}

/// Build a quaternion from three orthonormal basis columns (x, y, z).
fn quat_from_basis(x: [f64; 3], y: [f64; 3], z: [f64; 3]) -> [f64; 4] {
    // Standard Shepperd's method.
    let trace = x[0] + y[1] + z[2];
    if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        [
            0.25 / s,
            (y[2] - z[1]) * s,
            (z[0] - x[2]) * s,
            (x[1] - y[0]) * s,
        ]
    } else if x[0] > y[1] && x[0] > z[2] {
        let s = 2.0 * (1.0 + x[0] - y[1] - z[2]).sqrt();
        [
            (y[2] - z[1]) / s,
            0.25 * s,
            (y[0] + x[1]) / s,
            (z[0] + x[2]) / s,
        ]
    } else if y[1] > z[2] {
        let s = 2.0 * (1.0 + y[1] - x[0] - z[2]).sqrt();
        [
            (z[0] - x[2]) / s,
            (y[0] + x[1]) / s,
            0.25 * s,
            (z[1] + y[2]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + z[2] - x[0] - y[1]).sqrt();
        [
            (x[1] - y[0]) / s,
            (z[0] + x[2]) / s,
            (z[1] + y[2]) / s,
            0.25 * s,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq_q(a: [f64; 4], b: [f64; 4]) -> bool {
        // Quaternions q and -q represent the same rotation.
        let same = (0..4).all(|i| (a[i] - b[i]).abs() < 1e-9);
        let neg = (0..4).all(|i| (a[i] + b[i]).abs() < 1e-9);
        same || neg
    }

    #[test]
    fn identity() {
        assert_eq!(Pose::IDENTITY.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn euler_xyz_roundtrip() {
        let q = quat_from_euler([0.5, 0.7, 1.1], "xyz");
        // Reconstruct via successive axis-angle multiplications (intrinsic).
        let qx = quat_from_axis_angle([1.0, 0.0, 0.0], 0.5);
        let qy = quat_from_axis_angle([0.0, 1.0, 0.0], 0.7);
        let qz = quat_from_axis_angle([0.0, 0.0, 1.0], 1.1);
        let expected = quat_mul(quat_mul(qx, qy), qz);
        assert!(approx_eq_q(q, expected));
    }

    #[test]
    fn z_axis() {
        let q = quat_from_z_axis([0.0, 0.0, 1.0]);
        assert!(approx_eq_q(q, [1.0, 0.0, 0.0, 0.0]));
    }

    #[test]
    fn pose_mul_identity() {
        let p = Pose {
            pos: [1.0, 2.0, 3.0],
            quat: quat_from_axis_angle([0.0, 0.0, 1.0], 1.0),
        };
        let r = pose_mul(Pose::IDENTITY, p);
        assert!(approx_eq_q(r.quat, p.quat));
        for i in 0..3 {
            assert!((r.pos[i] - p.pos[i]).abs() < 1e-12);
        }
    }
}
