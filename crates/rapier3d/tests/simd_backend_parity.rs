//! The SIMD backend's bitwise-parity contract.
//!
//! The solver is 4-lane AoSoA in every build, over `wide`'s `WideF32x4`. That
//! backend is only allowed to change codegen relative to the portable scalar
//! reference (`AutoF32x4`), never results — otherwise the platform-specific
//! intrinsics `wide` picks would each be their own determinism domain.
//!
//! This pins the guarantee at the op level, for every operation the solver
//! performs on `SimdReal` (see the `simd_*` call sites under `src/dynamics`).
//! [`simd_backend_determinism`] pins the same guarantee end-to-end on a whole
//! simulation; this one localizes a regression to the offending operation.
//!
//! `simd8` is deliberately out of scope: it changes the lane WIDTH, which
//! changes constraint bundling and is its own determinism domain.
#![cfg(not(feature = "simd8"))]

use rapier3d::na::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use simba::simd::{AutoF32x4, SimdBool as _, SimdPartialOrd, SimdRealField, SimdValue, WideF32x4};

struct XorShift(u64);

impl XorShift {
    /// A finite float in roughly [-8, 8], never exactly zero: `±0.0` ties in
    /// `min`/`max` are a documented bit-pattern caveat of the determinism
    /// contract (see `glam_backend_determinism`), not a backend divergence.
    fn f(&mut self) -> f32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        let f = ((((x >> 32) as u32) as f32) / u32::MAX as f32 - 0.5) * 16.0;
        if f == 0.0 { 1.0 } else { f }
    }
}

/// Runs every op the solver uses, generically over the backend, and flattens
/// each result to lanes so two backends can be compared bit for bit.
fn ops<N>(s: &[f32], out: &mut Vec<(String, [f32; 4])>)
where
    N: SimdRealField<Element = f32> + SimdValue<Element = f32> + SimdPartialOrd + Copy,
{
    let lane = |i: usize| {
        let mut n = N::splat(s[i]);
        for k in 1..4 {
            n.replace(k, s[(i + k) % s.len()]);
        }
        n
    };
    let lanes = |n: N| -> [f32; 4] { core::array::from_fn(|k| n.extract(k)) };

    macro_rules! push {
        ($name:expr, $n:expr) => {
            out.push(($name.to_string(), lanes($n)))
        };
    }
    macro_rules! push_v {
        ($name:expr, $v:expr) => {
            for (k, c) in $v.as_slice().iter().enumerate() {
                out.push((format!("{}[{}]", $name, k), lanes(*c)));
            }
        };
    }

    let (a, b) = (lane(0), lane(4));

    // Arithmetic and the reciprocal the solver leans on hardest (`simd_inv`).
    push!("add", a + b);
    push!("sub", a - b);
    push!("mul", a * b);
    push!("div", a / b);
    push!("neg", -a);
    push!("inv", N::one() / a);
    push!("sqrt", a.simd_abs().simd_sqrt());
    push!("abs", a.simd_abs());
    push!("signum", a.simd_signum());
    push!("copysign", a.simd_copysign(b));
    push!("max", a.simd_max(b));
    push!("min", a.simd_min(b));
    push!("clamp", a.simd_clamp(-b.simd_abs(), b.simd_abs()));
    push!("two_pi_mul", a * N::simd_two_pi());

    // Comparisons feed `select`, so compare the selected values (a lane mask
    // has no bit pattern of its own to hash).
    push!("gt", a.simd_gt(b).if_else(|| a, || b));
    push!("ge", a.simd_ge(b).if_else(|| a, || b));
    push!("lt", a.simd_lt(b).if_else(|| a, || b));
    push!("le", a.simd_le(b).if_else(|| a, || b));
    push!("ne", a.simd_ne(b).if_else(|| a, || b));
    push!(
        "and_select",
        (a.simd_gt(b) & a.simd_lt(-b)).if_else(|| a, || b)
    );

    // Transcendentals are only backend-stable under `enhanced-determinism`,
    // which routes both backends per-lane through libm; otherwise `wide` uses
    // polynomial approximations and `AutoSimd` the host libm.
    #[cfg(feature = "enhanced-determinism")]
    {
        push!("asin", (a / N::splat(8.0)).simd_asin());
        push!("sin", a.simd_sin());
        push!("cos", a.simd_cos());
        push!("atan2", a.simd_atan2(b));
    }

    // The nalgebra geometry the constraint math is built from.
    let v1 = Vector3::new(lane(0), lane(1), lane(2));
    let v2 = Vector3::new(lane(3), lane(4), lane(5));
    let q1 = UnitQuaternion::new_normalize(Quaternion::from_parts(
        lane(6),
        Vector3::new(lane(7), lane(8), lane(9)),
    ));
    let q2 = UnitQuaternion::new_normalize(Quaternion::from_parts(
        lane(10),
        Vector3::new(lane(11), lane(12), lane(13)),
    ));
    #[rustfmt::skip]
    let m = Matrix3::new(
        lane(0), lane(1), lane(2),
        lane(3), lane(4), lane(5),
        lane(6), lane(7), lane(8),
    );

    push!("dot", v1.dot(&v2));
    push!("norm", v1.norm());
    push_v!("cross", v1.cross(&v2));
    push_v!("normalize", v1.normalize());
    push_v!("quat*vec", q1 * v1);
    push_v!("quat_inv*vec", q1.inverse() * v1);
    push_v!("quat*quat", (q1 * q2).coords.xyz());
    push_v!(
        "quat->mat_col0",
        q1.to_rotation_matrix().into_inner().column(0).into_owned()
    );
    push_v!("mat*vec", m * v1);
    push_v!("matmat_col0", (m * m).column(0).into_owned());
    push_v!("mat_tr*vec", m.transpose() * v1);
}

#[test]
fn portable_backend_matches_wide_bitwise() {
    const ITERS: usize = 2000;
    let mut rng = XorShift(0x243f6a8885a308d3);
    let mut mismatches: Vec<String> = Vec::new();

    for iter in 0..ITERS {
        let s: Vec<f32> = (0..16).map(|_| rng.f()).collect();
        let (mut auto, mut wide) = (Vec::new(), Vec::new());
        ops::<AutoF32x4>(&s, &mut auto);
        ops::<WideF32x4>(&s, &mut wide);
        assert_eq!(
            auto.len(),
            wide.len(),
            "the two runs must cover the same ops"
        );

        for ((name, a), (_, w)) in auto.iter().zip(wide.iter()) {
            for k in 0..4 {
                if a[k].to_bits() != w[k].to_bits() {
                    mismatches.push(format!(
                        "iter{iter}/{name}/lane{k}: AutoF32x4 {:e} ({:08x}) vs WideF32x4 {:e} ({:08x})",
                        a[k],
                        a[k].to_bits(),
                        w[k],
                        w[k].to_bits()
                    ));
                }
            }
        }
    }

    if !mismatches.is_empty() {
        let mut per_op: std::collections::BTreeMap<&str, usize> = Default::default();
        for m in &mismatches {
            *per_op.entry(m.split('/').nth(1).unwrap()).or_default() += 1;
        }
        panic!(
            "{} bitwise mismatches over {ITERS} iterations; per-op counts: {per_op:?}\nfirst 6:\n{}",
            mismatches.len(),
            mismatches[..6.min(mismatches.len())].join("\n")
        );
    }
}
