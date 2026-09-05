//! Exact crossing predicates shared by the tangle detection and the volume constraints.

use crate::math::Vector;
#[cfg(feature = "dim3")]
use crate::utils::DotProduct;

/// Whether two segments intersect (sign tests; parallel or degenerate pairs never cross).
#[cfg(feature = "dim2")]
pub(crate) fn segments_cross(a: [Vector; 2], b: [Vector; 2]) -> bool {
    let (d1, d2) = (a[1] - a[0], b[1] - b[0]);
    let denom = d1.perp_dot(d2);
    if denom.abs() < 1.0e-12 {
        return false;
    }
    let t = (b[0] - a[0]).perp_dot(d2) / denom;
    let u = (b[0] - a[0]).perp_dot(d1) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Whether two triangles cross: any edge of one pierces the other. For a self pair, an
/// edge touching the other triangle's vertices is skipped (the fan around a shared vertex
/// is contact, not crossing), matching the recovery's self-crossing predicate.
#[cfg(feature = "dim3")]
pub(crate) fn tri_pair_crosses(
    ids_a: &[u32],
    pa: &[Vector; 3],
    ids_b: &[u32],
    qb: &[Vector; 3],
    is_self: bool,
) -> bool {
    let edge_hits = |ids_e: &[u32], pe: &[Vector; 3], ids_t: &[u32], pt: &[Vector; 3]| {
        for k in 0..3 {
            let (i, j) = (k, (k + 1) % 3);
            if is_self && (ids_t.contains(&ids_e[i]) || ids_t.contains(&ids_e[j])) {
                continue;
            }
            if segment_crosses_triangle(pe[i], pe[j], pt[0], pt[1], pt[2]) {
                return true;
            }
        }
        false
    };
    edge_hits(ids_a, pa, ids_b, qb) || edge_hits(ids_b, qb, ids_a, pa)
}

/// Whether the open segment strictly pierces the triangle (sign tests; touching or
/// degenerate configurations never cross).
#[cfg(feature = "dim3")]
pub(crate) fn segment_crosses_triangle(
    p: Vector,
    q: Vector,
    a: Vector,
    b: Vector,
    c: Vector,
) -> bool {
    let n = (b - a).cross(c - a);
    let dp = (p - a).gdot(n);
    let dq = (q - a).gdot(n);
    if dp == 0.0 || dq == 0.0 || dp.signum() == dq.signum() {
        return false;
    }
    let d = q - p;
    let s1 = (a - p).cross(b - p).gdot(d);
    let s2 = (b - p).cross(c - p).gdot(d);
    let s3 = (c - p).cross(a - p).gdot(d);
    (s1 > 0.0 && s2 > 0.0 && s3 > 0.0) || (s1 < 0.0 && s2 < 0.0 && s3 < 0.0)
}
