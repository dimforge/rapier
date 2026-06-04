//! Smooth, crease-aware per-vertex normal generation for visual meshes.
//!
//! Faceted mesh formats (notably STL, which most MuJoCo Menagerie robots
//! ship) store one normal per *triangle* with fully unshared vertices, so
//! the file normals — and any naive per-face recompute — can only ever
//! produce flat shading. [`smooth_vertex_normals`] regenerates normals the
//! way MuJoCo's compiler does: weld coincident vertices to recover the
//! shared-edge topology, then blend each corner from the area-weighted
//! normals of the incident faces, leaving sharp edges creased.

use std::collections::HashMap;

/// Dot threshold below which two adjacent faces form a crease and are *not*
/// blended into a shared normal: `cos(45°)`. Matches the crease angle
/// MuJoCo uses when it generates mesh normals.
const CREASE_COS: f64 = 0.707_106_781_186;
/// Position quantization grid (1 µm) used to weld coincident vertices. STL
/// duplicates shared corners with bit-identical coordinates, so this is
/// exact in practice while also collapsing any near-coincident vertices.
const INV_EPS: f64 = 1.0e6;

/// Compute per-vertex normals for a visual trimesh the way MuJoCo does:
/// weld coincident vertices to recover the shared-edge topology a faceted
/// source (STL) throws away, then blend each corner's normal from the
/// area-weighted normals of the incident faces. Adjacent faces whose
/// normals differ by more than the crease angle ([`CREASE_COS`]) are *not*
/// blended, so box edges stay crisp while curved surfaces read as smooth.
/// When `smooth` is set (MJCF `smoothnormal="true"`) the crease threshold
/// is disabled and every shared face is blended.
///
/// `vertices` is the per-corner vertex buffer (`[x, y, z]`) and `indices`
/// the triangle list into it. The returned buffer is parallel to
/// `vertices`, so a renderer can use it verbatim without merging vertices
/// (which would otherwise break per-vertex UV alignment).
pub fn smooth_vertex_normals(
    vertices: &[[f64; 3]],
    indices: &[[u32; 3]],
    smooth: bool,
) -> Vec<[f32; 3]> {
    // original vertex index -> welded vertex id.
    let mut welded: HashMap<[i64; 3], usize> = HashMap::new();
    let mut rep = vec![0usize; vertices.len()];
    for (i, p) in vertices.iter().enumerate() {
        let key = [
            (p[0] * INV_EPS).round() as i64,
            (p[1] * INV_EPS).round() as i64,
            (p[2] * INV_EPS).round() as i64,
        ];
        let next = welded.len();
        rep[i] = *welded.entry(key).or_insert(next);
    }

    // Area-weighted face normals + per-welded-vertex incident faces.
    let mut face_n: Vec<[f64; 3]> = Vec::with_capacity(indices.len());
    let mut incident: Vec<Vec<usize>> = vec![Vec::new(); welded.len()];
    for (fi, t) in indices.iter().enumerate() {
        let a = vertices[t[0] as usize];
        let b = vertices[t[1] as usize];
        let c = vertices[t[2] as usize];
        // Un-normalized cross product = 2·area·unit_normal → area weighting.
        face_n.push(cross(sub(b, a), sub(c, a)));
        for &corner in t {
            incident[rep[corner as usize]].push(fi);
        }
    }

    // Blend each corner from the incident faces within the crease angle.
    let mut out = vec![[0.0f32; 3]; vertices.len()];
    for (fi, t) in indices.iter().enumerate() {
        let nf = normalize_or(face_n[fi], [0.0, 0.0, 1.0]);
        for &corner in t {
            let mut acc = [0.0f64; 3];
            for &gf in &incident[rep[corner as usize]] {
                if smooth || dot(nf, normalize_or(face_n[gf], nf)) >= CREASE_COS {
                    acc = add(acc, face_n[gf]);
                }
            }
            let n = normalize_or(acc, nf);
            out[corner as usize] = [n[0] as f32, n[1] as f32, n[2] as f32];
        }
    }
    out
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn normalize_or(v: [f64; 3], fallback: [f64; 3]) -> [f64; 3] {
    let len = dot(v, v).sqrt();
    if len > 1.0e-12 {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::smooth_vertex_normals;

    fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    /// Flatten triangles STL-style: every triangle gets its own three
    /// vertices (no sharing), so the welding inside `smooth_vertex_normals`
    /// is what must recover the shared edges — exactly the faceted-input
    /// case that motivated the function.
    fn flatten(tris: &[[[f64; 3]; 3]]) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let mut verts = Vec::new();
        let mut idx = Vec::new();
        for t in tris {
            let base = verts.len() as u32;
            verts.extend_from_slice(t);
            idx.push([base, base + 1, base + 2]);
        }
        (verts, idx)
    }

    /// Two triangles hinged along the shared edge (0,0,0)-(1,0,0) with the
    /// angle `beta` (radians) between their face normals. Face 1's normal is
    /// +Z; vertices are ordered so the shared corners land at indices 0
    /// (face 1) and 4 (face 2).
    fn hinge(beta: f64) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
        let e0 = [0.0, 0.0, 0.0];
        let e1 = [1.0, 0.0, 0.0];
        let w1 = [0.5, 1.0, 0.0]; // face 1 lies in z=0 → normal +Z
        let w2 = [0.5, -beta.cos(), beta.sin()]; // face 2 normal = (0, sinβ, cosβ)
        flatten(&[[e0, e1, w1], [e1, e0, w2]])
    }

    #[test]
    fn sharp_edge_stays_creased() {
        // 90° between faces > the 45° crease threshold: the two copies of
        // the shared corner must keep their own face normal (no blending).
        let (v, i) = hinge(std::f64::consts::FRAC_PI_2);
        let n = smooth_vertex_normals(&v, &i, /*smooth=*/ false);
        assert_eq!(n.len(), v.len());
        // Corner of face 1 at the shared edge → still +Z.
        assert!(dot(n[0], [0.0, 0.0, 1.0]) > 0.999, "{:?}", n[0]);
        // Same position, corner of face 2 → its own +Y normal: a crease.
        assert!(dot(n[4], [0.0, 1.0, 0.0]) > 0.999, "{:?}", n[4]);
    }

    #[test]
    fn shallow_edge_gets_smoothed() {
        // 20° between faces < the 45° threshold: the shared corners blend
        // to the average of the two face normals — the curved-surface case
        // that makes STL robots look faceted without this.
        let beta = 20.0_f64.to_radians();
        let (v, i) = hinge(beta);
        let n = smooth_vertex_normals(&v, &i, /*smooth=*/ false);
        // Blended normal bisects the two faces → cos(β/2) onto +Z, and is
        // *not* equal to either face normal (proving smoothing happened).
        let onto_z = dot(n[0], [0.0, 0.0, 1.0]);
        let expected = (beta / 2.0).cos() as f32;
        assert!((onto_z - expected).abs() < 1.0e-3, "{onto_z} vs {expected}");
        assert!(onto_z < 0.999, "should be blended, not flat: {onto_z}");
        // Both copies of the shared corner agree (fully smooth across edge).
        assert!(dot(n[0], n[4]) > 0.999, "{:?} vs {:?}", n[0], n[4]);
    }

    #[test]
    fn smoothnormal_forces_blend_across_sharp_edge() {
        // With `smooth=true` (MJCF smoothnormal) even the 90° edge blends.
        let (v, i) = hinge(std::f64::consts::FRAC_PI_2);
        let n = smooth_vertex_normals(&v, &i, /*smooth=*/ true);
        assert!(dot(n[0], n[4]) > 0.999, "{:?} vs {:?}", n[0], n[4]);
        assert!(dot(n[0], [0.0, 0.0, 1.0]) < 0.999, "should blend: {:?}", n[0]);
    }
}
