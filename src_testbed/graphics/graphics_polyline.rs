//! Render geometry of polylines: a stroked mesh in 2D, a tube in 3D. Both layouts only depend on
//! the segment count, so a deforming polyline is followed by rewriting its vertex buffer.

use kiss3d::prelude::*;
use rapier::geometry::Shape;
#[cfg(feature = "dim2")]
use rapier::geometry::ShapeType;

/// Half the width, in world units, of a rendered polyline's stroke.
#[cfg(feature = "dim2")]
const POLYLINE_HALF_WIDTH: f32 = 0.05;

/// The number of segments a round join is drawn with.
#[cfg(feature = "dim2")]
const POLYLINE_JOIN_SUBDIV: usize = 8;

/// The filled geometry of a stroked polyline: a quad per segment, plus a disc at every vertex
/// so the corners join cleanly. The layout only depends on the segment and vertex counts, so a
/// deforming polyline's vertex buffer can be rewritten in place (see `update_deformable_node`).
#[cfg(feature = "dim2")]
pub(super) fn stroked_polyline(vertices: &[Vec2], indices: &[[u32; 2]]) -> (Vec<Vec2>, Vec<[u32; 3]>) {
    let mut vtx = Vec::with_capacity(indices.len() * 4 + vertices.len() * (POLYLINE_JOIN_SUBDIV + 1));
    let mut idx = Vec::with_capacity(indices.len() * 2 + vertices.len() * POLYLINE_JOIN_SUBDIV);

    for segment in indices {
        let (a, b) = (vertices[segment[0] as usize], vertices[segment[1] as usize]);
        let dir = (b - a).normalize_or_zero();
        let normal = Vec2::new(-dir.y, dir.x) * POLYLINE_HALF_WIDTH;
        let base = vtx.len() as u32;
        vtx.extend_from_slice(&[a + normal, a - normal, b - normal, b + normal]);
        idx.push([base, base + 1, base + 2]);
        idx.push([base, base + 2, base + 3]);
    }

    for vertex in vertices {
        let center = vtx.len() as u32;
        vtx.push(*vertex);
        for k in 0..POLYLINE_JOIN_SUBDIV {
            let angle = k as f32 / POLYLINE_JOIN_SUBDIV as f32 * std::f32::consts::TAU;
            vtx.push(*vertex + Vec2::new(angle.cos(), angle.sin()) * POLYLINE_HALF_WIDTH);
        }
        for k in 0..POLYLINE_JOIN_SUBDIV {
            let next = (k + 1) % POLYLINE_JOIN_SUBDIV;
            idx.push([
                center,
                center + 1 + k as u32,
                center + 1 + next as u32,
            ]);
        }
    }

    (vtx, idx)
}

/// The vertices and segments of a shape drawn as a polyline (`None` for anything else).
#[cfg(feature = "dim2")]
pub(super) fn polyline_geometry(shape: &dyn Shape) -> Option<(Vec<Vec2>, Vec<[u32; 2]>)> {
    let to_vec2 = |pts: &[rapier::math::Vector]| -> Vec<Vec2> {
        pts.iter().map(|p| Vec2::new(p.x as f32, p.y as f32)).collect()
    };
    match shape.shape_type() {
        ShapeType::Polyline => {
            let polyline = shape.as_polyline()?;
            Some((to_vec2(polyline.vertices()), polyline.indices().to_vec()))
        }
        ShapeType::HeightField => {
            let (vertices, indices) = shape.as_heightfield()?.to_polyline();
            Some((to_vec2(&vertices), indices))
        }
        _ => None,
    }
}

/// The radius, in world units, of a rendered 3D polyline's tube.
#[cfg(feature = "dim3")]
const POLYLINE_RADIUS: f32 = 0.04;

/// The number of sides of that tube.
#[cfg(feature = "dim3")]
const POLYLINE_SIDES: usize = 6;

/// A 3D polyline as a closed tube: a ring of `POLYLINE_SIDES` vertices at each end of every
/// segment, its sides, and a flat cap on each end. The layout only depends on the segment count,
/// so a deforming polyline (a soft body's wire) is followed by rewriting the vertex buffer.
#[cfg(feature = "dim3")]
pub(super) fn tube_polyline(vertices: &[Vec3], indices: &[[u32; 2]]) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let ring = POLYLINE_SIDES as u32;
    let mut vtx = Vec::with_capacity(indices.len() * (2 * POLYLINE_SIDES + 2));
    let mut idx = Vec::with_capacity(indices.len() * 4 * POLYLINE_SIDES);

    for segment in indices {
        let (a, b) = (vertices[segment[0] as usize], vertices[segment[1] as usize]);
        let dir = (b - a).normalize_or_zero();
        // Any basis orthogonal to the segment: the tube is round, so its roll does not matter.
        let up = if dir.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let u = dir.cross(up).normalize_or_zero() * POLYLINE_RADIUS;
        let v = dir.cross(u.normalize_or_zero()) * POLYLINE_RADIUS;

        let base = vtx.len() as u32;
        for center in [a, b] {
            for k in 0..POLYLINE_SIDES {
                let angle = k as f32 / POLYLINE_SIDES as f32 * std::f32::consts::TAU;
                vtx.push(center + u * angle.cos() + v * angle.sin());
            }
        }
        vtx.push(a);
        vtx.push(b);

        for k in 0..ring {
            let next = (k + 1) % ring;
            // Sides.
            idx.push([base + k, base + ring + k, base + ring + next]);
            idx.push([base + k, base + ring + next, base + next]);
            // Caps.
            idx.push([base + 2 * ring, base + next, base + k]);
            idx.push([base + 2 * ring + 1, base + ring + k, base + ring + next]);
        }
    }

    (vtx, idx)
}

/// The vertices and segments of a 3D shape drawn as a polyline (`None` for anything else).
#[cfg(feature = "dim3")]
pub(super) fn polyline_geometry(shape: &dyn Shape) -> Option<(Vec<Vec3>, Vec<[u32; 2]>)> {
    let polyline = shape.as_polyline()?;
    let vertices = polyline
        .vertices()
        .iter()
        .map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    Some((vertices, polyline.indices().to_vec()))
}

#[cfg(test)]
mod tests {
    /// The tube built for a 3D polyline has a fixed layout per segment, which is what lets a
    /// deforming wire be followed by rewriting the vertex buffer.
    #[cfg(feature = "dim3")]
    #[test]
    fn tube_layout_is_fixed_per_segment() {
        use super::{POLYLINE_SIDES, tube_polyline};
        use kiss3d::prelude::Vec3;

        let vertices: Vec<Vec3> = (0..5)
            .map(|i| Vec3::new(i as f32 * 0.5, 0.0, 0.0))
            .collect();
        let segments: Vec<[u32; 2]> = (0..4).map(|i| [i, i + 1]).collect();
        let (vtx, idx) = tube_polyline(&vertices, &segments);

        assert_eq!(vtx.len(), segments.len() * (2 * POLYLINE_SIDES + 2));
        assert_eq!(idx.len(), segments.len() * 4 * POLYLINE_SIDES);
        assert!(
            vtx.iter().all(|v| v.is_finite()),
            "the tube went non-finite"
        );
        assert!(
            idx.iter().flatten().all(|i| (*i as usize) < vtx.len()),
            "the tube indexes a vertex it does not have"
        );

        // Moving the wire moves the tube with it, keeping the same layout.
        let moved: Vec<Vec3> = vertices.iter().map(|v| *v + Vec3::Y).collect();
        let (moved_vtx, _) = tube_polyline(&moved, &segments);
        assert_eq!(moved_vtx.len(), vtx.len());
        for (before, after) in vtx.iter().zip(&moved_vtx) {
            assert!((*after - *before - Vec3::Y).length() < 1.0e-5);
        }
    }

    /// A degenerate segment (two coincident points) must not produce NaNs.
    #[cfg(feature = "dim3")]
    #[test]
    fn a_degenerate_segment_stays_finite() {
        use super::tube_polyline;
        use kiss3d::prelude::Vec3;

        let (vtx, _) = tube_polyline(&[Vec3::ZERO, Vec3::ZERO], &[[0, 1]]);
        assert!(vtx.iter().all(|v| v.is_finite()));
    }
}
