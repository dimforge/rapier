//! Greedy coloring of a soft body's elements: two elements sharing a particle never get the
//! same color, so the solver can sweep a whole color in parallel. Runs at build time and again
//! whenever the topology changes (tearing).

use crate::alloc_prelude::*;

#[cfg(feature = "dim3")]
use super::SoftBodyDihedral;
use super::soft_body::SOFT_BODY_OVERFLOW_COLOR;
use super::{SoftBodyCell, SoftBodyEdge};

/// Colors every element of a soft body (edges, cells, dihedrals in one shared color space);
/// returns `(num_colors, has_overflow)` as [`color_elements`] does.
pub(crate) fn assign_colors(
    num_particles: usize,
    edges: &mut [SoftBodyEdge],
    cells: &mut [SoftBodyCell],
    #[cfg(feature = "dim3")] dihedrals: &mut [SoftBodyDihedral],
) -> (u8, bool) {
    let elements = edges
        .iter()
        .map(|e| &e.vertices[..])
        .chain(cells.iter().map(|c| &c.vertices[..]));
    #[cfg(feature = "dim3")]
    let elements = elements.chain(dihedrals.iter().map(|d| &d.vertices[..]));
    let (colors, num_colors, has_overflow) = color_elements(num_particles, elements);
    let mut colors = colors.into_iter();

    for e in edges.iter_mut() {
        e.color = colors.next().unwrap();
    }

    for c in cells.iter_mut() {
        c.color = colors.next().unwrap();
    }

    #[cfg(feature = "dim3")]
    for d in dihedrals.iter_mut() {
        d.color = colors.next().unwrap();
    }
    
    (num_colors, has_overflow)
}

/// Number of parallel colors available to soft-body elements (the `u128` per-particle masks).
const NUM_PARALLEL_COLORS: u32 = 128;

/// Assigns colors to elements (small sets of particle indices); returns `(colors, num_colors,
/// has_overflow)`: each element's color, the number of parallel colors (`0..num_colors`), and
/// whether some element got [`SOFT_BODY_OVERFLOW_COLOR`] (all colors taken at one particle).
pub(crate) fn color_elements<'a>(
    num_particles: usize,
    elements: impl Iterator<Item = &'a [u32]>,
) -> (Vec<u8>, u8, bool) {
    let mut particle_masks = vec![0u128; num_particles];
    let mut colors = Vec::new();
    let mut num_colors = 0u32;
    let mut has_overflow = false;

    for element in elements {
        let mut mask = 0u128;
        for &v in element {
            mask |= particle_masks[v as usize];
        }
        let free = !mask;
        let color = free.trailing_zeros();
        if color >= NUM_PARALLEL_COLORS {
            colors.push(SOFT_BODY_OVERFLOW_COLOR);
            has_overflow = true;
            continue;
        }
        num_colors = num_colors.max(color + 1);
        colors.push(color as u8);
        for &v in element {
            particle_masks[v as usize] |= 1u128 << color;
        }
    }

    (colors, num_colors as u8, has_overflow)
}
