//! The tear API of a soft body: tear marks, blade crossings, and the update of the state derived
//! from the elements after a tear or a cut.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::SoftBody;
use super::tearing_event::{CATASTROPHIC_STRESS, SoftBodyTearEvent};

/// A sorted particle-index pair.
pub(super) fn pair(a: u32, b: u32) -> [u32; 2] {
    [a.min(b), a.max(b)]
}

/// Number of vertices two faces share, the vertex `v` excluded.
pub(super) fn shared_others(a: &[u32], b: &[u32], v: u32) -> usize {
    a.iter().filter(|&&x| x != v && b.contains(&x)).count()
}

impl SoftBody {
    /// Marks the `i`-th edge as torn: the tear is applied at the end of the next step (use
    /// [`crate::dynamics::SoftBodySet::tear`] to tear immediately). A tear loses no material (see
    /// [`Self::rest_measure`]); pieces it disconnects become their own bodies ([`Self::origin`]).
    pub fn tear_edge(&mut self, i: usize) {
        if let Some(e) = self.edges.get_mut(i) {
            e.torn = true;
            // A requested tear is never paced by `max_tears_per_step`.
            e.stress = e.stress.max(CATASTROPHIC_STRESS);
            self.tearing_pending = true;
        }
    }

    /// Marks the `i`-th cell as torn: the tear is applied at the end of the next step. No cell is
    /// removed: one particle (nearest the centroid first) splits along the plane perpendicular to
    /// the cell's principal rest stretch direction (see [`Self::tear_edge`]).
    pub fn tear_cell(&mut self, i: usize) {
        if let Some(c) = self.cells.get_mut(i) {
            c.torn = true;
            self.tearing_pending = true;
        }
    }

    /// Whether some element has a pending tear mark.
    pub fn has_pending_tears(&self) -> bool {
        self.tearing_pending
    }

    /// The elements a blade (world segment in 2D, triangle in 3D) meets, as `(edge indices, cell
    /// indices)`: an edge when they intersect (endpoints included), a cell when one of its edges
    /// does. A preview of [`crate::dynamics::SoftBodySet::cut`], which removes none of them.
    pub fn crossing_elements(&self, blade: &[Vector; DIM]) -> (Vec<u32>, Vec<u32>) {
        let position = |i: u32| self.particles[i as usize].position;
        let crosses = |a: u32, b: u32| blade_hit(blade, position(a), position(b)).is_some();
        let edges = self
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| crosses(e.vertices[0], e.vertices[1]))
            .map(|(i, _)| i as u32)
            .collect();
        let cells = self
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| {
                (0..DIM + 1).any(|a| (a + 1..DIM + 1).any(|b| crosses(c.vertices[a], c.vertices[b])))
            })
            .map(|(i, _)| i as u32)
            .collect();
        (edges, cells)
    }

    /// Takes and clears the pending tear marks as (edge, cell) indices. At most
    /// `SoftBodyMaterial::max_tears_per_step` edges are taken (most loaded first, by smoothed
    /// `stress`); the others lose their mark (marked again next step if still overloaded).
    pub(crate) fn take_torn(&mut self) -> (Vec<u32>, Vec<u32>) {
        self.tearing_pending = false;
        let mut edges: Vec<(Real, u32)> = Vec::new();
        for (i, e) in self.edges.iter_mut().enumerate() {
            if core::mem::take(&mut e.torn) {
                edges.push((e.stress, i as u32));
            }
        }
        let limit = self.material.max_tears_per_step as usize;
        if edges.len() > limit {
            // The most loaded first; ties by index (deterministic). Edges loaded past twice
            // their threshold (the rim of a projectile punching through) all tear at once: the
            // limit paces the cracks that creep near the threshold, not a puncture.
            edges.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.1.cmp(&y.1)));
            let keep = edges
                .iter()
                .filter(|(stress, _)| *stress > CATASTROPHIC_STRESS)
                .count()
                .max(limit);
            edges.truncate(keep);
        }
        let mut edges: Vec<u32> = edges.into_iter().map(|(_, i)| i).collect();
        edges.sort_unstable();
        let mut cells = Vec::new();
        for (i, c) in self.cells.iter_mut().enumerate() {
            if core::mem::take(&mut c.torn) {
                cells.push(i as u32);
            }
        }
        (edges, cells)
    }

    /// Applies the given tears (see [`Self::tear_edge`]) and records them in `event`. Returns
    /// whether the topology changed; the colliders are not updated here (see `SoftBodySet::tear`).
    pub(crate) fn tear_topology(
        &mut self,
        torn_edges: &[u32],
        torn_cells: &[u32],
        event: &mut SoftBodyTearEvent,
        let any_cell_removed = cell_removed.contains(&true);
        if any_cell_removed {
            let mut kept_pairs: HashMap<[u32; 2], ()> = HashMap::default();
            let mut dead_pairs: HashMap<[u32; 2], ()> = HashMap::default();
            for (ci, c) in self.cells.iter().enumerate() {
                if !cell_removed[ci] {
                    for i in 0..=DIM {
                        for j in i + 1..=DIM {
                            kept_pairs.insert(pair(c.vertices[i], c.vertices[j]), ());
                        }
                    }
                }
            }
            for (ci, c) in self.cells.iter().enumerate() {
                if cell_removed[ci] {
                    for i in 0..=DIM {
                        for j in i + 1..=DIM {
                            let key = pair(c.vertices[i], c.vertices[j]);
                            if !kept_pairs.contains_key(&key) {
                                dead_pairs.insert(key, ());
                            }
                        }
                    }
                }
            }
                if !edge_removed[ei] && dead_pairs.contains_key(&key) {
            for (key, ()) in dead_pairs {
                torn_pairs.insert(key, ());
            }
        }
        // Surface elements owned by a removed cell or spanning a torn pair. A 3D cloth (no
        let mut surface_removed = vec![false; self.boundary.len()];
        for (si, element) in self.boundary.iter().enumerate() {
            let owner = self
                .boundary_element_cells
                .get(si)
                .copied()
                .unwrap_or(u32::MAX);
            let owner_removed =
                owner != u32::MAX && cell_removed.get(owner as usize) == Some(&true);
            if owner_removed
                || (!cloth && !torn_pairs.is_empty() && spans_pair(element, &torn_pairs))
            {
                surface_removed[si] = true;
            }
        }

        // Dihedrals spanning a torn pair or bending over a removed triangle.
        #[cfg(feature = "dim3")]
        let mut dihedral_removed = vec![false; self.dihedrals.len()];
        #[cfg(feature = "dim3")]
        {
            let mut removed_tris: HashMap<[u32; 3], ()> = HashMap::default();
            for (si, element) in self.boundary.iter().enumerate() {
                if surface_removed[si] {
                    let mut key = *element;
                    key.sort_unstable();
                    removed_tris.insert(key, ());
                }
            }
            for (di, d) in self.dihedrals.iter().enumerate() {
                let v = d.vertices;
                let mut t1 = [v[0], v[1], v[2]];
                let mut t2 = [v[0], v[1], v[3]];
                t1.sort_unstable();
                t2.sort_unstable();
                if spans_pair(&v, &torn_pairs)
                    || removed_tris.contains_key(&t1)
                    || removed_tris.contains_key(&t2)
                {
                    dihedral_removed[di] = true;
                }
            }
        }
                held >= 2
        // Split copies join the clusters of their source particle; the clusters' cell matches
        // follow the compacted cell list.
        self.inherit_cluster_membership(&duplicated);
        // the split copies, so the direct meshes keep their indices).
            cells: &cell_remap,
        true
    }
}
