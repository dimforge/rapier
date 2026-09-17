//! Tearing-related helpers of a soft body: connected pieces, tear resistance and particle damage.
use crate::alloc_prelude::*;
use super::super::SoftBody;
use crate::math::Real;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

impl SoftBody {
    /// The connected pieces of this soft body (particles joined by its elements), each sorted and
    /// ordered by first particle; linear in the elements. Tears split pieces off as soft bodies
    /// ([`Self::tear_edge`]), so only a body inserted as disconnected parts has more than one.
    pub fn connected_pieces(&self) -> Vec<Vec<u32>> {
        let n = self.particles.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let up = parent[parent[i as usize] as usize];
                parent[i as usize] = up;
                i = up;
            }
            i
        }
        let mut union = |vertices: &[u32]| {
            for w in vertices.windows(2) {
                let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
                if a != b {
                    parent[a.max(b) as usize] = a.min(b);
                }
            }
        };
        for e in &self.edges {
            union(&e.vertices);
        }
        for c in &self.cells {
            union(&c.vertices);
        }
        #[cfg(feature = "dim3")]
        for d in &self.dihedrals {
            union(&d.vertices);
        }
        for s in &self.boundary {
            union(s);
        }
        // Roots are the smallest index of their piece, so the pieces come out ordered.
        let mut piece_of_root = vec![u32::MAX; n];
        let mut pieces: Vec<Vec<u32>> = Vec::new();
        for v in 0..n as u32 {
            let root = find(&mut parent, v) as usize;
            if piece_of_root[root] == u32::MAX {
                piece_of_root[root] = pieces.len() as u32;
                pieces.push(Vec::new());
            }
            pieces[piece_of_root[root] as usize].push(v);
        }
        pieces
    }

    /// The components of the body's element graph restricted to `members` (all particles when
    /// `None`) that contain a particle of some `seeds` pair (the pairs a tear or cut separated, see
    /// [`crate::dynamics::SoftBodyTearEvent::split_particles`]), sorted; empty unless at least two.
    pub fn seeded_components(&self, members: Option<&[u32]>, seeds: &[[u32; 2]]) -> Vec<Vec<u32>> {
        let n = self.particles.len();
        let member =
            |v: u32| (v as usize) < n && members.is_none_or(|m| m.binary_search(&v).is_ok());
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let up = parent[parent[i as usize] as usize];
                parent[i as usize] = up;
                i = up;
            }
            i
        }
        // Pairwise over the in-set vertices: a chain through an out-of-set vertex must not break.
        let mut union = |vertices: &[u32]| {
            for i in 0..vertices.len() {
                if !member(vertices[i]) {
                    continue;
                }
                for j in i + 1..vertices.len() {
                    if !member(vertices[j]) {
                        continue;
                    }
                    let (a, b) = (find(&mut parent, vertices[i]), find(&mut parent, vertices[j]));
                    if a != b {
                        parent[a.max(b) as usize] = a.min(b);
                    }
                }
            }
        };
        for e in &self.edges {
            union(&e.vertices);
        }
        for c in &self.cells {
            union(&c.vertices);
        }
        #[cfg(feature = "dim3")]
        for d in &self.dihedrals {
            union(&d.vertices);
        }
        for s in &self.boundary {
            union(s);
        }
        let mut seeded = vec![false; n];
        for pair in seeds {
            for &v in pair {
                if member(v) {
                    seeded[find(&mut parent, v) as usize] = true;
                }
            }
        }
        if seeded.iter().filter(|s| **s).count() < 2 {
            return Vec::new();
        }
        // Roots are the smallest index of their component, so the components come out ordered.
        let mut piece_of_root = vec![u32::MAX; n];
        let mut pieces: Vec<Vec<u32>> = Vec::new();
        let mut visit = |v: u32, parent: &mut [u32]| {
            let root = find(parent, v) as usize;
            if !seeded[root] {
                return;
            }
            if piece_of_root[root] == u32::MAX {
                piece_of_root[root] = pieces.len() as u32;
                pieces.push(Vec::new());
            }
            pieces[piece_of_root[root] as usize].push(v);
        };
        match members {
            Some(members) => members.iter().for_each(|&v| visit(v, &mut parent)),
            None => (0..n as u32).for_each(|v| visit(v, &mut parent)),
        }
        pieces
    }

    /// The summed nominal mass of the given particles (out-of-range indices ignored).
    pub fn mass_of(&self, particles: &[u32]) -> Real {
        particles
            .iter()
            .filter_map(|&v| self.particles.get(v as usize))
            .map(|p| p.mass)
            .sum()
    }

    /// Sets the tear-threshold multiplier of the `i`-th edge (see
    /// [`crate::dynamics::SoftBodyEdge::tear_resistance`]); `1.0` restores the material's threshold.
    pub fn set_edge_tear_resistance(&mut self, i: usize, resistance: Real) {
        if let Some(e) = self.edges.get_mut(i) {
            e.tear_resistance = resistance.max(0.0);
            self.modified = true;
        }
    }

    /// Sets the tear-threshold multiplier of the `i`-th cell (see
    /// [`crate::dynamics::SoftBodyCell::tear_resistance`]); `1.0` restores the material's threshold.
    pub fn set_cell_tear_resistance(&mut self, i: usize, resistance: Real) {
        if let Some(c) = self.cells.get_mut(i) {
            c.tear_resistance = resistance.max(0.0);
            self.modified = true;
        }
    }

    /// Marks the `i`-th particle as damaged, or repairs it (see
    /// [`crate::dynamics::SoftBodyParticle::is_damaged`]): a way to seed a weak spot where a tear should start.
    pub fn set_particle_damaged(&mut self, i: usize, damaged: bool) {
        if let Some(p) = self.particles.get_mut(i) {
            p.damaged = damaged;
            self.modified = true;
        }
    }

    /// The tear-threshold multiplier of an element of resistance `resistance` over the particles
    /// `vertices`: the material's interior strength applies on top when every particle is
    /// interior and undamaged.
    pub(crate) fn effective_tear_resistance(&self, resistance: Real, vertices: &[u32]) -> Real {
        let interior = self.material.interior_strength;
        if interior > 1.0
            && vertices.iter().all(|&v| {
                let p = &self.particles[v as usize];
                !p.on_surface && !p.damaged
            })
        {
            resistance * interior
        } else {
            resistance
        }
    }
}
