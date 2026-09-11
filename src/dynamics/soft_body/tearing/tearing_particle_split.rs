//! Duplication of the particles a tear passes through: the measure elements around a particle
//! (its fan), their split along a plane or into disconnected groups, and the elements that follow
//! the copies.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::tearing::{pair, shared_others};
use super::super::{SoftBody, SoftBodyParticle};

impl SoftBody {
    /// The kind of this body's measure elements, by priority: its cells, its surface triangles
    /// (3D, without cells), its structural edges. `None` when it has none of them.
    pub(super) fn split_particles(&mut self, candidates: &[u32]) -> Vec<(u32, u32)> {
        let mut duplicated = Vec::new();
        // Faces: cells when the body has some, the surface elements otherwise (3D only: the
        // segments of a 2D polyline never split, a polyline is cut by removing a segment).
        let use_cells = !self.cells.is_empty();
        let use_surface = !use_cells && DIM == 3 && !self.boundary.is_empty();
        if !use_cells && !use_surface {
            return duplicated;
        }
        // Two faces around `v` are adjacent when they share a facet through `v`.
        let needed = if use_cells { DIM - 1 } else { DIM - 2 };

        for &v in candidates {
            let face_ids: Vec<u32> = if use_cells {
                self.cells
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| c.vertices.contains(&v))
                    .map(|(i, _)| i as u32)
                    .collect()
            } else {
                self.boundary
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| s.contains(&v))
                    .map(|(i, _)| i as u32)
                    .collect()
            };
            if face_ids.len() < 2 {
                continue;
            }
            let face_vertices = |f: u32| -> Vec<u32> {
                if use_cells {
                    self.cells[f as usize].vertices.to_vec()
                } else {
                    self.boundary[f as usize].to_vec()
                }
            };
            // Union-find over the faces at `v`, groups numbered by first appearance.
            let n = face_ids.len();
            let mut parent: Vec<usize> = (0..n).collect();
            fn find(parent: &mut [usize], mut i: usize) -> usize {
                while parent[i] != i {
                    parent[i] = parent[parent[i]];
                    i = parent[i];
                }
                i
            }
            let verts: Vec<Vec<u32>> = face_ids.iter().map(|&f| face_vertices(f)).collect();
            for i in 0..n {
                for j in i + 1..n {
                    if shared_others(&verts[i], &verts[j], v) >= needed {
                        let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                        if ri != rj {
                            parent[ri.max(rj)] = ri.min(rj);
                        }
                    }
                }
            }
            let mut group_of_root: Vec<usize> = vec![usize::MAX; n];
            let mut group: Vec<usize> = vec![0; n];
            let mut num_groups = 0;
            for i in 0..n {
                let r = find(&mut parent, i);
                if group_of_root[r] == usize::MAX {
                    group_of_root[r] = num_groups;
                    num_groups += 1;
                }
                group[i] = group_of_root[r];
            }
            if num_groups < 2 {
                continue;
            }

            self.split_vertex(
                v,
                use_cells,
                &face_ids,
                &verts,
                &group,
                num_groups,
                &mut duplicated,
            );
        }
        duplicated
    }

    /// Opens a cloth crack across the torn edge `(a, b)`: one endpoint (the first, in index
    /// order, whose fan has triangles on both sides) is split along the plane through it
    /// perpendicular to the edge in the rest shape, the triangles on the other endpoint's side
    /// going to the copy. The material is left whole; the tension the edge carried is released
    /// through the crack instead.
    pub(super) fn crack_across(&mut self, a: u32, b: u32, duplicated: &mut Vec<(u32, u32)>) {
        for (v, other) in [(a, b), (b, a)] {
            let face_ids: Vec<u32> = self
                .boundary
                .iter()
                .enumerate()
                .filter(|(_, s)| s.contains(&v))
                .map(|(i, _)| i as u32)
                .collect();
            if face_ids.len() < 2 {
                continue;
            }
            let rest = |i: u32| self.particles[i as usize].rest_position;
            let axis = rest(other) - rest(v);
            let verts: Vec<Vec<u32>> = face_ids
                .iter()
                .map(|&f| self.boundary[f as usize].to_vec())
                .collect();
            // Group 1: the triangles whose rest centroid lies on the other endpoint's side.
            let group: Vec<usize> = verts
                .iter()
                .map(|fv| {
                    let mut c = Vector::ZERO;
                    for &w in fv {
                        c += rest(w) - rest(v);
                    }
                    (c.dot(axis) > 0.0) as usize
                })
                .collect();
            if group.iter().all(|&g| g == 0) || group.iter().all(|&g| g == 1) {
    /// Whether the plane split of `v` given by `fan` leaves at least `MIN_PIECE` triangles
    /// reachable on each side without passing through `v` (no confetti). Always `true` outside
    /// 3D triangle bodies.
    pub(super) fn opens_without_confetti(&self, v: u32, fan: &Fan) -> bool {
        if fan.kind != MeasureKind::Triangles {
            return true;
        }
        let piece_ok = |side: usize| -> bool {
                .filter(|&i| fan.sides[i] == side)
                }
            self.reachable_triangles(&seeds, v, MIN_PIECE) >= MIN_PIECE
        };
        piece_ok(0) && piece_ok(1)
            const MIN_PIECE: usize = 10;
                    .iter()
                    .zip(&group)
                    .map(|(&f, _)| f)
                continue;
            }
            self.split_vertex(v, false, &face_ids, &verts, &group, 2, duplicated);
    /// Number of surface triangles reachable from `seeds` through shared edges not touching
    /// `pivot`, counting up to `limit`.
    fn reachable_triangles(&self, seeds: &[u32], pivot: u32, limit: usize) -> usize {
        // Edge -> triangles map of the whole surface (a tear is a rare event).
        let mut by_edge: HashMap<[u32; 2], Vec<u32>> = HashMap::default();
        for (ti, t) in self.boundary.iter().enumerate() {
            for i in 0..DIM {
                let key = pair(t[i], t[(i + 1) % DIM]);
                if key[0] != pivot && key[1] != pivot {
                    by_edge.entry(key).or_default().push(ti as u32);
        while let Some(ti) = stack.pop() {
            let t = self.boundary[ti as usize];
            for i in 0..DIM {
                let key = pair(t[i], t[(i + 1) % DIM]);
                if let Some(neighbors) = by_edge.get(&key) {
            }
        }
    }

    /// Splits the particle `v` into one copy per group of its faces (`face_ids`, their
    /// vertices in `verts`, `group[i]` in `0..num_groups`): group 0 keeps `v`, the others get a
    /// copy; the mass follows, and so do the other elements around `v` (see `split_particles`).
    #[allow(clippy::too_many_arguments)]
    fn split_vertex(
        use_cells: bool,
        face_ids: &[u32],
        verts: &[Vec<u32>],
        group: &[usize],
        num_groups: usize,
        duplicated: &mut Vec<(u32, u32)>,
    ) {
        // A cloth fan group holding no spring at `v` (all its edges there torn) is a loose bit
        // of fabric: its triangles are dropped instead of a copy of `v` that nothing holds
        // (it would fly off with the triangle it renders).
        if !use_cells {
                for g in 0..num_groups {
                    if has_spring[g] {
                    }
                let mut kept_ids = Vec::new();
                let mut kept_verts = Vec::new();
                let mut kept_group = Vec::new();
                        v,
                        use_cells,
                        &kept_ids,
                        &kept_verts,
                        &kept_group,
                        kept_groups,
                        duplicated,
                    );
                let mut keep =
                return;
            }
        }
        // Mass split by rest volume (cells) or face count.
        let mut weights = vec![0.0; num_groups];
        for (i, &f) in face_ids.iter().enumerate() {
            weights[group[i]] += if use_cells {
                self.cells[f as usize].rest_volume.abs()
            } else {
                1.0
            };
        }
        let new_index = |g: usize, particles: &mut Vec<SoftBodyParticle>| -> u32 {
                source.mass * weights[g] / total
            } else {
                source.mass / num_groups as Real
                let p = &mut particles[v as usize];
                v
                // A split particle is never pinned, so its copies are free like it.
                particles.push(SoftBodyParticle {
                particles.len() as u32 - 1
            }
        };
        let mut copies: Vec<u32> = Vec::with_capacity(num_groups);
        for g in 0..num_groups {
            let idx = new_index(g, &mut self.particles);
            if g > 0 {
                duplicated.push((idx, v));
            }
            copies.push(idx);
        }

        // Reassign the faces of the other groups to their copies.
        for (i, &f) in face_ids.iter().enumerate() {
            let g = group[i];
            let target: &mut [u32] = if use_cells {
                &mut self.cells[f as usize].vertices
            } else {
                &mut self.boundary[f as usize]
            };
            for x in target.iter_mut() {
                if *x == v {
                    *x = copies[g];
                }
            }
        }

        // The other elements at `v`: the group of the faces they share a vertex with, or
        // (unattached) the group whose fan points their way in the rest shape.
        let group_of_element = |vertices: &[u32], this: &SoftBody| -> usize {
            let mut best: Option<usize> = None;
            for (i, fv) in verts.iter().enumerate() {
                if fv.iter().any(|x| *x != v && vertices.contains(x)) {
                    best = Some(best.map_or(group[i], |b: usize| b.min(group[i])));
                }
            }
            if let Some(g) = best {
                return g;
            }
            let rest = |i: u32| this.particles[i as usize].rest_position;
            let mut dir = Vector::ZERO;
            for &w in vertices.iter().filter(|&&w| w != v) {
                dir += rest(w) - rest(v);
            }
            let mut best_score = -Real::MAX;
            let mut best_group = 0;
            for (i, fv) in verts.iter().enumerate() {
                for &w in fv.iter().filter(|&&w| w != v) {
                    let d = rest(w) - rest(v);
                    let score = d.dot(dir) / (d.length() * dir.length()).max(1.0e-12);
                    if score > best_score {
                        best_score = score;
                        best_group = group[i];
                    }
                }
            }
            best_group
        };
            let vertices = self.edges[ei].vertices;
            if !vertices.contains(&v) {
            }
            let x = if vertices[0] == v {
                vertices[1]
            } else {
                vertices[0]
            };
            let mut groups: Vec<usize> = verts
                .iter()
                .enumerate()
                .filter(|(_, fv)| fv.contains(&x))
                .map(|(i, _)| group[i])
                .collect();
            groups.sort_unstable();
            groups.dedup();
            let g = if groups.is_empty() {
                group_of_element(&vertices, self)
            } else {
                groups[0]
            };
            for x in &mut self.edges[ei].vertices {
                if *x == v {
                    *x = copies[g];
                }
            }
                for x in &mut copy.vertices {
                    if *x == copies[g] {
                        *x = copies[other];
                    }
                }
        #[cfg(feature = "dim3")]
        {
            let mut removed = Vec::new();
            for di in 0..self.dihedrals.len() {
                let vertices = self.dihedrals[di].vertices;
                if !vertices.contains(&v) {
                    continue;
                }
                // A dihedral bending across the tear (its two triangles in different
                // fans) is dropped.
                let mut groups: Vec<usize> = Vec::new();
                for (i, fv) in verts.iter().enumerate() {
                    if fv.iter().any(|x| *x != v && vertices.contains(x)) {
                        groups.push(group[i]);
                    }
                }
                groups.sort_unstable();
                groups.dedup();
                if groups.len() > 1 {
                    removed.push(di);
                    continue;
                }
                let g = group_of_element(&vertices, self);
                for x in &mut self.dihedrals[di].vertices {
                    if *x == v {
                        *x = copies[g];
                    }
                }
            }
            for di in removed.into_iter().rev() {
                self.dihedrals.swap_remove(di);
            }
        }
        if use_cells {
            // A surface element follows the cell that owns it (the one holding all its
            // vertices): the fan of a shared vertex alone would let a face bridge two pieces.
            for si in 0..self.boundary.len() {
                let vertices = self.boundary[si];
                if !vertices.contains(&v) {
            }
                let owner = verts
                    .iter()
                    .enumerate()
                    .find(|(_, fv)| vertices.iter().all(|x| fv.contains(x)));
                let g = match owner {
                    Some((i, _)) => group[i],
                    None => group_of_element(&vertices, self),
                };
                for x in &mut self.boundary[si] {
                    if *x == v {
                        *x = copies[g];
                    }
    }
}
}
