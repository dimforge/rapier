//! Duplication of the particles a tear passes through: the measure elements around a particle
//! (its fan), their split along a plane or into disconnected groups, and the elements that follow
//! the copies.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::soft_body_builder::cell_faces;
use super::super::{SoftBody, SoftBodyEdgeKind, SoftBodyParticle};
use super::tearing::shared_others;

/// The elements carrying a body's rest measure (see [`SoftBody::rest_measure`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum MeasureKind {
    /// The cells (triangles in 2D, tetrahedra in 3D).
    Cells,
    /// The surface triangles of a 3D body without cells (cloth, shells).
    Triangles,
    /// The structural edges (ropes, polylines, polygons, wires, spring networks).
    Segments,
}

impl MeasureKind {
    /// The number of vertices besides `v` two elements have in common when they share a facet
    /// through `v` (segments always do: their facet is `v` itself).
    fn facet_others(self) -> usize {
        match self {
            MeasureKind::Cells => DIM - 1,
            MeasureKind::Triangles => 1,
            MeasureKind::Segments => 0,
        }
    }
}

/// The measure elements containing a particle, and the groups a split of the particle gives them.
#[derive(Clone, Debug)]
pub(super) struct Fan {
    /// The kind of the elements.
    pub kind: MeasureKind,
    /// The element indices: into the cells, the boundary or the edges, by `kind`.
    pub elements: Vec<u32>,
    /// The vertices of each element, as they were when the fan was taken.
    pub vertices: Vec<Vec<u32>>,
    /// The rest measure of each element.
    pub measures: Vec<Real>,
    /// The side of the split plane each element lies on (`0` or `1`; `0` without a plane).
    pub sides: Vec<usize>,
    /// The group of each element in `0..num_groups`: group 0 keeps the particle, every other
    /// group gets a copy.
    pub groups: Vec<usize>,
    /// The number of groups.
    pub num_groups: usize,
}

impl Fan {
    /// Groups the elements into the connected components of their adjacency at `v` (sharing a
    /// facet through `v`) within each side, numbered side 0 first, then by first element.
    pub(super) fn assign_groups(&mut self, v: u32) {
        let n = self.elements.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let needed = self.kind.facet_others();
        for i in 0..n {
            for j in i + 1..n {
                if self.sides[i] == self.sides[j]
                    && shared_others(&self.vertices[i], &self.vertices[j], v) >= needed
                {
                    let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                    if ri != rj {
                        parent[ri.max(rj)] = ri.min(rj);
                    }
                }
            }
        }
        let mut group_of_root = vec![usize::MAX; n];
        self.num_groups = 0;
        for side in 0..2 {
            for i in (0..n).filter(|&i| self.sides[i] == side) {
                let root = find(&mut parent, i);
                if group_of_root[root] == usize::MAX {
                    group_of_root[root] = self.num_groups;
                    self.num_groups += 1;
                }
                self.groups[i] = group_of_root[root];
            }
        }
    }
}

/// What the particle splits of a tear did.
#[derive(Clone, Debug, Default)]
pub(super) struct SplitLog {
    /// The particle copies, as `(copy, source)`, in creation order.
    pub split_particles: Vec<(u32, u32)>,
    /// The particle pairs of the non-measure edges removed for straddling an opened crack.
    pub removed_edges: Vec<[u32; 2]>,
    /// The segments a cut inserted particles into, as `[a, b, p, q]`: the segment `(a, b)` became
    /// `(a, p)` and `(q, b)`.
    pub inserted: Vec<[u32; 4]>,
}

/// Replaces every occurrence of `from` in `vertices` by `to`.
fn replace(vertices: &mut [u32], from: u32, to: u32) {
    for x in vertices.iter_mut() {
        if *x == from {
            *x = to;
        }
    }
}

impl SoftBody {
    /// The kind of this body's measure elements, by priority: its cells, its surface triangles
    /// (3D, without cells), its structural edges. `None` when it has none of them.
    pub(super) fn measure_kind(&self) -> Option<MeasureKind> {
        if !self.cells.is_empty() {
            Some(MeasureKind::Cells)
        } else if DIM == 3 && !self.boundary.is_empty() {
            Some(MeasureKind::Triangles)
        } else if self
            .edges
            .iter()
            .any(|e| e.kind == SoftBodyEdgeKind::Structural)
        {
            Some(MeasureKind::Segments)
        } else {
            None
        }
    }

    /// Calls `f` with the index and the vertices of every measure element of the given kind.
    pub(super) fn for_each_measure_element(
        &self,
        kind: MeasureKind,
        mut f: impl FnMut(u32, &[u32]),
    ) {
        match kind {
            MeasureKind::Cells => {
                for (i, c) in self.cells.iter().enumerate() {
                    f(i as u32, &c.vertices);
                }
            }
            MeasureKind::Triangles => {
                for (i, t) in self.boundary.iter().enumerate() {
                    f(i as u32, t);
                }
            }
            MeasureKind::Segments => {
                for (i, e) in self.edges.iter().enumerate() {
                    if e.kind == SoftBodyEdgeKind::Structural {
                        f(i as u32, &e.vertices);
                    }
                }
            }
        }
    }

    /// The total rest measure of this body's measure elements, by priority: the rest area (2D) or
    /// volume (3D) of its cells, else (3D) of its surface triangles, else the rest length of its
    /// structural edges, before plastic flow; `0.0` without any, and no tear or cut changes it.
    pub fn rest_measure(&self) -> Real {
        let Some(kind) = self.measure_kind() else {
            return 0.0;
        };
        let mut total = 0.0;
        self.for_each_measure_element(kind, |i, _| total += self.element_rest_measure(kind, i));
        total
    }

    /// The rest measure of the measure elements whose vertices all belong to the given sorted
    /// particle set (see [`Self::rest_measure`]): the measure of a piece of the body.
    pub fn rest_measure_of(&self, particles: &[u32]) -> Real {
        let Some(kind) = self.measure_kind() else {
            return 0.0;
        };
        let mut total = 0.0;
        self.for_each_measure_element(kind, |i, vertices| {
            if vertices.iter().all(|v| particles.binary_search(v).is_ok()) {
                total += self.element_rest_measure(kind, i);
            }
        });
        total
    }

    /// The rest-shape distance from `point` to the material of the given sorted particle set: the
    /// closest measure element with all vertices in the set (zero in a cell), else the closest
    /// particle of the set. What a joint reads to follow a split cluster's piece.
    pub(crate) fn rest_distance_to(&self, particles: &[u32], point: Vector) -> Real {
        use parry::query::PointQuery;
        let rest = |v: u32| self.particles[v as usize].rest_position;
        let mut best = Real::MAX;
        if let Some(kind) = self.measure_kind() {
            self.for_each_measure_element(kind, |_, vertices| {
                if !vertices.iter().all(|v| particles.binary_search(v).is_ok()) {
                    return;
                }
                let distance = match vertices.len() {
                    2 => parry::shape::Segment::new(rest(vertices[0]), rest(vertices[1]))
                        .distance_to_local_point(point, true),
                    3 => parry::shape::Triangle::new(
                        rest(vertices[0]),
                        rest(vertices[1]),
                        rest(vertices[2]),
                    )
                    .distance_to_local_point(point, true),
                    #[cfg(feature = "dim3")]
                    4 => parry::shape::Tetrahedron::new(
                        rest(vertices[0]),
                        rest(vertices[1]),
                        rest(vertices[2]),
                        rest(vertices[3]),
                    )
                    .distance_to_local_point(point, true),
                    _ => return,
                };
                best = best.min(distance);
            });
        }
        if best == Real::MAX {
            for &v in particles {
                if let Some(p) = self.particles.get(v as usize) {
                    best = best.min((p.rest_position - point).length());
                }
            }
        }
        best
    }

    /// The rest measure of the `i`-th measure element of the given kind: a cell's rest area or
    /// volume, a triangle's rest area, an edge's rest length before any plastic flow.
    fn element_rest_measure(&self, kind: MeasureKind, i: u32) -> Real {
        match kind {
            MeasureKind::Cells => self.cells[i as usize].rest_volume.abs(),
            MeasureKind::Triangles => {
                #[cfg(feature = "dim3")]
                {
                    let rest = |v: u32| self.particles[v as usize].initial_rest_position;
                    let t = self.boundary[i as usize];
                    (rest(t[1]) - rest(t[0])).cross(rest(t[2]) - rest(t[0])).length() * 0.5
                }
                #[cfg(feature = "dim2")]
                {
                    0.0
                }
            }
            MeasureKind::Segments => self.edges[i as usize].initial_rest_length(),
        }
    }

    /// Whether some measure element of this body contains all the given particles.
    pub(crate) fn measure_element_holds(&self, particles: &[u32]) -> bool {
        let Some(kind) = self.measure_kind() else {
            return false;
        };
        let mut found = false;
        self.for_each_measure_element(kind, |_, vertices| {
            found |= particles.iter().all(|p| vertices.contains(p));
        });
        found
    }

    /// The fan of `v`: the measure elements of the given kind containing it, in one group.
    pub(super) fn fan(&self, kind: MeasureKind, v: u32) -> Fan {
        let mut elements = Vec::new();
        let mut vertices = Vec::new();
        self.for_each_measure_element(kind, |i, element| {
            if element.contains(&v) {
                elements.push(i);
                vertices.push(element.to_vec());
            }
        });
        let n = elements.len();
        Fan {
            kind,
            measures: elements
                .iter()
                .map(|&i| self.element_rest_measure(kind, i))
                .collect(),
            elements,
            vertices,
            sides: vec![0; n],
            groups: vec![0; n],
            num_groups: (n > 0) as usize,
        }
    }

    /// Duplicates every candidate particle whose measure elements no longer form one connected
    /// fan (a crack left the particle joining two pieces alone): one copy per extra group, the
    /// other elements around the particle following the group they attach to.
    pub(super) fn split_particles(&mut self, candidates: &[u32], log: &mut SplitLog) {
        let Some(kind) = self.measure_kind() else {
            return;
        };
        for &v in candidates {
            let mut fan = self.fan(kind, v);
            if fan.elements.len() < 2 {
                continue;
            }
            fan.assign_groups(v);
            if fan.num_groups >= 2 {
                self.split_fan(v, &fan, log);
            }
        }
    }

    /// The fan of `v` split by the plane through `origin` with normal `normal`, in the rest
    /// shape: every element goes to the side of its rest centroid, and the groups are the
    /// connected components within a side. `None` when the fan lies on one side only.
    pub(super) fn plane_fan(&self, v: u32, origin: Vector, normal: Vector) -> Option<Fan> {
        // A pinned particle is never split: the crack must open elsewhere.
        if self.particles[v as usize].inv_mass == 0.0 {
            return None;
        }
        let kind = self.measure_kind()?;
        let mut fan = self.fan(kind, v);
        let rest = |i: u32| self.particles[i as usize].rest_position;
        for (side, vertices) in fan.sides.iter_mut().zip(&fan.vertices) {
            let offset: Vector = vertices.iter().map(|&w| rest(w) - origin).sum();
            *side = (offset.dot(normal) > 0.0) as usize;
        }
        if !fan.sides.contains(&0) || !fan.sides.contains(&1) {
            return None;
        }
        fan.assign_groups(v);
        Some(fan)
    }

    /// The smallest piece, in measure elements of the given kind, a tear may split off this
    /// body: the material's [`SoftBodyMaterial::min_piece`], else the kind's default (see
    /// [`default_min_piece`]).
    pub(super) fn min_piece(&self, kind: MeasureKind) -> usize {
        self.material
            .min_piece
            .map_or(default_min_piece(kind), |n| n.max(1) as usize)
    }

    /// Whether the split of `v` given by `fan` leaves every group at least [`Self::min_piece`]
    /// measure elements reachable without passing through `v` (no chips). Tears consult it, cuts
    /// do not.
    pub(super) fn opens_without_confetti(&self, v: u32, fan: &Fan) -> bool {
        let min = self.min_piece(fan.kind);
        (0..fan.num_groups).all(|g| {
            let seeds: Vec<u32> = (0..fan.elements.len())
                .filter(|&i| fan.groups[i] == g)
                .map(|i| fan.elements[i])
                .collect();
            self.reachable_elements(fan.kind, &seeds, v, min) >= min
        })
    }

    /// Number of measure elements of the given kind reachable from `seeds` through shared facets
    /// not containing `pivot`, counting up to `limit`.
    fn reachable_elements(
        &self,
        kind: MeasureKind,
        seeds: &[u32],
        pivot: u32,
        limit: usize,
    ) -> usize {
        // Calls `f` with every facet of an element not containing `pivot`: its vertices but one,
        // sorted and padded with `u32::MAX`.
        fn facets(vertices: &[u32], pivot: u32, mut f: impl FnMut([u32; 3])) {
            for skip in 0..vertices.len() {
                let mut key = [u32::MAX; 3];
                let mut k = 0;
                for (i, &w) in vertices.iter().enumerate() {
                    if i != skip {
                        key[k] = w;
                        k += 1;
                    }
                }
                if !key[..k].contains(&pivot) {
                    key[..k].sort_unstable();
                    f(key);
                }
            }
        }
        // Facet -> elements map of the whole body (a tear is a rare event).
        let mut by_facet: HashMap<[u32; 3], Vec<u32>> = HashMap::default();
        let mut elements: HashMap<u32, Vec<u32>> = HashMap::default();
        self.for_each_measure_element(kind, |i, vertices| {
            elements.insert(i, vertices.to_vec());
            facets(vertices, pivot, |key| by_facet.entry(key).or_default().push(i));
        });
        let mut seen: HashMap<u32, ()> = HashMap::default();
        let mut stack: Vec<u32> = seeds.to_vec();
        for &s in seeds {
            seen.insert(s, ());
        }
        while let Some(e) = stack.pop() {
            if seen.len() >= limit {
                return seen.len();
            }
            facets(&elements[&e], pivot, |key| {
                if let Some(neighbors) = by_facet.get(&key) {
                    for &n in neighbors {
                        if !seen.contains_key(&n) {
                            seen.insert(n, ());
                            stack.push(n);
                        }
                    }
                }
            });
        }
        seen.len()
    }

    /// Splits `v` by the groups of its fan (see [`Self::split_vertex`]); returns whether the
    /// topology changed.
    pub(super) fn split_fan(&mut self, v: u32, fan: &Fan, log: &mut SplitLog) -> bool {
        // A pinned (fixed or kinematic) particle is never split: a clamp keeps its material.
        if fan.num_groups < 2 || self.particles[v as usize].inv_mass == 0.0 {
            return false;
        }
        self.split_vertex(v, fan, log);
        true
    }

    /// The group an element around `v` that holds no measure element follows: the lowest group
    /// of the fan elements it shares a vertex with, or the group whose fan points its way in the
    /// rest shape.
    fn group_toward(&self, v: u32, fan: &Fan, vertices: &[u32]) -> usize {
        let mut best: Option<usize> = None;
        for (i, fv) in fan.vertices.iter().enumerate() {
            if fv.iter().any(|x| *x != v && vertices.contains(x)) {
                best = Some(best.map_or(fan.groups[i], |b| b.min(fan.groups[i])));
            }
        }
        if let Some(g) = best {
            return g;
        }
        let rest = |i: u32| self.particles[i as usize].rest_position;
        let mut dir = Vector::ZERO;
        for &w in vertices.iter().filter(|&&w| w != v) {
            dir += rest(w) - rest(v);
        }
        let mut best_score = -Real::MAX;
        let mut best_group = 0;
        for (i, fv) in fan.vertices.iter().enumerate() {
            for &w in fv.iter().filter(|&&w| w != v) {
                let d = rest(w) - rest(v);
                let score = d.dot(dir) / (d.length() * dir.length()).max(1.0e-12);
                if score > best_score {
                    best_score = score;
                    best_group = fan.groups[i];
                }
            }
        }
        best_group
    }

    /// Whether some measure element of the given kind contains both `x` and `y`.
    fn share_measure_element(&self, kind: MeasureKind, x: u32, y: u32) -> bool {
        let mut found = false;
        self.for_each_measure_element(kind, |_, vertices| {
            found |= vertices.contains(&x) && vertices.contains(&y);
        });
        found
    }

    /// Splits the particle `v` into one particle per group of its fan: group 0 keeps `v`, every
    /// other group gets a copy with its fan elements, and the mass is shared by the groups'
    /// rest measures; the remaining elements at `v` are handled per [`Self::group_toward`].
    fn split_vertex(&mut self, v: u32, fan: &Fan, log: &mut SplitLog) {
        let kind = fan.kind;
        let n = fan.elements.len();
        let mut weights = vec![0.0; fan.num_groups];
        for (&g, &measure) in fan.groups.iter().zip(&fan.measures) {
            weights[g] += measure;
        }
        let total: Real = weights.iter().sum();
        let source = self.particles[v as usize];
        let mut copies: Vec<u32> = Vec::with_capacity(fan.num_groups);
        for (g, weight) in weights.iter().enumerate() {
            let mass = if total > 0.0 {
                source.mass * weight / total
            } else {
                source.mass / fan.num_groups as Real
            };
            if g == 0 {
                let p = &mut self.particles[v as usize];
                p.mass = mass;
                if p.inv_mass != 0.0 {
                    p.inv_mass = crate::utils::inv(mass);
                }
                copies.push(v);
            } else {
                // A split particle is never pinned, so its copies are free like it.
                self.particles.push(SoftBodyParticle {
                    mass,
                    inv_mass: crate::utils::inv(mass),
                    next_position: None,
                    split_root: source.split_root(v),
                    ..source
                });
                let copy = self.particles.len() as u32 - 1;
                log.split_particles.push((copy, v));
                copies.push(copy);
            }
        }

        // The groups of the fan elements holding each neighbor of `v`, sorted.
        let mut ring: HashMap<u32, Vec<usize>> = HashMap::default();
        for (vertices, &g) in fan.vertices.iter().zip(&fan.groups) {
            for &w in vertices.iter().filter(|&&w| w != v) {
                let groups = ring.entry(w).or_default();
                if let Err(pos) = groups.binary_search(&g) {
                    groups.insert(pos, g);
                }
            }
        }
        // The facets through `v` shared by cells of different groups: the crack opens there.
        let mut crack_facets: Vec<(usize, usize)> = Vec::new();
        if kind == MeasureKind::Cells {
            for i in 0..n {
                for j in i + 1..n {
                    if fan.groups[i] != fan.groups[j]
                        && shared_others(&fan.vertices[i], &fan.vertices[j], v) == DIM - 1
                    {
                        crack_facets.push((i, j));
                    }
                }
            }
        }

        for (i, &e) in fan.elements.iter().enumerate() {
            let g = fan.groups[i];
            if g == 0 {
                continue;
            }
            let target: &mut [u32] = match kind {
                MeasureKind::Cells => &mut self.cells[e as usize].vertices,
                MeasureKind::Triangles => &mut self.boundary[e as usize],
                MeasureKind::Segments => &mut self.edges[e as usize].vertices,
            };
            replace(target, v, copies[g]);
        }

        // The other edges at `v`. One shared by fan elements of several groups (the crack ends
        // there) gets a copy per group, so no side stretches freely across the crack.
        let is_measure = |e: &super::super::SoftBodyEdge| {
            kind == MeasureKind::Segments && e.kind == SoftBodyEdgeKind::Structural
        };
        let mut new_edges = Vec::new();
        for ei in 0..self.edges.len() {
            let edge = self.edges[ei];
            if !edge.vertices.contains(&v) || is_measure(&edge) {
                continue;
            }
            let x = if edge.vertices[0] == v {
                edge.vertices[1]
            } else {
                edge.vertices[0]
            };
            let groups = ring.get(&x).map(Vec::as_slice).unwrap_or(&[]);
            let g = match groups.first() {
                Some(&g) => g,
                None => self.group_toward(v, fan, &edge.vertices),
            };
            replace(&mut self.edges[ei].vertices, v, copies[g]);
            for &other in groups.iter().skip(1) {
                let mut copy = self.edges[ei];
                replace(&mut copy.vertices, copies[g], copies[other]);
                copy.impulse = 0.0;
                new_edges.push(copy);
            }
        }
        self.edges.extend(new_edges);

        // Straddlers: a non-measure edge between two neighbors of `v` on disjoint groups, not
        // held together by a measure element (a bend edge over `v`, a quad diagonal).
        let mut straddling = vec![false; self.edges.len()];
        for (ei, e) in self.edges.iter().enumerate() {
            if is_measure(e) || e.vertices.iter().any(|w| copies.contains(w)) {
                continue;
            }
            let [x, y] = e.vertices;
            let (Some(gx), Some(gy)) = (ring.get(&x), ring.get(&y)) else {
                continue;
            };
            if gx.iter().all(|g| gy.binary_search(g).is_err())
                && !self.share_measure_element(kind, x, y)
            {
                straddling[ei] = true;
                log.removed_edges.push(e.vertices);
            }
        }
        if straddling.contains(&true) {
            let mut keep = straddling.iter().map(|s| !s);
            self.edges.retain(|_| keep.next().unwrap());
        }

        #[cfg(feature = "dim3")]
        {
            // The fan element holding each triangle of a dihedral, if any.
            let holder = |t: [u32; 3]| {
                fan.vertices
                    .iter()
                    .position(|fv| t.iter().all(|w| fv.contains(w)))
            };
            let mut removed = vec![false; self.dihedrals.len()];
            for di in 0..self.dihedrals.len() {
                let d = self.dihedrals[di].vertices;
                if !d.contains(&v) {
                    continue;
                }
                let g = match (holder([d[0], d[1], d[2]]), holder([d[0], d[1], d[3]])) {
                    // Bending across the crack.
                    (Some(i), Some(j)) if fan.groups[i] != fan.groups[j] => {
                        removed[di] = true;
                        continue;
                    }
                    (Some(i), _) | (None, Some(i)) => fan.groups[i],
                    (None, None) => self.group_toward(v, fan, &d),
                };
                replace(&mut self.dihedrals[di].vertices, v, copies[g]);
            }
            if removed.contains(&true) {
                let mut keep = removed.iter().map(|r| !r);
                self.dihedrals.retain(|_| keep.next().unwrap());
            }
        }

        if kind != MeasureKind::Triangles {
            // A surface element follows the measure element holding it (its cell, or the segment
            // it is): the fan of a shared vertex alone would let a face bridge two pieces.
            for si in 0..self.boundary.len() {
                let element = self.boundary[si];
                if !element.contains(&v) {
                    continue;
                }
                let holder = fan
                    .vertices
                    .iter()
                    .position(|fv| element.iter().all(|x| fv.contains(x)));
                let g = match holder {
                    Some(i) => fan.groups[i],
                    None => self.group_toward(v, fan, &element),
                };
                replace(&mut self.boundary[si], v, copies[g]);
            }
        }
        for &(i, j) in &crack_facets {
            for k in [i, j] {
                let cell = self.cells[fan.elements[k] as usize].vertices;
                let copy = copies[fan.groups[k]];
                let on_facet = |w: &u32| {
                    *w == copy || (*w != v && fan.vertices[i].contains(w) && fan.vertices[j].contains(w))
                };
                if let Some(face) = cell_faces(cell)
                    .into_iter()
                    .find(|face| face.iter().all(on_facet))
                {
                    self.boundary.push(face);
                }
            }
        }
    }

    /// Removes the non-measure elements (edges, dihedrals, surface elements of a body of cells or
    /// segments) joining pieces that the measure elements alone leave disconnected: the catch-all
    /// for straddlers a split misses. Elements touching a measure-less particle stay.
    pub(super) fn remove_straddlers_across_pieces(&mut self, log: &mut SplitLog) {
        let Some(kind) = self.measure_kind() else {
            return;
        };
        let n = self.particles.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        let mut supported = vec![false; n];
        fn find(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let up = parent[parent[i as usize] as usize];
                parent[i as usize] = up;
                i = up;
            }
            i
        }
        self.for_each_measure_element(kind, |_, vertices| {
            for &w in vertices {
                supported[w as usize] = true;
            }
            for w in vertices.windows(2) {
                let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
                if a != b {
                    parent[a.max(b) as usize] = a.min(b);
                }
            }
        });
        let roots: Vec<u32> = (0..n as u32).map(|i| find(&mut parent, i)).collect();
        let spans = |vertices: &[u32]| {
            vertices.iter().all(|&w| supported[w as usize])
                && vertices
                    .iter()
                    .any(|&w| roots[w as usize] != roots[vertices[0] as usize])
        };

        let removed: Vec<bool> = self
            .edges
            .iter()
            .map(|e| {
                !(kind == MeasureKind::Segments && e.kind == SoftBodyEdgeKind::Structural)
                    && spans(&e.vertices)
            })
            .collect();
        if removed.contains(&true) {
            for (e, _) in self.edges.iter().zip(&removed).filter(|(_, r)| **r) {
                log.removed_edges.push(e.vertices);
            }
            let mut keep = removed.iter().map(|r| !r);
            self.edges.retain(|_| keep.next().unwrap());
        }
        #[cfg(feature = "dim3")]
        self.dihedrals.retain(|d| !spans(&d.vertices));
        if kind != MeasureKind::Triangles {
            self.boundary.retain(|s| !spans(s));
        }
    }
}

/// The smallest piece, in triangles, a tear may split off a 3D cloth or shell.
pub(super) const MIN_PIECE_TRIANGLES: usize = 10;
/// The smallest piece, in cells, a tear may split off: more than a grid quad (2D) or a lattice
/// cube (3D).
pub(super) const MIN_PIECE_CELLS: usize = if DIM == 2 { 3 } else { 6 };
/// The smallest piece, in structural edges, a tear may split off a rope, polyline or wire.
pub(super) const MIN_PIECE_SEGMENTS: usize = 3;

/// The smallest piece, in measure elements of the given kind, a tear may split off by default
/// (see [`SoftBodyMaterial::min_piece`]).
pub(super) fn default_min_piece(kind: MeasureKind) -> usize {
    match kind {
        MeasureKind::Cells => MIN_PIECE_CELLS,
        MeasureKind::Triangles => MIN_PIECE_TRIANGLES,
        MeasureKind::Segments => MIN_PIECE_SEGMENTS,
    }
}
