//! Tears of soft bodies: a torn edge or cell opens a crack by splitting one of its particles along
//! a plane in the rest shape, so no measure element is ever removed.

use crate::alloc_prelude::*;
use crate::dynamics::solver::symmetric_eigen;
use crate::math::{DIM, Real, Vector};
use parry::utils::hashmap::HashMap;

use super::super::SoftBody;
use super::tearing_event::SoftBodyTearEvent;
use super::tearing_particle_split::{Fan, MeasureKind, SplitLog};
use super::tearing::pair;

/// The mean load of the fan elements on the given side of the split plane.
fn side_load(fan: &Fan, side: usize, load: impl Fn(u32) -> Real) -> Real {
    let (sum, count) = fan
        .elements
        .iter()
        .zip(&fan.sides)
        .filter(|(_, s)| **s == side)
        .fold((0.0, 0usize), |(sum, count), (&e, _)| (sum + load(e), count + 1));
    sum / count.max(1) as Real
}

impl SoftBody {
    /// Opens a crack across every torn edge and cell (see [`SoftBody::tear_edge`]), then splits the
    /// particles left joining two pieces alone. Returns whether the topology changed; `event` gets
    /// the opened elements (pre-tear indices), the removed straddling edges and split particles.
    pub(super) fn crack_body(
        &mut self,
        torn_edges: &[u32],
        torn_cells: &[u32],
        event: &mut SoftBodyTearEvent,
    ) -> bool {
        let edges: Vec<[u32; 2]> = torn_edges
            .iter()
            .filter_map(|&e| self.edges.get(e as usize))
            .map(|e| e.vertices)
            .collect();
        let mut cells: Vec<u32> = torn_cells
            .iter()
            .copied()
            .filter(|&c| (c as usize) < self.cells.len())
            .collect();
        cells.sort_unstable();
        cells.dedup();
        let Some(kind) = self.measure_kind() else {
            return false;
        };
        if edges.is_empty() && cells.is_empty() {
            return false;
        }
        // Splits rewire the cells and triangles in place: their indices stay valid through the
        // whole tear. Edge indices do not (straddlers are removed), so segment loads are read at
        // each crack.
        let loads = self.element_loads(kind);
        let cell_vertices: Vec<[u32; DIM + 1]> = cells
            .iter()
            .map(|&c| self.cells[c as usize].vertices)
            .collect();

        let mut log = SplitLog::default();
        for &[a, b] in &edges {
            let first = log.split_particles.len();
            // An edge an earlier split removed (a straddler) is already open.
            let Some((x, y)) = self.current_pair(a, b, &log) else {
                continue;
            };
            if self.crack_across_edge(x, y, &loads, &mut log) {
                event.torn_edges.push([a, b]);
                self.mark_damaged(&[x, y], &log.split_particles[first..]);
            }
        }
        for (&c, original) in cells.iter().zip(&cell_vertices) {
            let first = log.split_particles.len();
            let vertices = self.cells[c as usize].vertices;
            if self.crack_through_cell(c, &mut log) {
                event.torn_cells.push(*original);
                self.mark_damaged(&vertices, &log.split_particles[first..]);
            }
        }
        if log.split_particles.is_empty() {
            return false;
        }

        self.split_pinched(kind, &mut log);
        self.remove_straddlers_across_pieces(&mut log);
        event.removed_edges.extend_from_slice(&log.removed_edges);
        event
            .split_particles
            .extend_from_slice(&log.split_particles);
        self.finish_topology_change(&log);
        true
    }

    /// Splits the particles an opened facet left joining two pieces alone (a strip one element
    /// thick, a cloth crack reaching the border): those of the elements touching a split in `log`
    /// whose fans are no longer connected. Segments share their facet, so only cells and triangles.
    pub(super) fn split_pinched(&mut self, kind: MeasureKind, log: &mut SplitLog) {
        if kind == MeasureKind::Segments {
            return;
        }
        let mut near = vec![false; self.particles.len()];
        for &(copy, source) in &log.split_particles {
            near[copy as usize] = true;
            near[source as usize] = true;
        }
        let mut candidates: Vec<u32> = Vec::new();
        self.for_each_measure_element(kind, |_, vertices| {
            if vertices.iter().any(|&v| near[v as usize]) {
                candidates.extend_from_slice(vertices);
            }
        });
        candidates.sort_unstable();
        candidates.dedup();
        self.split_particles(&candidates, log);
    }

    /// The load of every cell or triangle (by `kind`; empty for segments, whose load is their own
    /// stress): the largest of its own stress (cells) and the stresses of the edges between its
    /// particles.
    fn element_loads(&self, kind: MeasureKind) -> Vec<Real> {
        let mut edge_stress: HashMap<[u32; 2], Real> = HashMap::default();
        for e in &self.edges {
            let stress = edge_stress
                .entry(pair(e.vertices[0], e.vertices[1]))
                .or_insert(0.0);
            *stress = stress.max(e.stress);
        }
        let load = |own: Real, vertices: &[u32]| {
            let mut load = own;
            for i in 0..vertices.len() {
                for j in i + 1..vertices.len() {
                    if let Some(s) = edge_stress.get(&pair(vertices[i], vertices[j])) {
                        load = load.max(*s);
                    }
                }
            }
            load
        };
        match kind {
            MeasureKind::Cells => self.cells.iter().map(|c| load(c.stress, &c.vertices)).collect(),
            MeasureKind::Triangles => self.boundary.iter().map(|t| load(0.0, t)).collect(),
            MeasureKind::Segments => Vec::new(),
        }
    }

    /// The particles standing for the torn edge `(a, b)` after the splits of `log`: the first
    /// pair of copies (or originals) still sharing a measure element, else the first still
    /// joined by an edge (a bend edge follows its copies). `None` when no such pair is left.
    fn current_pair(&self, a: u32, b: u32, log: &SplitLog) -> Option<(u32, u32)> {
        let family = |root: u32| {
            let mut family = vec![root];
            for &(copy, source) in &log.split_particles {
                if family.contains(&source) {
                    family.push(copy);
                }
            }
            family
        };
        let (fa, fb) = (family(a), family(b));
        for &x in &fa {
            for &y in &fb {
                if self.measure_element_holds(&[x, y]) {
                    return Some((x, y));
                }
            }
        }
        for &x in &fa {
            for &y in &fb {
                if self.edges.iter().any(|e| e.vertices == [x, y] || e.vertices == [y, x]) {
                    return Some((x, y));
                }
            }
        }
        None
    }

    /// Marks the given particles and copies as damaged: the elements around them lose their
    /// interior strength.
    fn mark_damaged(&mut self, vertices: &[u32], copies: &[(u32, u32)]) {
        for &v in vertices.iter().chain(copies.iter().map(|(copy, _)| copy)) {
            self.particles[v as usize].damaged = true;
        }
    }

    /// Opens a crack across the torn edge `(a, b)`: an endpoint (the more loaded far side first) is
    /// split along the rest-shape plane through it normal to the edge, its far side going to the
    /// copy; one with all elements on one side, or leaving a piece below the minimum size, cannot.
    fn crack_across_edge(&mut self, a: u32, b: u32, loads: &[Real], log: &mut SplitLog) -> bool {
        let mut options: Vec<(Real, u32, Fan)> = Vec::new();
        for (v, other) in [(a, b), (b, a)] {
            let origin = self.particles[v as usize].rest_position;
            let normal = self.particles[other as usize].rest_position - origin;
            if let Some(fan) = self.plane_fan(v, origin, normal) {
                let load = side_load(&fan, 1, |e| match fan.kind {
                    MeasureKind::Segments => self.edges[e as usize].stress,
                    _ => loads[e as usize],
                });
                options.push((load, v, fan));
            }
        }
        options.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.1.cmp(&y.1)));
        let Some((_, v, fan)) = options
            .into_iter()
            .find(|(_, v, fan)| self.opens_without_confetti(*v, fan))
        else {
            return false;
        };
        let first = log.split_particles.len();
        if !self.split_fan(v, &fan, log) {
            return false;
        }
        // The torn edge no longer bears its load: its warm start and stress would pull the fresh
        // copy and tear the edge again at its other endpoint next step.
        let other = if v == a { b } else { a };
        let opened: Vec<u32> = core::iter::once(v)
            .chain(log.split_particles[first..].iter().map(|(copy, _)| *copy))
            .collect();
        for e in &mut self.edges {
            if e.vertices.contains(&other) && e.vertices.iter().any(|w| opened.contains(w)) {
                e.impulse = 0.0;
                e.stress = 0.0;
            }
        }
        true
    }

    /// Opens a crack through the torn cell `c`: a particle is split along the plane through it
    /// perpendicular to the cell's rest-frame principal stretch, closest to the parallel plane
    /// through the centroid first; a split leaving a piece below the minimum size is skipped.
    fn crack_through_cell(&mut self, c: u32, log: &mut SplitLog) -> bool {
        let vertices = self.cells[c as usize].vertices;
        let rest: [Vector; DIM + 1] =
            core::array::from_fn(|k| self.particles[vertices[k] as usize].rest_position);
        let current: [Vector; DIM + 1] =
            core::array::from_fn(|k| self.particles[vertices[k] as usize].position);
        let rest_matrix = Self::cell_edge_matrix(rest);
        let det = rest_matrix.determinant();
        if det == 0.0 || !det.is_finite() {
            return false;
        }
        // `F = Ds Dm⁻¹`; the eigenvectors of `FᵀF` are the rest-frame stretch directions.
        let f = Self::cell_edge_matrix(current) * rest_matrix.inverse();
        let (values, vectors) = symmetric_eigen(&(f.transpose() * f));
        let k = (0..DIM)
            .max_by(|&i, &j| values[i].total_cmp(&values[j]))
            .unwrap_or(0);
        let normal = vectors.col(k);
        let centroid = rest.iter().copied().sum::<Vector>() / (DIM + 1) as Real;
        let mut order: Vec<(Real, u32)> = vertices
            .iter()
            .zip(&rest)
            .map(|(&v, &r)| ((r - centroid).dot(normal).abs(), v))
            .collect();
        order.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        for (_, v) in order {
            let origin = self.particles[v as usize].rest_position;
            if let Some(fan) = self.plane_fan(v, origin, normal) {
                if self.opens_without_confetti(v, &fan) && self.split_fan(v, &fan, log) {
                    return true;
                }
            }
        }
        false
    }
}
