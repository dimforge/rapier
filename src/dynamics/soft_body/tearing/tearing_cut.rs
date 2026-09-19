//! Cuts of soft bodies along a blade: cells and triangles snap the cut to their facets by
//! splitting particles, segments get two particles inserted at the crossing.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};

use super::super::{SoftBody, SoftBodyEdgeKind};
use super::tearing_event::SoftBodyTearEvent;
use super::tearing_particle_split::{MeasureKind, SplitLog};

/// The shortest part of a segment, as a fraction of its length, a cut inserts particles for: a
/// crossing closer to an endpoint splits that endpoint instead.
const MIN_INSERTION_FRACTION: Real = 0.2;

/// The normal of the blade's supporting line (2D) or plane (3D); zero for a degenerate blade.
fn blade_normal(blade: &[Vector; DIM]) -> Vector {
    #[cfg(feature = "dim2")]
    {
        let d = blade[1] - blade[0];
        Vector::new(-d.y, d.x)
    }
    #[cfg(feature = "dim3")]
    {
        (blade[1] - blade[0]).cross(blade[2] - blade[0])
    }
}

/// Where segment `[p, q]` meets the blade, as `t` in `p + (q - p) * t`: the endpoints lie on or
/// across the blade's supporting line (2D) or plane (3D) and the meeting point lies on the blade
/// (closed tests). `None` for a segment within the line or plane, or a degenerate blade.
pub(super) fn blade_hit(blade: &[Vector; DIM], p: Vector, q: Vector) -> Option<Real> {
    let n = blade_normal(blade);
    let (dp, dq) = (n.dot(p - blade[0]), n.dot(q - blade[0]));
    if dp * dq > 0.0 || dp == dq {
        return None;
    }
    let t = (dp / (dp - dq)).clamp(0.0, 1.0);
    let hit = p + (q - p) * t;
    #[cfg(feature = "dim2")]
    {
        let d = blade[1] - blade[0];
        let s = (hit - blade[0]).dot(d);
        (s >= 0.0 && s <= d.length_squared()).then_some(t)
    }
    #[cfg(feature = "dim3")]
    {
        let [a, b, c] = *blade;
        let inside = (b - a).cross(hit - a).dot(n) >= 0.0
            && (c - b).cross(hit - b).dot(n) >= 0.0
            && (a - c).cross(hit - c).dot(n) >= 0.0;
        inside.then_some(t)
    }
}

impl SoftBody {
    /// Cuts this body along the blade (see [`crate::dynamics::SoftBodySet::cut`]). Returns whether
    /// the topology changed, recording in `event` the crossed elements (particle indices before
    /// the cut), the removed straddlers, the split particles and the inserted ones.
    pub(crate) fn cut_topology(
        &mut self,
        blade: &[Vector; DIM],
        event: &mut SoftBodyTearEvent,
    ) -> bool {
        let Some(kind) = self.measure_kind() else {
            return false;
        };
        let (edges, cells) = self.crossing_elements(blade);
        if edges.is_empty() && cells.is_empty() {
            return false;
        }
        let crossed_edges: Vec<[u32; 2]> = edges
            .iter()
            .map(|&e| self.edges[e as usize].vertices)
            .collect();
        let crossed_cells: Vec<[u32; DIM + 1]> = cells
            .iter()
            .map(|&c| self.cells[c as usize].vertices)
            .collect();

        let mut log = SplitLog::default();
        if kind == MeasureKind::Segments {
            self.cut_segments(blade, &mut log);
        } else {
            self.cut_facets(kind, blade, &mut log);
        }
        if log.split_particles.is_empty() && log.inserted.is_empty() {
            return false;
        }
        // The cut passed through these particles: the elements around them lose their interior
        // strength.
        for &(copy, source) in &log.split_particles {
            self.particles[copy as usize].damaged = true;
            self.particles[source as usize].damaged = true;
        }
        for inserted in &log.inserted {
            for &v in inserted {
                self.particles[v as usize].damaged = true;
            }
        }

        self.remove_straddlers_across_pieces(&mut log);
        event.torn_edges.extend(crossed_edges);
        event.torn_cells.extend(crossed_cells);
        event.removed_edges.extend_from_slice(&log.removed_edges);
        event
            .split_particles
            .extend_from_slice(&log.split_particles);
        event
            .inserted_particles
            .extend(log.inserted.iter().flat_map(|&[_, _, p, q]| [p, q]));
        self.finish_topology_change(&log);
        true
    }

    /// Splits `v` by the blade's supporting line or plane, each element of its fan going to the
    /// side of its current centroid; the particle itself stays with the elements on its own side
    /// (the first element's side when it lies on the blade). Returns whether it split.
    fn split_by_blade(
        &mut self,
        kind: MeasureKind,
        v: u32,
        blade: &[Vector; DIM],
        log: &mut SplitLog,
    ) -> bool {
        let mut fan = self.fan(kind, v);
        if fan.elements.len() < 2 {
            return false;
        }
        let normal = blade_normal(blade);
        let offset = |point: Vector| normal.dot(point - blade[0]);
        let raw: Vec<bool> = fan
            .vertices
            .iter()
            .map(|vertices| {
                let sum: Vector = vertices
                    .iter()
                    .map(|&w| self.particles[w as usize].position)
                    .sum();
                offset(sum / vertices.len() as Real) > 0.0
            })
            .collect();
        let own = offset(self.particles[v as usize].position);
        let kept = if own != 0.0 { own > 0.0 } else { raw[0] };
        fan.sides = raw.iter().map(|&r| (r != kept) as usize).collect();
        fan.assign_groups(v);
        self.split_fan(v, &fan, log)
    }

    /// Cuts a body of cells or triangles: every particle ending an element edge the blade meets is
    /// split by the blade ([`Self::split_by_blade`]), so crossed elements go whole to one side and
    /// the cut follows their facets; particles then left joining two pieces alone are split.
    fn cut_facets(&mut self, kind: MeasureKind, blade: &[Vector; DIM], log: &mut SplitLog) {
        let mut candidate = vec![false; self.particles.len()];
        self.for_each_measure_element(kind, |_, vertices| {
            for i in 0..vertices.len() {
                for j in i + 1..vertices.len() {
                    let (x, y) = (vertices[i] as usize, vertices[j] as usize);
                    let (px, py) = (self.particles[x].position, self.particles[y].position);
                    if blade_hit(blade, px, py).is_some() {
                        candidate[x] = true;
                        candidate[y] = true;
                    }
                }
            }
        });
        for v in 0..candidate.len() {
            if candidate[v] {
                self.split_by_blade(kind, v as u32, blade, log);
            }
        }
        if !log.split_particles.is_empty() {
            self.split_pinched(kind, log);
        }
    }

    /// Cuts a body of segments: a structural edge crossed away from its endpoints gets two
    /// particles inserted at the crossing ([`Self::insert_into_segment`]); one crossed near or at
    /// an endpoint splits it instead, unless pinned (never split), which inserts the crossing.
    fn cut_segments(&mut self, blade: &[Vector; DIM], log: &mut SplitLog) {
        let crossings: Vec<(usize, Real)> = self
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| e.kind == SoftBodyEdgeKind::Structural)
            .filter_map(|(i, e)| {
                let [a, b] = e.vertices;
                let (pa, pb) = (
                    self.particles[a as usize].position,
                    self.particles[b as usize].position,
                );
                blade_hit(blade, pa, pb).map(|t| (i, t))
            })
            .collect();
        // Insertions rewrite edges in place or append them: the crossing indices stay valid.
        let mut straddling = Vec::new();
        let mut snapped = Vec::new();
        for (ei, t) in crossings {
            let [a, b] = self.edges[ei].vertices;
            let near = if t < 0.5 { a } else { b };
            let force = self.particles[near as usize].inv_mass == 0.0;
            if !self.insert_into_segment(ei, t, force, &mut straddling, log) {
                snapped.push(near);
            }
        }
        if !straddling.is_empty() {
            straddling.sort_unstable();
            straddling.dedup();
            let mut removed = vec![false; self.edges.len()];
            for ei in straddling {
                removed[ei] = true;
                log.removed_edges.push(self.edges[ei].vertices);
            }
            let mut keep = removed.iter().map(|r| !r);
            self.edges.retain(|_| keep.next().unwrap());
        }
        snapped.sort_unstable();
        snapped.dedup();
        for v in snapped {
            let first = log.split_particles.len();
            if self.split_by_blade(MeasureKind::Segments, v, blade, log) {
                // The segments moved to a copy no longer bear their load: their warm start and
                // stress would pull the fresh copy.
                let copies: Vec<u32> = log.split_particles[first..]
                    .iter()
                    .map(|(c, _)| *c)
                    .collect();
                for e in &mut self.edges {
                    if e.vertices.iter().any(|w| copies.contains(w)) {
                        e.impulse = 0.0;
                        e.stress = 0.0;
                    }
                }
            }
        }
    }

    /// Inserts `p`, `q` where the blade crosses structural edge `ei` at `t`: the edge becomes
    /// `(a, p)`, `(q, b)` (proportional rest lengths and mass, surface segment too) and spanning
    /// edges go to `straddling`. `false` for a massless segment or, unless `force`, an endpoint.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // A NaN measure must be rejected too.
    fn insert_into_segment(
        &mut self,
        ei: usize,
        t: Real,
        force: bool,
        straddling: &mut Vec<usize>,
        log: &mut SplitLog,
    ) -> bool {
        let range = MIN_INSERTION_FRACTION..=1.0 - MIN_INSERTION_FRACTION;
        if !force && !range.contains(&t) {
            return false;
        }
        let t = t.clamp(*range.start(), *range.end());
        let edge = self.edges[ei];
        let [a, b] = edge.vertices;
        let length = edge.initial_rest_length();
        let share = |v: u32| {
            let total: Real = self
                .edges
                .iter()
                .filter(|e| e.kind == SoftBodyEdgeKind::Structural && e.vertices.contains(&v))
                .map(|e| e.initial_rest_length())
                .sum();
            if total > 0.0 {
                self.particles[v as usize].mass * length / total
            } else {
                0.0
            }
        };
        let (share_a, share_b) = (share(a), share(b));
        let mass = share_a + share_b;
        if !(mass > 0.0) || !(length > 0.0) {
            return false;
        }

        // The non-structural edges from `a`'s side of the segment to `b`'s: bending edges over it.
        let neighbors = |v: u32, other: u32| -> Vec<u32> {
            let mut side = vec![v];
            for e in &self.edges {
                if e.kind == SoftBodyEdgeKind::Structural && e.vertices.contains(&v) {
                    let w = if e.vertices[0] == v {
                        e.vertices[1]
                    } else {
                        e.vertices[0]
                    };
                    if w != other {
                        side.push(w);
                    }
                }
            }
            side
        };
        let (side_a, side_b) = (neighbors(a, b), neighbors(b, a));
        for (i, e) in self.edges.iter().enumerate() {
            if e.kind == SoftBodyEdgeKind::Structural {
                continue;
            }
            let [x, y] = e.vertices;
            let only =
                |side: &[u32], other: &[u32], w: u32| side.contains(&w) && !other.contains(&w);
            if (only(&side_a, &side_b, x) && only(&side_b, &side_a, y))
                || (only(&side_b, &side_a, x) && only(&side_a, &side_b, y))
            {
                straddling.push(i);
            }
        }

        let (pa, pb) = (self.particles[a as usize], self.particles[b as usize]);
        for (v, share, kept) in [(a, share_a, t), (b, share_b, 1.0 - t)] {
            let p = &mut self.particles[v as usize];
            p.mass -= share - mass * kept * 0.5;
            if p.inv_mass != 0.0 {
                p.inv_mass = crate::utils::inv(p.mass);
            }
        }
        let mut inserted = [0; 2];
        for (k, kept) in [t, 1.0 - t].into_iter().enumerate() {
            let mass = mass * kept * 0.5;
            self.particles.push(super::super::SoftBodyParticle {
                position: pa.position.lerp(pb.position, t),
                velocity: pa.velocity.lerp(pb.velocity, t),
                rest_position: pa.rest_position.lerp(pb.rest_position, t),
                initial_rest_position: pa.initial_rest_position.lerp(pb.initial_rest_position, t),
                mass,
                inv_mass: crate::utils::inv(mass),
                force: Vector::ZERO,
                next_position: None,
                // New material, not a copy: its own root.
                split_root: self.particles.len() as u32,
                ..if k == 0 { pa } else { pb }
            });
            inserted[k] = self.particles.len() as u32 - 1;
        }
        let [p, q] = inserted;

        // Plastic flow keeps its strain: the initial rest lengths scale like the rest lengths.
        let half = |vertices: [u32; 2], fraction: Real| {
            let mut half = edge;
            half.vertices = vertices;
            half.rest_length = edge.rest_length * fraction;
            half.impulse = 0.0;
            half.stress = 0.0;
            half.torn = false;
            half
        };
        self.edges[ei] = half([a, p], t);
        self.edges.push(half([q, b], 1.0 - t));
        #[cfg(feature = "dim2")]
        for si in 0..self.boundary.len() {
            let s = self.boundary[si];
            if s == [a, b] {
                self.boundary[si] = [a, p];
                self.boundary.push([q, b]);
            } else if s == [b, a] {
                self.boundary[si] = [b, q];
                self.boundary.push([p, a]);
            }
        }
        log.inserted.push([a, b, p, q]);
        true
    }
}
