//! The self contact detection of one soft collision mesh: tangle signal, self candidates and self-overlap regions.

use crate::alloc_prelude::*;

#[cfg(feature = "dim2")]
use crate::dynamics::soft_body_crossing_tests::segments_cross;
use crate::dynamics::{SoftBody, SoftCollisionMesh};
use crate::geometry::{Collider, ColliderHandle};
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;
use parry::partitioning::BvhWorkspace;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_contacts_classify::{classify_inside_self, project_on_element};
use super::soft_contacts_edge_pass::detect_edges;
use super::soft_contacts_vertex_pass::detect_vertex_pass;
use super::soft_contacts_volume::{VolumeBin, VolumeSide, patch_vertices, volume_bins, volume_split};
use super::{SelfTangles, SoftDetectionCtx, SoftVertexPass, SoftEdgePass};

/// The self contact detection of one soft collision mesh (see
/// [`NarrowPhase::soft_self_contacts`]): its tangle signal, and its self candidates.
#[derive(Clone, Default)]
pub(crate) struct SoftSelfContacts {
    /// The body's currently inverted cells and the particles they touch (empty while every
    /// cell is healthy, and for cell-less bodies): the material-side tangle signal.
    pub inverted_cells: Vec<bool>,
    pub inverted_particles: Vec<bool>,
    /// Elements (and vertices) whose self contacts stand down this step: backed by inverted
    /// cells, or part of a self-crossing of the surface (empty while the surface is healthy).
    /// See [`detect_self_tangles`].
    pub tangled_elements: Vec<bool>,
    pub tangled_vertices: Vec<bool>,
    /// The crossing element pairs the sweep found (capped, sorted).
    pub crossings: Vec<(u32, u32)>,
    /// The mesh's `crossing_sweep_travel` of the next step: reset by a sweep, accumulated otherwise.
    pub crossing_sweep_travel_next: Real,
    /// The mesh's vertices against its own surface.
    pub vertex_pass: SoftVertexPass,
    /// The mesh's edges against its own edges (`None`: no self edge constraints for this mesh).
    pub edges: Option<SoftEdgePass>,
    /// The bins of the self-overlaps between distinct regions of the closed surface (see
    /// `overlap_self_regions`), one entry per region constraint, in emission order.
    pub region_bins: Vec<Vec<VolumeBin>>,
    /// Workspace of the self-crossing sweep's tree traversal.
    bvh_workspace: BvhWorkspace,
}

impl SoftSelfContacts {
    /// The mesh's tangle signal, as the self passes read it.
    pub fn tangles(&self) -> SelfTangles<'_> {
        SelfTangles {
            tangled_elements: &self.tangled_elements,
            tangled_vertices: &self.tangled_vertices,
            crossings: &self.crossings,
        }
    }
}

/// Marks the elements (and vertices) of `mesh` whose self contacts stand down this step: features
/// backed by inverted cells or taking part in a self-crossing of the surface, where proximity
/// constraints would freeze the tangle that elasticity resolves; both signals are memoryless.
fn detect_self_tangles(
    sb: &SoftBody,
    mesh: &SoftCollisionMesh,
    surface_co: &Collider,
    run_crossing_sweep: bool,
    out: &mut SoftSelfContacts,
) {
    out.tangled_elements.clear();
    out.tangled_vertices.clear();
    out.crossings.clear();
    let num_elements = mesh.indices().len();
    let num_vertices = mesh.vertex_count();
    // The tables are allocated on the first flag: the healthy path stays free.
    fn mark_element(
        tangled: (&mut Vec<bool>, &mut Vec<bool>),
        sizes: (usize, usize),
        element: &[u32],
        e: usize,
    ) {
        if tangled.0.is_empty() {
            tangled.0.resize(sizes.0, false);
            tangled.1.resize(sizes.1, false);
        }
        tangled.0[e] = true;
        for &v in element {
            tangled.1[v as usize] = true;
        }
    }

    // Inverted material: elements whose backing cell is inverted, and vertices held by
    // any inverted cell (a boundary vertex pushed through its own surface sits in one).
    if !out.inverted_particles.is_empty() {
        for (e, &cell) in mesh.element_cell_ids(sb).iter().enumerate() {
            if out
                .inverted_cells
                .get(cell as usize)
                .copied()
                .unwrap_or(false)
            {
                mark_element(
                    (&mut out.tangled_elements, &mut out.tangled_vertices),
                    (num_elements, num_vertices),
                    mesh.element(e),
                    e,
                );
            }
        }
        for v in 0..num_vertices {
            let (anchors, weights) = mesh.vertex_anchors(sb, v);
            if anchors
                .iter()
                .zip(&weights)
                .any(|(&p, &w)| p != u32::MAX && w > 0.01 && out.inverted_particles[p as usize])
            {
                if out.tangled_vertices.is_empty() {
                    out.tangled_elements.resize(num_elements, false);
                    out.tangled_vertices.resize(num_vertices, false);
                }
                out.tangled_vertices[v] = true;
            }
        }
    }

    if !run_crossing_sweep {
        return;
    }
    // Surface self-crossings, from one traversal of the surface's tree against itself
    // (the leaves are element ids, in the shape's local space on both sides, so no pose
    // is involved).
    #[cfg(feature = "dim3")]
    if mesh.is_wire() {
        // Two wire curves cross with measure zero: no instantaneous predicate can see
        // their tangles (left to CCD prevention).
        return;
    }
    let Some(bvh) = surface_co.shape().as_composite_shape().map(|c| c.bvh()) else {
        return;
    };
    // The predicates read the shape's own vertex buffer: the exact positions the tree's
    // leaves were refit on, contiguous, and all in one frame (crossings are
    // frame-invariant, so the shape's local space is fine).
    #[cfg(feature = "dim2")]
    let shape_vertices = surface_co.shape().as_polyline().map(|p| p.vertices());
    #[cfg(feature = "dim3")]
    let shape_vertices = surface_co.shape().as_trimesh().map(|t| t.vertices());
    let Some(shape_vertices) = shape_vertices else {
        return;
    };
    if shape_vertices.len() < num_vertices {
        return;
    }
    // Every leaf pair of the tree against itself is tested for a crossing (see `test_crossing`)
    // into a sink, applied afterwards. A large surface runs in parallel: pairs and chunk sinks
    // keep the sequential order, so the records (and the cap) are the same.
    let mut sink = TangleSink::default();
    #[cfg(feature = "parallel")]
    let parallel = num_elements >= 4096 && rayon::current_num_threads() > 1;
    #[cfg(not(feature = "parallel"))]
    let parallel = false;
    if parallel {
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            let pairs = bvh.traverse_bvtt_single_tree_parallel::<false>();
            let sinks: Vec<TangleSink> = pairs
                .par_chunks(2048)
                .map(|chunk| {
                    let mut sink = TangleSink::default();
                    for &(i, j) in chunk {
                        test_crossing(mesh, shape_vertices, i as usize, j as usize, &mut sink);
                    }
                    sink
                })
                .collect();
            for chunk in sinks {
                sink.merge(chunk);
            }
        }
    } else {
        let SoftSelfContacts { bvh_workspace, .. } = &mut *out;
        bvh.traverse_bvtt_single_tree::<false>(bvh_workspace, &mut |i, j| {
            test_crossing(mesh, shape_vertices, i as usize, j as usize, &mut sink);
        });
    }
    let SoftSelfContacts {
        tangled_elements,
        tangled_vertices,
        crossings,
        ..
    } = out;
    for &e in &sink.marks {
        mark_element(
            (tangled_elements, tangled_vertices),
            (num_elements, num_vertices),
            mesh.element(e as usize),
            e as usize,
        );
    }
    *crossings = sink.crossings;
    // Deterministic untangler input whatever order the traversal visited pairs in.
    crossings.sort_unstable();
    crossings.dedup();
}

/// What the self-crossing sweep found: the elements to flag as tangled, and the crossing
/// element pairs, deduplicated in visitation order and capped.
#[derive(Default)]
struct TangleSink {
    marks: Vec<u32>,
    crossings: Vec<(u32, u32)>,
}

impl TangleSink {
    /// The cap on recorded crossings: room for several simultaneous tangles (a tight cap
    /// starves later tangles of any recorded crossings, collapsing the cluster analysis).
    #[cfg(feature = "dim3")]
    const MAX_CROSSINGS: usize = 256;
    #[cfg(feature = "dim2")]
    const MAX_CROSSINGS: usize = 64;

    #[inline]
    fn record(&mut self, pair: (u32, u32)) {
        if self.crossings.len() < Self::MAX_CROSSINGS && !self.crossings.contains(&pair) {
            self.crossings.push(pair);
        }
    }

    /// Appends a later chunk's findings (its records after this sink's, same rule).
    fn merge(&mut self, other: TangleSink) {
        self.marks.extend(other.marks);
        for pair in other.crossings {
            self.record(pair);
        }
    }
}

/// The crossing test of one leaf pair of the sweep (see `detect_self_tangles`).
#[inline]
fn test_crossing(
    mesh: &SoftCollisionMesh,
    shape_vertices: &[Vector],
    i: usize,
    j: usize,
    sink: &mut TangleSink,
) {
    if i == j {
        return;
    }
    let (ei, ej) = (mesh.element(i), mesh.element(j));
    let p = |v: u32| shape_vertices[v as usize];
    #[cfg(feature = "dim2")]
    {
        // Adjacent segments only meet at their shared vertex: never a crossing.
        if ei.iter().any(|v| ej.contains(v)) {
            return;
        }
        if segments_cross([p(ei[0]), p(ei[1])], [p(ej[0]), p(ej[1])]) {
            sink.marks.push(i as u32);
            sink.marks.push(j as u32);
            sink.record((i.min(j) as u32, i.max(j) as u32));
        }
    }
    #[cfg(feature = "dim3")]
    {
        // Triangles sharing an edge have no testable edge (a touching edge is adjacent, never a
        // transversal pierce): most visited pairs die on this index-only test before reading any
        // position.
        if ei.iter().filter(|v| ej.contains(v)).count() >= 2 {
            return;
        }
        // Each direction: the edges of one triangle against the other, skipping those touching
        // it; what is left (a resting fold, a ring neighbor) mostly dies on one plane test. A hit
        // flags the pierced triangle and those sharing the piercing edge, and records the pair.
        let mut pierce = |ea: &[u32], eb: &[u32], a: usize, b: usize| {
            let tb = [p(eb[0]), p(eb[1]), p(eb[2])];
            let n = (tb[1] - tb[0]).cross(tb[2] - tb[0]);
            let nn = n.length_squared();
            if nn < 1.0e-20 {
                return;
            }
            // Plane offsets scale as `|n| * distance`; the band is a distance of
            // 1.0e-4 triangle scales (the scale is about `sqrt(|n|)`), so an edge
            // grazing the plane (a resting fold) never reads as a pierce.
            let len_n = nn.sqrt();
            let band = 1.0e-4 * len_n * len_n.sqrt();
            let pa = [p(ea[0]), p(ea[1]), p(ea[2])];
            let dp = [
                (pa[0] - tb[0]).dot(n),
                (pa[1] - tb[0]).dot(n),
                (pa[2] - tb[0]).dot(n),
            ];
            if dp.iter().all(|d| *d >= -band) || dp.iter().all(|d| *d <= band) {
                return;
            }
            for k in 0..3 {
                let k1 = (k + 1) % 3;
                if dp[k] * dp[k1] >= 0.0 {
                    continue;
                }
                let (v0, v1) = (ea[k], ea[k1]);
                if eb.contains(&v0) || eb.contains(&v1) {
                    continue;
                }
                let (q0, q1) = (pa[k], pa[k1]);
                let pq = q1 - q0;
                let d = [
                    pq.dot((tb[0] - q0).cross(tb[1] - q0)),
                    pq.dot((tb[1] - q0).cross(tb[2] - q0)),
                    pq.dot((tb[2] - q0).cross(tb[0] - q0)),
                ];
                if !d.iter().all(|x| *x >= 0.0) && !d.iter().all(|x| *x <= 0.0) {
                    continue;
                }
                sink.marks.push(b as u32);
                // The pierce fires once per piercing edge and the traversal revisits
                // pairs: recorded deduplicated.
                sink.record((a.min(b) as u32, a.max(b) as u32));
                for &incident in mesh.vertex_element_ids(v0) {
                    if mesh.element(incident as usize).contains(&v1) {
                        sink.marks.push(incident);
                    }
                }
            }
        };
        pierce(ei, ej, i, j);
        pierce(ej, ei, j, i);
    }
}

/// The self contact detection of one mesh with self contacts enabled, on a body with free
/// particles: the tangle signal, then the vertex and edge candidates against itself.
pub(super) fn update_self(
    out: &mut SoftSelfContacts,
    sb: &SoftBody,
    mesh: &SoftCollisionMesh,
    handle: ColliderHandle,
    co: &Collider,
    ctx: &SoftDetectionCtx,
) {
    let recovery = &ctx.params.soft_bodies.recovery;
    // The material-side tangle signal: the cells currently inverted (negative volume
    // relative to rest) and the particles they touch. Left empty while every cell is
    // healthy, so the healthy path allocates and scans nothing downstream.
    out.inverted_cells.clear();
    out.inverted_particles.clear();
    if recovery.inverted_cell_detection && !sb.cells.is_empty() {
        for (ci, cell) in sb.cells.iter().enumerate() {
            let x: [Vector; DIM + 1] =
                core::array::from_fn(|k| sb.particles[cell.vertices[k] as usize].position);
            if SoftBody::cell_volume(x) * cell.rest_volume <= 0.0 {
                if out.inverted_cells.is_empty() {
                    out.inverted_cells.resize(sb.cells.len(), false);
                    out.inverted_particles.resize(sb.particles.len(), false);
                }
                out.inverted_cells[ci] = true;
                for &p in &cell.vertices {
                    out.inverted_particles[p as usize] = true;
                }
            }
        }
    }
    // The self-crossing sweep (the expensive half) runs when the surface's accumulated travel
    // since the last sweep could have bridged half a skin, and every step while anything is
    // flagged; the cheap inverted-cell scan always runs.
    let was_tangled = mesh.crossed_partners.contains(&handle);
    let travel = mesh.crossing_sweep_travel + ctx.motion_margin(co);
    let run_crossing_sweep = recovery.self_crossing_detection
        && (was_tangled || !recovery.detection_motion_gating || travel > 0.5 * co.contact_skin());
    out.crossing_sweep_travel_next = if run_crossing_sweep { 0.0 } else { travel };
    detect_self_tangles(sb, mesh, co, run_crossing_sweep, out);
    let side = (sb, mesh, handle, co);
    let mut vertex_pass = core::mem::take(&mut out.vertex_pass);
    detect_vertex_pass(&mut vertex_pass, side, side, Some(out.tangles()), false, ctx);
    out.vertex_pass = vertex_pass;
    let mut edges = out.edges.take().unwrap_or_default();
    let has_edges = detect_edges(&mut edges, side, side, Some(out.tangles()), false, ctx);
    out.edges = has_edges.then_some(edges);
    detect_self_regions(out, sb, mesh, co, ctx);
}

/// Self-overlaps between distinct regions of one closed surface (see `overlap_self_regions`):
/// vertices behind another part of their own surface, grouped into connected regions, each binned
/// with the vertices of the elements it faces, like a pair of bodies would.
fn detect_self_regions(
    out: &mut SoftSelfContacts,
    sb: &SoftBody,
    mesh: &SoftCollisionMesh,
    co: &Collider,
    ctx: &SoftDetectionCtx,
) {
    let params = ctx.params;
    let recovery = &params.soft_bodies.recovery;
    out.region_bins.clear();
    if !recovery.overlap_constraints
        || !recovery.overlap_self_regions
        || !mesh.is_closed()
        || mesh.orientation_unreliable
        || out.crossings.is_empty()
    {
        return;
    }
    let n_v = mesh.vertex_count();
    let n_e = mesh.indices().len();
    let mut crossing = vec![false; n_e];
    for &(i, j) in &out.crossings {
        crossing[i as usize] = true;
        crossing[j as usize] = true;
    }
    let mut vertex_elements: Vec<Vec<u32>> = vec![Vec::new(); n_v];
    for (e, el) in mesh.indices().iter().enumerate() {
        for &v in el {
            vertex_elements[v as usize].push(e as u32);
        }
    }
    let mut inside = Vec::new();
    if !classify_inside_self(mesh, sb, co.contact_skin(), &crossing, &vertex_elements, &mut inside) {
        return;
    }
    // Each inside vertex's facing element (the nearest one outside its neighborhood) and
    // distance; a vertex in front of the surface it faces is a mirrored lobe's, not a
    // region's.
    let mut facing: Vec<Option<(usize, Real)>> = vec![None; n_v];
    for v in 0..n_v {
        if !inside[v] {
            continue;
        }
        let p = mesh.vertex(sb, v);
        let ring = &mesh.ring[mesh.ring_offsets[v] as usize..mesh.ring_offsets[v + 1] as usize];
        let mut best: Option<(Real, usize, Vector)> = None;
        for e in 0..n_e {
            let el = mesh.element(e);
            if el.iter().any(|&u| u as usize == v || ring.contains(&u)) {
                continue;
            }
            let Some((q, _)) = project_on_element(mesh, sb, e, p) else {
                continue;
            };
            let d2 = (q - p).length_squared();
            if best.is_none_or(|b| d2 < b.0) {
                best = Some((d2, e, q));
            }
        }
        match best {
            Some((d2, e, q))
                if mesh
                    .element_outward_normal(sb, e)
                    .is_some_and(|n| (p - q).gdot(n) < 0.0) =>
            {
                facing[v] = Some((e, d2.sqrt()));
            }
            _ => inside[v] = false,
        }
    }
    // The regions: the inside vertices flooded through their rings.
    let mut parent: Vec<u32> = (0..n_v as u32).collect();
    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            parent[i as usize] = parent[parent[i as usize] as usize];
            i = parent[i as usize];
        }
        i
    }
    for v in 0..n_v {
        if !inside[v] {
            continue;
        }
        for &u in &mesh.ring[mesh.ring_offsets[v] as usize..mesh.ring_offsets[v + 1] as usize] {
            if inside[u as usize] {
                let (a, b) = (find(&mut parent, v as u32), find(&mut parent, u));
                if a != b {
                    parent[a as usize] = b;
                }
            }
        }
    }
    let region: Vec<u32> = (0..n_v as u32)
        .map(|v| {
            if inside[v as usize] {
                find(&mut parent, v)
            } else {
                u32::MAX
            }
        })
        .collect();
    let mut roots: Vec<u32> = region.iter().copied().filter(|&r| r != u32::MAX).collect();
    roots.sort_unstable();
    roots.dedup();
    // One constraint per region, against the vertices of the elements it faces; a pair of
    // regions facing each other is emitted once, by the lower root.
    let mut emitted: Vec<(u32, u32)> = Vec::new();
    for &root in &roots {
        let mut depth_own = vec![Real::NEG_INFINITY; n_v];
        let mut depth_other = vec![Real::NEG_INFINITY; n_v];
        let mut faced_roots: Vec<u32> = Vec::new();
        for v in 0..n_v {
            if region[v] != root {
                continue;
            }
            let Some((e, d)) = facing[v] else {
                continue;
            };
            depth_own[v] = d;
            for &u in mesh.element(e) {
                let u = u as usize;
                if region[u] == root {
                    continue;
                }
                depth_other[u] = facing[u].map_or(0.0, |f| f.1);
                if region[u] != u32::MAX && !faced_roots.contains(&region[u]) {
                    faced_roots.push(region[u]);
                }
            }
        }
        if faced_roots
            .iter()
            .any(|&r| emitted.contains(&(r.min(root), r.max(root))))
        {
            continue;
        }
        for &r in &faced_roots {
            emitted.push((r.min(root), r.max(root)));
        }
        let own = VolumeSide {
            body: sb,
            mesh,
            vertices: patch_vertices(mesh, sb, &depth_own, 0.0),
        };
        let other = VolumeSide {
            body: sb,
            mesh,
            vertices: patch_vertices(mesh, sb, &depth_other, 0.0),
        };
        if own.vertices.is_empty() {
            continue;
        }
        out.region_bins
            .push(volume_bins(&own, Some(&other), volume_split(params)));
    }
}

