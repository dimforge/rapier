//! Chunking of the soft contact constraints for the parallel contact stages: constraints are
//! grouped by the pair of bodies they hold and the chunks colored so that one color shares no
//! solver body (one stage per color, uncolored chunks in the serial tail on worker 0).

use crate::alloc_prelude::*;
use parry::utils::hashmap::HashMap;

use super::soft_constraints_set::SoftConstraintsSet;
use super::soft_contact::SoftContact;
use crate::dynamics::IntegrationParameters;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::Vector;

/// Colors tried before a chunk goes to the serial tail.
const MAX_CONTACT_COLORS: usize = 16;
/// A chunk against the world (one soft body, no rigid solver body) holding at least this many
/// constraints is split by particle colors into pieces (see [`PIECE_LEN`]), so a big cloth on the
/// ground does not sit entirely in the serial tail.
const SPLIT_MIN: usize = 128;
/// Constraints per piece of a split chunk (the claim granularity of its parallel stages).
const PIECE_LEN: usize = 32;
/// Particle colors tried before a constraint of a split chunk stays in the serial chunk.
const MAX_PARTICLE_COLORS: usize = 16;

/// The bodies a chunk holds: two soft bodies (`u32::MAX`: none) and a rigid solver slot
/// (`u32::MAX`: none or the world).
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
struct ChunkKey {
    soft: [u32; 2],
    rigid: u32,
}

impl ChunkKey {
    fn of(c: &SoftContact) -> Self {
        let (soft, rigid) = if c.other_body != u32::MAX {
            (
                [
                    c.support_body.min(c.other_body),
                    c.support_body.max(c.other_body),
                ],
                u32::MAX,
            )
        } else if c.element.is_none() && !c.soft_other {
            ([c.support_body, u32::MAX], c.body)
        } else {
            // A soft side without a solver DOF (asleep, or frozen and not in the awake list).
            ([c.support_body, u32::MAX], u32::MAX)
        };
        Self { soft, rigid }
    }
}

/// The solver slots written by the chunks of one color so far.
#[derive(Default)]
struct ColorSlots {
    written: Vec<bool>,
}

impl ColorSlots {
    fn clear(&mut self, num_slots: usize) {
        self.written.clear();
        self.written.resize(num_slots, false);
    }

    /// Whether a chunk writing `slots` shares none of them with the color's chunks.
    fn accepts(&self, mut slots: impl Iterator<Item = u32>) -> bool {
        slots.all(|slot| !self.written[slot as usize])
    }

    fn insert(&mut self, slots: impl Iterator<Item = u32>) {
        for slot in slots {
            self.written[slot as usize] = true;
        }
    }
}

/// The solver slots a contact constraint writes: its anchors, the other side's slot (a rigid
/// body, or a particle of another soft body) or element anchors, and every particle of a FEM
/// side's body (the response spans the body).
fn written_slots(c: &SoftContact) -> impl Iterator<Item = u32> + '_ {
    let anchors = c
        .particles
        .iter()
        .chain(core::iter::once(&c.body))
        .chain(c.element.iter().flat_map(|e| e.particles.iter()))
        .copied()
        .filter(|&slot| slot != u32::MAX);
    let fem = c
        .fem
        .iter()
        .flatten()
        .flat_map(|side| side.first_slot..side.first_slot + side.num_particles);
    anchors.chain(fem)
}

/// Workspace of the contact chunking kept across steps.
#[derive(Default)]
pub(crate) struct ContactChunkWorkspace {
    chunk_of_key: HashMap<ChunkKey, u32>,
    /// Chunk of every constraint of the group.
    constraint_chunk: Vec<u32>,
    /// Per chunk: its key, its row count, its first row (cursor of the counting sort).
    keys: Vec<ChunkKey>,
    counts: Vec<usize>,
    cursors: Vec<u32>,
    color_slots: Vec<ColorSlots>,
    /// Per color: its chunks, in chunk order (the last entry: the uncolored chunks).
    color_chunks: Vec<Vec<usize>>,
    /// The chunks in layout order.
    ordered: Vec<usize>,
    /// The constraints of every chunk (CSR over `chunk_rows`), for the coloring.
    chunk_row_offsets: Vec<u32>,
    chunk_rows: Vec<u32>,
    /// Per solver slot: the particle colors of the split chunk at hand touching it.
    slot_masks: Vec<u16>,
}

impl SoftConstraintsSet {
    /// Builds the chunk layout of every group's contact constraints (see the module doc): the chunks of
    /// the parallel colors first, in color order, then the serial tail. Colors with a single
    /// chunk go to the tail as well (a stage of one chunk is as serial, without the barrier).
    pub(super) fn chunk_contacts(&mut self) {
        self.contact_chunk_constraints.clear();
        self.contact_chunks.clear();
        self.contact_colors.clear();
        let mut workspace = core::mem::take(&mut self.contact_workspace);
        for gi in 0..self.groups.len() {
            let constraints = self.groups[gi].contacts.clone();
            let colors_start = self.contact_colors.len();
            let chunks_start = self.contact_chunks.len();
            if constraints.is_empty() {
                let g = &mut self.groups[gi];
                g.contact_colors = colors_start..colors_start;
                g.contact_serial = chunks_start..chunks_start;
                continue;
            }
            // Chunks by first appearance (deterministic: the constraints are in assembly order); the
            // constraints of a pair are contiguous, so the map is rarely consulted.
            workspace.chunk_of_key.clear();
            workspace.keys.clear();
            workspace.counts.clear();
            workspace.constraint_chunk.clear();
            let mut last: Option<(ChunkKey, u32)> = None;
            for c in &self.contacts[constraints.clone()] {
                let key = ChunkKey::of(c);
                let chunk = match last {
                    Some((k, chunk)) if k == key => chunk,
                    _ => {
                        let next = workspace.keys.len() as u32;
                        let chunk = *workspace.chunk_of_key.entry(key).or_insert(next);
                        if chunk == next {
                            workspace.keys.push(key);
                            workspace.counts.push(0);
                        }
                        last = Some((key, chunk));
                        chunk
                    }
                };
                workspace.counts[chunk as usize] += 1;
                workspace.constraint_chunk.push(chunk);
            }
            // Particle-colored pieces of the large chunks against the world (see `SPLIT_MIN`): a
            // constraint goes to the first particle color none of its particles has; one with a
            // FEM side or beyond the colors stays in the original chunk. Pieces are chunks too.
            let mut num_slots = 0usize;
            for c in &self.contacts[constraints.clone()] {
                for slot in written_slots(c) {
                    num_slots = num_slots.max(slot as usize + 1);
                }
            }
            let plain_chunks = workspace.keys.len();
            for chunk in 0..plain_chunks {
                let key = workspace.keys[chunk];
                if key.rigid != u32::MAX
                    || key.soft[1] != u32::MAX
                    || workspace.counts[chunk] < SPLIT_MIN
                {
                    continue;
                }
                workspace.slot_masks.clear();
                workspace.slot_masks.resize(num_slots, 0);
                // The current piece of every particle color, and its fill.
                let mut pieces = [(usize::MAX, 0usize); MAX_PARTICLE_COLORS];
                for (i, c) in self.contacts[constraints.clone()].iter().enumerate() {
                    if workspace.constraint_chunk[i] as usize != chunk {
                        continue;
                    }
                    if c.fem.iter().any(|side| side.is_some()) {
                        continue;
                    }
                    let mut mask = 0u16;
                    for slot in written_slots(c) {
                        mask |= workspace.slot_masks[slot as usize];
                    }
                    let color = mask.trailing_ones() as usize;
                    if color >= MAX_PARTICLE_COLORS {
                        continue;
                    }
                    for slot in written_slots(c) {
                        workspace.slot_masks[slot as usize] |= 1 << color;
                    }
                    let (piece, fill) = &mut pieces[color];
                    if *fill == PIECE_LEN || *piece == usize::MAX {
                        *piece = workspace.keys.len();
                        *fill = 0;
                        workspace.keys.push(key);
                        workspace.counts.push(0);
                    }
                    *fill += 1;
                    workspace.constraint_chunk[i] = *piece as u32;
                    workspace.counts[*piece] += 1;
                    workspace.counts[chunk] -= 1;
                }
            }
            let num_chunks = workspace.keys.len();

            // The constraints of every chunk (counting sort, stable).
            workspace.chunk_row_offsets.clear();
            workspace.chunk_row_offsets.resize(num_chunks + 1, 0);
            for &chunk in &workspace.constraint_chunk {
                workspace.chunk_row_offsets[chunk as usize + 1] += 1;
            }
            for chunk in 0..num_chunks {
                workspace.chunk_row_offsets[chunk + 1] += workspace.chunk_row_offsets[chunk];
            }
            workspace.cursors.clear();
            workspace
                .cursors
                .extend_from_slice(&workspace.chunk_row_offsets[..num_chunks]);
            workspace.chunk_rows.clear();
            workspace.chunk_rows.resize(constraints.len(), 0);
            for (i, &chunk) in workspace.constraint_chunk.iter().enumerate() {
                let slot = &mut workspace.cursors[chunk as usize];
                workspace.chunk_rows[*slot as usize] = i as u32;
                *slot += 1;
            }

            // Greedy coloring: the first color whose chunks write none of this chunk's solver
            // slots (its rigid slot, its constraints' particles). Chunks of one color are then
            // solved concurrently.
            workspace
                .color_slots
                .resize_with(MAX_CONTACT_COLORS, Default::default);
            for slots in &mut workspace.color_slots {
                slots.clear(num_slots);
            }
            for color_chunks in &mut workspace.color_chunks {
                color_chunks.clear();
            }
            workspace
                .color_chunks
                .resize_with(MAX_CONTACT_COLORS + 1, Default::default);
            for chunk in 0..num_chunks {
                let rows = &workspace.chunk_rows[workspace.chunk_row_offsets[chunk] as usize
                    ..workspace.chunk_row_offsets[chunk + 1] as usize];
                let contacts = &self.contacts[constraints.clone()];
                let slots = || rows.iter().flat_map(|&i| written_slots(&contacts[i as usize]));
                let color = workspace
                    .color_slots
                    .iter()
                    .position(|color| color.accepts(slots()))
                    .unwrap_or(MAX_CONTACT_COLORS);
                if color < MAX_CONTACT_COLORS {
                    workspace.color_slots[color].insert(slots());
                }
                workspace.color_chunks[color].push(chunk);
            }

            // Layout: chunk id -> position, parallel colors (two chunks at least) then the tail
            // (single-chunk colors in color order, then the overflow).
            let ordered = &mut workspace.ordered;
            ordered.clear();
            for color_chunks in &workspace.color_chunks[..MAX_CONTACT_COLORS] {
                if color_chunks.len() < 2 {
                    continue;
                }
                let start = chunks_start + ordered.len();
                ordered.extend_from_slice(color_chunks);
                self.contact_colors
                    .push(start..chunks_start + ordered.len());
            }
            let serial_start = chunks_start + ordered.len();
            for color_chunks in &workspace.color_chunks[..MAX_CONTACT_COLORS] {
                if color_chunks.len() == 1 {
                    ordered.push(color_chunks[0]);
                }
            }
            ordered.extend_from_slice(&workspace.color_chunks[MAX_CONTACT_COLORS]);
            debug_assert_eq!(ordered.len(), num_chunks);

            // Constraint ranges of the chunks in layout order, then the constraints (counting sort, stable).
            let mut cursor = self.contact_chunk_constraints.len();
            workspace.cursors.clear();
            workspace.cursors.resize(num_chunks, 0);
            for &chunk in ordered.iter() {
                let count = workspace.counts[chunk];
                workspace.cursors[chunk] = cursor as u32;
                self.contact_chunks.push(cursor..cursor + count);
                cursor += count;
            }
            self.contact_chunk_constraints.resize(cursor, 0);
            for (i, &chunk) in workspace.constraint_chunk.iter().enumerate() {
                let slot = &mut workspace.cursors[chunk as usize];
                self.contact_chunk_constraints[*slot as usize] = (constraints.start + i) as u32;
                *slot += 1;
            }

            let g = &mut self.groups[gi];
            g.contact_colors = colors_start..self.contact_colors.len();
            g.contact_serial = serial_start..self.contact_chunks.len();
            // Every parallel color must be slot-disjoint: two chunks of a color writing one
            // solver body would race on its velocity.
            #[cfg(debug_assertions)]
            for color in &self.contact_colors[g.contact_colors.clone()] {
                let mut seen = vec![usize::MAX; num_slots];
                for chunk in color.clone() {
                    for &row in &self.contact_chunk_constraints[self.contact_chunks[chunk].clone()] {
                        let c = &self.contacts[row as usize];
                        for slot in written_slots(c) {
                            let prev = core::mem::replace(&mut seen[slot as usize], chunk);
                            if prev != usize::MAX && prev != chunk {
                                let key = |ch: usize| workspace.keys[workspace.ordered[ch - chunks_start]];
                                let (k1, k2) = (key(prev), key(chunk));
                                panic!(
                                    "slot {slot} shared: chunk {prev} (soft {:?} rigid {}) and chunk {chunk} (soft {:?} rigid {}); row body {} element {} soft_other {} fem {:?} particles {:?}",
                                    k1.soft, k1.rigid, k2.soft, k2.rigid, c.body, c.element.is_some(), c.soft_other,
                                    c.fem.iter().map(|f| f.is_some()).collect::<Vec<_>>(), c.particles
                                );
                            }
                        }
                    }
                }
            }
        }
        self.contact_workspace = workspace;
    }

    /// Solves the constraints of one contact chunk: update, optional warm start, one Gauss-Seidel
    /// sweep. Chunks of one color hold pairwise-disjoint solver bodies, so they are solved
    /// concurrently.
    #[inline]
    pub fn solve_contact_chunk(
        &mut self,
        chunk: usize,
        bodies: &mut SolverBodies,
        params: &IntegrationParameters,
        wo_bias: bool,
        update: bool,
        warmstart: bool,
    ) {
        // Split borrows only: chunks of one color are solved by several workers at once, and
        // every one of them reads the shared response pool.
        let Self {
            contacts,
            contact_chunk_constraints,
            contact_chunks,
            fem_responses,
            ..
        } = self;
        let pool: &[Vector] = fem_responses;
        for &constraint in &contact_chunk_constraints[contact_chunks[chunk].clone()] {
            let c = &mut contacts[constraint as usize];
            if wo_bias && c.soft_other && c.dist0 > 0.0 {
                continue;
            }
            if update {
                c.update(bodies, params, wo_bias);
            }
            if warmstart {
                c.warmstart(bodies, pool);
            }
            c.solve(bodies, pool);
        }
    }
}
