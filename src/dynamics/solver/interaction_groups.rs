use crate::alloc_prelude::*;
use crate::dynamics::JointGraphEdge;
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::solver_contact_graph::ContactRef;

use crate::math::{SIMD_LAST_INDEX, SIMD_WIDTH};

use crate::dynamics::RigidBodyHandle;

pub(crate) struct ParallelInteractionGroups {
    bodies_color: Vec<u128>,         // Workspace.
    interaction_indices: Vec<usize>, // Workspace.
    interaction_colors: Vec<usize>,  // Workspace.
    sorted_interactions: Vec<usize>,
    groups: Vec<usize>,
    group_colors: Vec<u8>,
}

impl ParallelInteractionGroups {
    pub fn new() -> Self {
        Self {
            bodies_color: Vec::new(),
            interaction_indices: Vec::new(),
            interaction_colors: Vec::new(),
            sorted_interactions: Vec::new(),
            groups: Vec::new(),
            group_colors: Vec::new(),
        }
    }

    pub fn group(&self, i: usize) -> &[usize] {
        let range = self.groups[i]..self.groups[i + 1];
        &self.sorted_interactions[range]
    }

    /// The color id assigned to the `i`-th group.
    pub fn group_color(&self, i: usize) -> u8 {
        self.group_colors[i]
    }

    pub fn num_groups(&self) -> usize {
        self.groups.len().saturating_sub(1)
    }

    /// Greedy per-color grouping over the joints' stamped solver-body ids (`solver_body_ids`:
    /// `active_set_id` per side, or `u32::MAX` for a world-attached side — fixed, sleeping, or
    /// no body). Never dereferences the rigid-body set. NOTE: multibody-linked interactions
    /// must not be passed here (categorization routes them to the serial generic path): the
    /// coloring would need to remap a multibody's links to one representative body to keep
    /// same-color interactions truly independent.
    #[inline]
    fn keep_or_pick(stored: u8, color_mask: u128, pick: impl FnOnce() -> usize) -> usize {
        if stored < 128 && color_mask & (1u128 << stored) == 0 {
            return stored as usize;
        }
        pick()
    }

    pub fn group_interactions(
        &mut self,
        num_island_bodies: usize,
        interactions: &mut [JointGraphEdge],
        interaction_indices: &[usize],
        // Per-body masks (indexed by rigid-body arena index) of colors already used by another
        // constraint kind (e.g. the contacts' persistent solver colors): colors assigned here
        // avoid them, so same-id groups of both colorings never share a body. Empty = disabled.
        external_color_masks: &[u128],
    ) {
        self.bodies_color.clear();
        self.interaction_indices.clear();
        self.groups.clear();
        self.sorted_interactions.clear();
        self.interaction_colors.clear();
        self.group_colors.clear();

        let external_mask = |handle: RigidBodyHandle| {
            external_color_masks
                .get(handle.into_raw_parts().0 as usize)
                .copied()
                .unwrap_or(0)
        };

        // Color 128 is the "couldn't color" overflow bucket (used when a body's
        // 128-bit color mask is exhausted); consumers must solve it serially.
        let mut color_len = [0; 129];
        self.bodies_color.resize(num_island_bodies, 0u128);
        self.interaction_indices
            .extend_from_slice(interaction_indices);
        self.interaction_colors.resize(interaction_indices.len(), 0);
        let bcolors = &mut self.bodies_color;

        for (interaction_id, color) in self
            .interaction_indices
            .iter()
            .zip(self.interaction_colors.iter_mut())
        {
            // Solver-body ids stamped by the selection pass; `u32::MAX` is a
            // world-attached side (fixed, sleeping, or no body): it doesn't
            // conflict with anything.
            let joint = &interactions[*interaction_id].weight;
            let stored_color = joint.solver_color;
            let [id1, id2] = joint.solver_body_ids;
            let is_fixed1 = id1 == u32::MAX;
            let is_fixed2 = id2 == u32::MAX;
            // Colors used by the external coloring on the pair's bodies (indexed
            // by rigid-body arena index — no arena read); the color chosen below
            // must avoid them.
            let ext_mask = external_mask(joint.body1) | external_mask(joint.body2);

            // Grow the masks to cover appended (frontier) solver-body slots.
            let max_id = if is_fixed1 { 0 } else { id1 as usize }.max(if is_fixed2 {
                0
            } else {
                id2 as usize
            });
            if max_id >= bcolors.len() {
                bcolors.resize(max_id + 1, 0u128);
            }

            match (is_fixed1, is_fixed2) {
                (false, false) => {
                    // Reserve the top colors for dynamic-vs-fixed contacts so those are solved
                    // last (see `SOLVER_DYNAMIC_COLOR_COUNT`); dyn-vs-dyn overflows instead of
                    // encroaching on that band.
                    let color_mask = bcolors[id1 as usize] | bcolors[id2 as usize] | ext_mask;
                    let dynamic_free =
                        !color_mask & ((1u128 << crate::geometry::SOLVER_DYNAMIC_COLOR_COUNT) - 1);
                    *color = Self::keep_or_pick(stored_color, color_mask, || {
                        dynamic_free.trailing_zeros() as usize
                    });
                    color_len[*color] += 1;
                    if *color < 128 {
                        bcolors[id1 as usize] |= 1 << *color;
                        bcolors[id2 as usize] |= 1 << *color;
                    }
                }
                (true, false) | (false, true) => {
                    let id = if is_fixed1 { id2 } else { id1 } as usize;
                    let color_mask = bcolors[id] | ext_mask;
                    let free = !color_mask;
                    *color = Self::keep_or_pick(stored_color, color_mask, || {
                        if free == 0 {
                            128
                        } else {
                            127 - free.leading_zeros() as usize
                        }
                    });
                    color_len[*color] += 1;
                    if *color < 128 {
                        bcolors[id] |= 1 << *color;
                    }
                }
                (true, true) => unreachable!(),
            }
        }

        // Persist the assignment: the next rebuild starts from these colors, so a cold
        // assembly reproduces the layout a warm one is holding.
        for (interaction_id, color) in self
            .interaction_indices
            .iter()
            .zip(self.interaction_colors.iter())
        {
            interactions[*interaction_id].weight.solver_color = *color as u8;
        }

        let mut sort_offsets = [0; 129];
        let mut last_offset = 0;

        for i in 0..129 {
            if color_len[i] != 0 {
                self.groups.push(last_offset);
                self.group_colors.push(i as u8);
                sort_offsets[i] = last_offset;
                last_offset += color_len[i];
            }
        }

        self.sorted_interactions
            .resize(interaction_indices.len(), 0);

        for (interaction_id, color) in interaction_indices
            .iter()
            .zip(self.interaction_colors.iter())
        {
            self.sorted_interactions[sort_offsets[*color]] = *interaction_id;
            sort_offsets[*color] += 1;
        }

        self.groups.push(self.sorted_interactions.len());
    }
}

/// Per-interaction data prefetched once at the beginning of `group_manifold_refs`
/// so the contact-count passes don't re-read the rigid-body set.
#[derive(Copy, Clone)]
struct InteractionToGroup {
    /// `active_set_id` of the first body, or `u32::MAX` if it doesn't conflict
    /// (fixed, kinematic-less, or sleeping).
    id1: u32,
    /// Same as `id1` for the second body.
    id2: u32,
    /// Position of this interaction inside `interaction_indices`.
    position: u32,
}

pub(crate) struct InteractionGroups {
    // The buckets are indexed by the bit index of the `u128` conflict masks.
    bucket_len: [u8; 128],
    body_masks: Vec<u128>,
    /// Workspace: per row-layout signature, the mask of buckets currently
    /// accumulating joints of that signature (joints of different signatures
    /// can't share a SIMD group).
    to_group: Vec<InteractionToGroup>,
    /// Bucket workspace of [`Self::group_manifold_refs`].
    ref_bucket_slots: Box<[[ContactRef; SIMD_WIDTH]; 128]>,
    pub simd_ref_interactions: Vec<ContactRef>,
    pub nongrouped_ref_interactions: Vec<ContactRef>,
}

impl InteractionGroups {
    pub fn new() -> Self {
        Self {
            bucket_len: [0; 128],
            body_masks: Vec::new(),
            to_group: Vec::new(),
            ref_bucket_slots: Box::new([[ContactRef::PADDING; SIMD_WIDTH]; 128]),
            simd_ref_interactions: Vec::new(),
            nongrouped_ref_interactions: Vec::new(),
        }
    }

    pub fn clear_groups(&mut self) {
        self.simd_ref_interactions.clear();
        self.nongrouped_ref_interactions.clear();
    }

    /// Single-threaded, bucket-direct grouping: groups the overflow-color manifolds (which may
    /// share bodies) into body-disjoint SIMD groups, reading stamped ids/counts through the store.
    pub fn group_manifold_refs(
        &mut self,
        num_island_bodies: usize,
        store: &ManifoldStore,
        refs: &[ContactRef],
    ) {
        self.body_masks.resize(num_island_bodies, 0u128);

        self.to_group.clear();
        self.to_group.reserve(refs.len());
        let mut max_id = 0usize;
        for (position, r) in refs.iter().enumerate() {
            let data = &store.get(*r).data;
            let [id1, id2] = data.solver_body_ids;

            if id1 == u32::MAX && id2 == u32::MAX {
                continue;
            }

            let num_contacts = data.num_active_contacts() as u32;
            if num_contacts == 0 {
                continue;
            }

            if id1 != u32::MAX {
                max_id = max_id.max(id1 as usize);
            }
            if id2 != u32::MAX {
                max_id = max_id.max(id2 as usize);
            }
            self.to_group.push(InteractionToGroup {
                id1,
                id2,
                position: position as u32,
            });
        }

        if max_id >= self.body_masks.len() {
            self.body_masks.resize(max_id + 1, 0u128);
        }

        // Single grouping pass: the wide constraint kernels handle per-lane
        // contact counts, so a group may freely mix counts.
        {
            let mut occupied_mask = 0u128;

            for i in 0..self.to_group.len() {
                let meta = self.to_group[i];
                let interaction_r = refs[meta.position as usize];
                let is_fixed1 = meta.id1 == u32::MAX;
                let is_fixed2 = meta.id2 == u32::MAX;
                let mask1 = if !is_fixed1 {
                    self.body_masks[meta.id1 as usize]
                } else {
                    0
                };
                let mask2 = if !is_fixed2 {
                    self.body_masks[meta.id2 as usize]
                } else {
                    0
                };
                let conflicts = mask1 | mask2;
                let conflictfree_targets = !(conflicts & occupied_mask);
                let conflictfree_occupied_targets = conflictfree_targets & occupied_mask;

                let target_index = if conflictfree_occupied_targets != 0 {
                    // Try to fill partial WContacts first.
                    conflictfree_occupied_targets.trailing_zeros()
                } else {
                    conflictfree_targets.trailing_zeros()
                };

                if target_index == 128 {
                    // The interaction conflicts with every bucket we can manage.
                    self.nongrouped_ref_interactions.push(interaction_r);
                    continue;
                }

                let target_mask_bit = 1 << target_index;
                let bucket = &mut self.ref_bucket_slots[target_index as usize];
                let bucket_len = &mut self.bucket_len[target_index as usize];

                if *bucket_len as usize == SIMD_LAST_INDEX {
                    // We completed our group.
                    bucket[SIMD_LAST_INDEX] = interaction_r;
                    self.simd_ref_interactions.extend_from_slice(&bucket[..]);
                    *bucket_len = 0;
                    occupied_mask &= !target_mask_bit;
                } else {
                    bucket[*bucket_len as usize] = interaction_r;
                    *bucket_len += 1;
                    occupied_mask |= target_mask_bit;
                }

                // NOTE: fixed bodies don't transmit forces. Therefore they don't
                // imply any interaction conflicts.
                if !is_fixed1 {
                    self.body_masks[meta.id1 as usize] |= target_mask_bit;
                }
                if !is_fixed2 {
                    self.body_masks[meta.id2 as usize] |= target_mask_bit;
                }
            }

            // Flush the partially-filled buckets.
            for target_index in 0..128 {
                let bucket_len = self.bucket_len[target_index] as usize;
                if bucket_len != 0 {
                    self.nongrouped_ref_interactions
                        .extend_from_slice(&self.ref_bucket_slots[target_index][..bucket_len]);
                    self.bucket_len[target_index] = 0;
                }
            }
            self.body_masks.iter_mut().for_each(|e| *e = 0);
        }

        assert!(
            self.simd_ref_interactions.len() % SIMD_WIDTH == 0,
            "Invalid SIMD contact grouping."
        );
    }
}
