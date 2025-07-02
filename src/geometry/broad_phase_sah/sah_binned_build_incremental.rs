use super::{SahTree, SahTreeNode, SahWorkspace};
use crate::geometry::broad_phase_sah::sah_tree::SahTreeNodeWide;
use crate::geometry::Aabb;
use crate::math::Real;
use aligned_vec::AVec;
use parry::bounding_volume::BoundingVolume;
use std::collections::VecDeque;
use std::ops::Range;

#[derive(Clone, Debug)]
struct RebuildSubstep {
    parent_id: u32,
    range: Range<usize>,
}

#[derive(Clone)]
pub(super) struct BinnedRebuildState {
    partial_tree: AVec<SahTreeNodeWide>,
    substeps: VecDeque<RebuildSubstep>,
    leaves_to_sort: Vec<SahTreeNode>,
}

impl Default for BinnedRebuildState {
    fn default() -> Self {
        Self {
            partial_tree: AVec::new(std::mem::align_of::<SahTreeNodeWide>()),
            substeps: Default::default(),
            leaves_to_sort: Default::default(),
        }
    }
}

impl BinnedRebuildState {
    pub fn step_rebuild(&mut self, tree: &mut SahTree) -> bool {
        if self.substeps.is_empty() {
            if tree.leaf_count() <= 2 {
                return false; // No rebuild needed.
            }

            // No ongoing rebuild. Start a new one by taking a snapshot
            // of the current tree.
            self.leaves_to_sort.clear();
            self.leaves_to_sort.reserve(tree.leaf_count() as usize);
            for node in tree.nodes.iter() {
                if node.left.is_leaf() {
                    self.leaves_to_sort.push(node.left);
                }
                if node.right.is_leaf() {
                    self.leaves_to_sort.push(node.right);
                }
            }
            self.partial_tree.clear();
            self.partial_tree.reserve(tree.nodes.len());
            self.partial_tree.push(SahTreeNodeWide::zeros());
            self.substeps.push_back(RebuildSubstep {
                parent_id: 0,
                range: 0..self.leaves_to_sort.len(),
            });

            // Exit now. The actual rebuild will start at the next step since this
            // initialization above is a bit expensive.
            return false;
        }

        // If enabled, the latest leaf states will be copied to the leaves being sorted
        // so that their SAH can take the more recent data into account.
        // Whether this is a net gain needs more experiments.
        const ALWAYS_UPDATE_LEAF_STATE: bool = true;

        if ALWAYS_UPDATE_LEAF_STATE {
            for leaf in &mut self.leaves_to_sort {
                let leaf_data = tree.leaf_data.get_unknown_gen(leaf.children).unwrap();
                *leaf = *tree.nodes[leaf_data.node as usize].as_array()
                    [leaf_data.left_or_right as usize];
            }
        }

        let mut num_leaves_to_process = self.leaves_to_sort.len() as isize;
        let mut substep_run = 0;

        while num_leaves_to_process > 0 {
            let Some(substep) = self.substeps.pop_front() else {
                break;
            };
            num_leaves_to_process -= substep.range.len() as isize;
            self.rebuild_range(substep);
            substep_run += 1;
        }
        // println!("Substep run: {}", substep_run);

        if self.substeps.is_empty() {
            // println!("No more substeps!");
            // Incremental rebuild is finished!
            // Finalize the new tree by doing a refit with cache optimization.
            // Cache optimization is important here since our tree is currently
            // stored in breadth-first order, but we actually want it in depth-first order!
            //
            // 1. Read the latest node states.
            // TODO PERF: this could be part of the refit?
            for node in self.partial_tree.iter_mut() {
                if node.left.is_leaf() {
                    let leaf_data = tree.leaf_data.get_unknown_gen(node.left.children).unwrap();
                    node.left = *tree.nodes[leaf_data.node as usize].as_array()
                        [leaf_data.left_or_right as usize];
                }
                if node.right.is_leaf() {
                    let leaf_data = tree.leaf_data.get_unknown_gen(node.right.children).unwrap();
                    node.right = *tree.nodes[leaf_data.node as usize].as_array()
                        [leaf_data.left_or_right as usize];
                }
            }
            // 2. Refit.
            SahTree::refit_buffers(&mut self.partial_tree, &mut tree.nodes, &mut tree.leaf_data);

            true
        } else {
            // println!("Finished with remaining substeps: {}", self.substeps.len());
            false
        }
    }

    fn rebuild_range(&mut self, substep: RebuildSubstep) {
        // PERF: calculate an optimal bin count dynamically based on the number of leaves to split?
        //       The paper suggests (4 + 2 * sqrt(num_leaves).floor()).min(16)
        const NUM_BINS: usize = 8;
        const BIN_EPSILON: Real = 1.0e-5;

        let leaves = &mut self.leaves_to_sort[substep.range.clone()];
        let target_node_id = substep.parent_id;

        let mut bins = [SahBin::default(); NUM_BINS];

        assert!(leaves.len() > 1);

        let centroid_aabb = Aabb::from_points(leaves.iter().map(|node| node.center()));
        let bins_axis = centroid_aabb.extents().imax();
        let bins_range = [centroid_aabb.mins[bins_axis], centroid_aabb.maxs[bins_axis]];

        // Compute bins characteristics.
        let k1 = NUM_BINS as Real * (1.0 - BIN_EPSILON) / (bins_range[1] - bins_range[0]);
        let k0 = bins_range[0];
        for leaf in &*leaves {
            let bin_id = (k1 * (leaf.center()[bins_axis] - k0)) as usize;
            let bin = &mut bins[bin_id];
            bin.aabb.merge(&leaf.aabb());
            bin.leaf_count += 1;
        }

        // Select the best splitting plane (there are NUM_BINS - 1 splitting planes) based on SAH.
        let mut right_merges = bins;
        let mut right_acc = bins[NUM_BINS - 1];

        for i in 1..NUM_BINS - 1 {
            right_acc.aabb.merge(&right_merges[NUM_BINS - 1 - i].aabb);
            right_acc.leaf_count += &right_merges[NUM_BINS - 1 - i].leaf_count;
            right_merges[NUM_BINS - 1 - i] = right_acc;
        }

        let mut best_cost = Real::MAX;
        let mut best_plane = 0;
        let mut left_merge = bins[0];
        let mut best_leaf_count = bins[0].leaf_count;

        for i in 0..NUM_BINS - 1 {
            let right = &right_merges[i + 1];

            let cost = left_merge.aabb.volume() * left_merge.leaf_count as Real
                + right.aabb.volume() * right.leaf_count as Real;
            if cost < best_cost {
                best_cost = cost;
                best_plane = i;
                best_leaf_count = left_merge.leaf_count;
            }

            left_merge.aabb.merge(&bins[i + 1].aabb);
            left_merge.leaf_count += bins[i + 1].leaf_count;
        }

        // With the splitting plane selected, sort & split the leaves in place.
        let mut mid = best_leaf_count as usize;

        // In degenerate cases where all the node end up on the same bin,
        // just split the range in two.
        if mid == 0 || mid == leaves.len() {
            mid = leaves.len() / 2;
        } else {
            // Sort in-place.
            // TODO PERF: try with using teh leaves_tmp instead of in-place sorting.
            let bin = |leaves: &mut [SahTreeNode], id: usize| {
                let node = &leaves[id];
                (k1 * (node.center()[bins_axis] - k0)) as usize
            };

            let mut left_id = 0;
            let mut right_id = mid;

            'outer: while left_id != mid && right_id != leaves.len() {
                while bin(leaves, left_id) <= best_plane {
                    left_id += 1;

                    if left_id == mid {
                        break 'outer;
                    }
                }

                while bin(leaves, right_id) > best_plane {
                    right_id += 1;

                    if right_id == leaves.len() {
                        break 'outer;
                    }
                }

                leaves.swap(left_id, right_id);
                left_id += 1;
                right_id += 1;
            }
        }

        // Recurse.
        let (left_leaves, right_leaves) = leaves.split_at(mid);

        let target = &mut self.partial_tree[target_node_id as usize];
        target.left.data.set_leaf_count(left_leaves.len() as u32);
        target.right.data.set_leaf_count(right_leaves.len() as u32);

        // Recurse.
        if left_leaves.len() == 1 {
            let target = &mut self.partial_tree[target_node_id as usize];
            target.left = left_leaves[0];
        } else {
            let left_id = self.partial_tree.len() as u32;
            self.partial_tree.push(SahTreeNodeWide::zeros());
            self.substeps.push_back(RebuildSubstep {
                parent_id: left_id,
                range: substep.range.start..(substep.range.start + mid),
            });

            let target = &mut self.partial_tree[target_node_id as usize];
            target.left.children = left_id;
        }

        if right_leaves.len() == 1 {
            let target = &mut self.partial_tree[target_node_id as usize];
            target.right = right_leaves[0];
        } else {
            let right_id = self.partial_tree.len() as u32;
            self.partial_tree.push(SahTreeNodeWide::zeros());
            self.substeps.push_back(RebuildSubstep {
                parent_id: right_id,
                range: (substep.range.start + mid)..substep.range.end,
            });

            let target = &mut self.partial_tree[target_node_id as usize];
            target.right.children = right_id;
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct SahBin {
    aabb: Aabb,
    leaf_count: u32,
}

impl Default for SahBin {
    fn default() -> Self {
        Self {
            aabb: Aabb::new_invalid(),
            leaf_count: 0,
        }
    }
}
