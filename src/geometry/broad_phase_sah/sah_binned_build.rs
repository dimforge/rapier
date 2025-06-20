use crate::data::Index;
use crate::geometry::Aabb;
use crate::math::Real;
use parry::bounding_volume::BoundingVolume;

use super::{SahLeafData, SahTree, SahTreeNode, SahWorkspace};

impl SahTree {
    pub fn rebuild(&mut self, workspace: &mut SahWorkspace) {
        if self.nodes.is_empty() {
            return;
        }

        workspace.rebuild_leaves.clear();
        for (id, node) in self.nodes.iter().enumerate() {
            if node.is_leaf() {
                workspace.rebuild_leaves.push(id as u32);
            } else {
                // Never free the root.
                if id != 0 {
                    workspace.free_list.push(id);
                }
            }
        }

        self.nodes[0] = SahTreeNode::zeros();
        self.rebuild_range(0, &mut workspace.rebuild_leaves, &mut workspace.free_list);
    }

    pub fn rebuild_range(
        &mut self,
        target_node_id: u32,
        leaves: &mut [u32],
        free_list: &mut Vec<usize>,
    ) {
        // PERF: calculate an optimal bin count dynamically based on the number of leaves to split?
        //       The paper suggests (4 + 2 * sqrt(num_leaves).floor()).min(16)
        const NUM_BINS: usize = 8;
        const BIN_EPSILON: Real = 1.0e-5;

        let mut bins = [SahBin::default(); NUM_BINS];

        // PERF: have dedicated branches for:
        //       - edge-cases (e.g. if all the aabbs have the same center and can’t be splitted).
        //       - when the remaining leaf count is smaller than NUM_BINS (=> test all the splitting planes).
        //       - when the leaf count is equal to 2.
        if leaves.len() == 1 {
            let leaf = &self.nodes[target_node_id as usize];

            // Check if this is an actual leaf instead of a pseudo-leaf for partial rebuild.
            // If it is, update the corresponding handle mapping.
            if leaf.is_leaf() {
                let leaf_data_handle = Index::from_raw_parts(leaf.children[0], leaf.children[1]);
                self.leaf_data.insert(
                    leaf_data_handle,
                    SahLeafData {
                        node: target_node_id,
                    },
                );
            }

            return;
        }

        let centroid_aabb =
            Aabb::from_points(leaves.iter().map(|l| self.nodes[*l as usize].aabb.center()));
        let bins_axis = centroid_aabb.extents().imax();
        let bins_range = [centroid_aabb.mins[bins_axis], centroid_aabb.maxs[bins_axis]];

        // Compute bins characteristics.
        let k1 = NUM_BINS as Real * (1.0 - BIN_EPSILON) / (bins_range[1] - bins_range[0]);
        let k0 = bins_range[0];
        for leaf in &*leaves {
            let leaf = &self.nodes[*leaf as usize];
            let bin_id = (k1 * (leaf.aabb.center()[bins_axis] - k0)) as usize;
            let bin = &mut bins[bin_id];
            bin.aabb.merge(&leaf.aabb);
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
            let bin = |leaves: &mut [u32], id: usize| {
                let node = &self.nodes[leaves[id] as usize];
                (k1 * (node.aabb.center()[bins_axis] - k0)) as usize
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
        let (left_leaves, right_leaves) = leaves.split_at_mut(mid);

        assert!(!left_leaves.is_empty() && !right_leaves.is_empty());

        // Allocate child nodes.
        let left_id = if left_leaves.len() == 1 {
            left_leaves[0]
        } else if let Some(entry) = free_list.pop() {
            self.nodes[entry] = SahTreeNode::zeros();
            entry as u32
        } else {
            self.nodes.push(SahTreeNode::zeros());
            self.nodes.len() as u32 - 1
        };
        let right_id = if right_leaves.len() == 1 {
            right_leaves[0]
        } else if let Some(entry) = free_list.pop() {
            self.nodes[entry] = SahTreeNode::zeros();
            entry as u32
        } else {
            self.nodes.push(SahTreeNode::zeros());
            self.nodes.len() as u32 - 1
        };

        // Recurse.
        self.rebuild_range(left_id, left_leaves, free_list);
        self.rebuild_range(right_id, right_leaves, free_list);

        // Populate the parent’s node AABB and leaf count.
        let left_node = &self.nodes[left_id as usize];
        let right_node = &self.nodes[right_id as usize];
        let curr_aabb = left_node.aabb.merged(&right_node.aabb);
        let curr_count = left_node.leaf_count() + right_node.leaf_count();
        let curr_node = &mut self.nodes[target_node_id as usize];
        curr_node.aabb = curr_aabb;
        curr_node.data.set_leaf_count(curr_count);
        curr_node.children = [left_id, right_id];
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
