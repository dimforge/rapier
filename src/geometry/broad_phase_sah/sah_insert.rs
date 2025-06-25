use super::SahTreeNode;
use crate::data::Index;
use crate::geometry::broad_phase_sah::sah_tree::{SahLeafData, SahTreeNodeWide};
use crate::geometry::Aabb;
use crate::geometry::SahTree;
use crate::math::{Real, Vector};
use parry::bounding_volume::BoundingVolume;

impl SahTree {
    pub const CHANGE_DETECTION_ENABLED: bool = true;

    pub fn pre_update_or_insert(&mut self, aabb: Aabb, leaf_data: u32) {
        if let Some(leaf) = self.leaf_data.get_mut_unknown_gen(leaf_data) {
            let node =
                &mut self.nodes[leaf.node as usize].as_array_mut()[leaf.left_or_right as usize];

            if Self::CHANGE_DETECTION_ENABLED {
                if !node.contains_aabb(&aabb) {
                    const MARGIN: Real = 1.0e-2; // TODO: multiply this by the physics length unit?
                    node.mins = aabb.mins - Vector::repeat(MARGIN);
                    node.maxs = aabb.maxs + Vector::repeat(MARGIN);
                    node.data.set_change_pending();
                }
            } else {
                node.mins = aabb.mins;
                node.maxs = aabb.maxs;
            }
        } else {
            self.insert(aabb, leaf_data);
        }
    }

    // TODO: move to its own file.
    pub fn insert(&mut self, aabb: Aabb, leaf_data: u32) {
        let unknown_gen = 0; // FIXME
        self.leaf_data.insert(
            Index::from_raw_parts(leaf_data, unknown_gen),
            SahLeafData::default(),
        );
        let leaf_data_mut = self.leaf_data.get_mut_unknown_gen(leaf_data).unwrap();

        // If the tree is empty, create the root.
        if self.nodes.is_empty() {
            self.nodes.push(SahTreeNodeWide {
                left: SahTreeNode::leaf(aabb, leaf_data),
                right: SahTreeNode::zeros(),
            });
            leaf_data_mut.node = 0;
            leaf_data_mut.left_or_right = Self::LEFT;
            return;
        }

        // If we have a root, but it is partial, just complete it.
        if self.nodes[0].right.leaf_count() == 0 {
            self.nodes[0].right = SahTreeNode::leaf(aabb, leaf_data);
            leaf_data_mut.node = 0;
            leaf_data_mut.left_or_right = Self::RIGHT;
            return;
        }

        // General case: traverse the tree to find room for the new leaf.
        let mut curr_id = 0u32;

        loop {
            let curr_node = &self.nodes[curr_id as usize];

            // Need to determine the best side to insert our node.
            let left = &curr_node.left;
            let right = &curr_node.right;

            let left_merged_aabb = left.aabb().merged(&aabb);
            let right_merged_aabb = right.aabb().merged(&aabb);

            let left_merged_vol = left_merged_aabb.volume();
            let right_merged_vol = right_merged_aabb.volume();
            let left_vol = left.aabb().volume();
            let right_vol = right.aabb().volume();
            let left_count = left.leaf_count();
            let right_count = right.leaf_count();

            // NOTE: when calculating the SAH cost, we don’t care about dividing by the
            //       parent’s volume since both compared costs use the same factor so
            //       ignoring it doesn’t affect the comparison.
            let left_cost =
                left_merged_vol * (left_count + 1) as Real + right_vol * right_count as Real;
            let right_cost =
                right_merged_vol * (right_count + 1) as Real + left_vol * left_count as Real;

            // Insert into the branch with lowest post-insertion SAH cost.
            // If the costs are equal, just pick the branch with the smallest leaf count.
            if left_cost < right_cost || (left_cost == right_cost && left_count < right_count) {
                // Insert left. The `left` node will become an internal node.
                // We create a new wide leaf containing the current and new leaves.
                if left.is_leaf() {
                    let new_leaf_id = self.nodes.len();
                    let wide_node = SahTreeNodeWide {
                        left: *left,
                        right: SahTreeNode::leaf(aabb, leaf_data),
                    };
                    self.nodes.push(wide_node);

                    let left = &mut self.nodes[curr_id as usize].left;
                    *self.leaf_data.get_mut_unknown_gen(left.children).unwrap() = SahLeafData {
                        node: new_leaf_id as u32,
                        left_or_right: Self::LEFT,
                    };
                    *self.leaf_data.get_mut_unknown_gen(leaf_data).unwrap() = SahLeafData {
                        node: new_leaf_id as u32,
                        left_or_right: Self::RIGHT,
                    };

                    left.children = new_leaf_id as u32;
                    left.data.add_leaf_count(1);
                    left.mins = left.mins.inf(&aabb.mins);
                    left.maxs = left.maxs.sup(&aabb.maxs);
                    return;
                } else {
                    let left = &mut self.nodes[curr_id as usize].left;
                    curr_id = left.children;
                    left.data.add_leaf_count(1);
                    left.mins = left.mins.inf(&aabb.mins);
                    left.maxs = left.maxs.sup(&aabb.maxs);
                }
            } else {
                // Insert right. The `right` node will become an internal node.
                // We create a new wide leaf containing the current and new leaves.
                if right.is_leaf() {
                    let new_leaf_id = self.nodes.len();
                    self.nodes.push(SahTreeNodeWide {
                        left: SahTreeNode::leaf(aabb, leaf_data),
                        right: *right,
                    });

                    let right = &mut self.nodes[curr_id as usize].right;
                    *self.leaf_data.get_mut_unknown_gen(leaf_data).unwrap() = SahLeafData {
                        node: new_leaf_id as u32,
                        left_or_right: Self::LEFT,
                    };
                    *self.leaf_data.get_mut_unknown_gen(right.children).unwrap() = SahLeafData {
                        node: new_leaf_id as u32,
                        left_or_right: Self::RIGHT,
                    };

                    right.children = new_leaf_id as u32;
                    right.data.add_leaf_count(1);
                    right.mins = right.mins.inf(&aabb.mins);
                    right.maxs = right.maxs.sup(&aabb.maxs);
                    return;
                } else {
                    let right = &mut self.nodes[curr_id as usize].right;
                    curr_id = right.children;
                    right.data.add_leaf_count(1);
                    right.mins = right.mins.inf(&aabb.mins);
                    right.maxs = right.maxs.sup(&aabb.maxs);
                }
            }
        }
    }
}
