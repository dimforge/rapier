use super::SahTreeNode;
use crate::data::Index;
use crate::geometry::broad_phase_sah::sah_tree::{SahLeafData, SahTreeNodeWide};
use crate::geometry::Aabb;
use crate::geometry::SahTree;
use crate::math::{Real, Vector};
use parry::bounding_volume::BoundingVolume;

impl SahTree {
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
        let mut path_taken = vec![];

        const APPLY_ROTATIONS_DOWN: bool = true;
        const APPLY_ROTATIONS_UP: bool = false;

        loop {
            if APPLY_ROTATIONS_UP {
                path_taken.push(curr_id);
            }

            if APPLY_ROTATIONS_DOWN {
                self.maybe_apply_rotation(curr_id);
            }

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
                    break;
                } else {
                    let right = &mut self.nodes[curr_id as usize].right;
                    curr_id = right.children;
                    right.data.add_leaf_count(1);
                    right.mins = right.mins.inf(&aabb.mins);
                    right.maxs = right.maxs.sup(&aabb.maxs);
                }
            }
        }

        if APPLY_ROTATIONS_UP {
            while let Some(node) = path_taken.pop() {
                self.maybe_apply_rotation(node);
            }
        }
    }

    // Applies a tree rotation at the given `node` if this improves the SAH metric at that node.
    pub fn maybe_apply_rotation(&mut self, node_id: u32) {
        let node = self.nodes[node_id as usize];
        let left = &node.left;
        let right = &node.right;

        let curr_score =
            left.volume() * left.leaf_count() as Real + right.volume() * right.leaf_count() as Real;

        macro_rules! eval_costs {
            ($left: ident, $right: ident) => {
                if !$left.is_leaf() {
                    let children = self.nodes[$left.children as usize];
                    let left_child = &children.left;
                    let right_child = &children.right;

                    // New SAH score after transforming [{left_child, right_child}, right]
                    // into [left_child, {right_child, right}].
                    let new_score1 = left_child.volume() * left_child.leaf_count() as Real
                        + right_child.merged_volume($right)
                            * (right_child.leaf_count() + $right.leaf_count()) as Real;

                    // New SAH score after transforming [{left_child, right_child}, right]
                    // into [right_child, {left_child, right}].
                    let new_score2 = right_child.volume() * right_child.leaf_count() as Real
                        + left_child.merged_volume($right)
                            * (left_child.leaf_count() + $right.leaf_count()) as Real;

                    if new_score1 < new_score2 {
                        (new_score1 - curr_score, true)
                    } else {
                        (new_score2 - curr_score, false)
                    }
                } else {
                    (Real::MAX, false)
                }
            };
        }

        // Because of the rotation some leaves might have changed location.
        // This a helper to update the `leaf_data` map accordingly.
        macro_rules! set_leaf_data {
            ($leaf_data_id: ident, $node_id: ident, $left_or_right: expr) => {
                let leaf_data = self.leaf_data.get_mut_unknown_gen($leaf_data_id).unwrap();
                leaf_data.node = $node_id;
                leaf_data.left_or_right = $left_or_right;
            };
        }

        // For right rotation.
        let (rotation_score0, left_child_moves_up0) = eval_costs!(left, right);
        // For left rotation.
        let (rotation_score1, left_child_moves_up1) = eval_costs!(right, left);

        if rotation_score0 < 0.0 || rotation_score1 < 0.0 {
            // At least one of the rotations is worth it, apply the one with
            // the best impact on SAH scoring.
            if rotation_score0 < rotation_score1 {
                // Apply RIGHT rotation.
                let children_id = left.children;
                let children = self.nodes[children_id as usize];
                let left_child = &children.left;
                let right_child = &children.right;

                let right_is_leaf = right.is_leaf();
                let left_child_is_leaf = left_child.is_leaf();
                let right_child_is_leaf = right_child.is_leaf();

                let right_leaf_data = right.children;
                let left_child_leaf_data = left_child.children;
                let right_child_leaf_data = right_child.children;

                if left_child_moves_up0 {
                    // The left child moves into `left`, and `right` takes it place.
                    self.nodes[node_id as usize].left = *left_child;
                    self.nodes[children_id as usize].left = *right;
                    self.nodes[node_id as usize].right =
                        self.nodes[children_id as usize].merged(children_id);
                    if left_child_is_leaf {
                        set_leaf_data!(left_child_leaf_data, node_id, Self::LEFT);
                    }
                    if right_is_leaf {
                        set_leaf_data!(right_leaf_data, children_id, Self::LEFT);
                    }
                } else {
                    // The right child moves into `left`, and `right` takes it place.
                    self.nodes[node_id as usize].left = *right_child;
                    self.nodes[children_id as usize].right = *right;
                    self.nodes[node_id as usize].right =
                        self.nodes[children_id as usize].merged(children_id);
                    if right_child_is_leaf {
                        set_leaf_data!(right_child_leaf_data, node_id, Self::LEFT);
                    }
                    if right_is_leaf {
                        set_leaf_data!(right_leaf_data, children_id, Self::RIGHT);
                    }
                }
            } else {
                // Apply LEFT rotation.
                let children_id = right.children;
                let children = self.nodes[children_id as usize];
                let left_child = &children.left;
                let right_child = &children.right;

                let left_is_leaf = left.is_leaf();
                let left_child_is_leaf = left_child.is_leaf();
                let right_child_is_leaf = right_child.is_leaf();

                let left_leaf_data = left.children;
                let left_child_leaf_data = left_child.children;
                let right_child_leaf_data = right_child.children;

                if left_child_moves_up1 {
                    // The left child moves into `right`, and `left` takes it place.
                    self.nodes[node_id as usize].right = *left_child;
                    self.nodes[children_id as usize].left = *left;
                    self.nodes[node_id as usize].left =
                        self.nodes[children_id as usize].merged(children_id);
                    if left_child_is_leaf {
                        set_leaf_data!(left_child_leaf_data, node_id, Self::RIGHT);
                    }
                    if left_is_leaf {
                        set_leaf_data!(left_leaf_data, children_id, Self::LEFT);
                    }
                } else {
                    // The right child moves into `right`, and `left` takes it place.
                    self.nodes[node_id as usize].right = *right_child;
                    self.nodes[children_id as usize].right = *left;
                    self.nodes[node_id as usize].left =
                        self.nodes[children_id as usize].merged(children_id);
                    if right_child_is_leaf {
                        set_leaf_data!(right_child_leaf_data, node_id, Self::RIGHT);
                    }
                    if left_is_leaf {
                        set_leaf_data!(left_leaf_data, children_id, Self::RIGHT);
                    }
                }
            }
        }
    }
}
