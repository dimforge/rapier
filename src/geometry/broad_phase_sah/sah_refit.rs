use super::{SahTree, SahTreeNode, SahWorkspace};
use crate::data::Coarena;
use crate::geometry::broad_phase_sah::sah_tree::{SahLeafData, SahTreeNodeWide, Uint};
use aligned_vec::AVec;

impl SahTree {
    pub(super) const LEFT: u8 = 0;
    pub(super) const RIGHT: u8 = 1;

    pub fn refit(&mut self, workspace: &mut SahWorkspace) {
        Self::refit_buffers(
            &mut self.nodes,
            &mut workspace.refit_tmp,
            &mut self.leaf_data,
        );

        // Swap the old nodes with the refitted ones.
        std::mem::swap(&mut self.nodes, &mut workspace.refit_tmp);
    }

    pub fn refit_buffers(
        source: &mut AVec<SahTreeNodeWide>,
        target: &mut AVec<SahTreeNodeWide>,
        leaf_data: &mut Coarena<SahLeafData>,
    ) {
        if !source.is_empty() && source[0].leaf_count() > 2 {
            target.resize(
                source.len(),
                SahTreeNodeWide {
                    left: SahTreeNode::zeros(),
                    right: SahTreeNode::zeros(),
                },
            );

            let mut len = 1;

            // Start with a special case for the root then recurse.
            let left_child_id = source[0].left.children;
            let right_child_id = source[0].right.children;

            if !source[0].left.is_leaf() {
                Self::refit_recurse(
                    source,
                    target,
                    leaf_data,
                    left_child_id,
                    &mut len,
                    0,
                    Self::LEFT,
                );
            } else {
                target[0].left = source[0].left;
                target[0].left.data.resolve_pending_change();

                // NOTE: updating the leaf_data shouldn’t be needed here since the root
                //       is always at 0.
                // *self.leaf_data.get_mut_unknown_gen(left_child_id).unwrap() = SahLeafData {
                //     node: 0,
                //     left_or_right: Self::LEFT,
                // };
            }

            if !source[0].right.is_leaf() {
                Self::refit_recurse(
                    source,
                    target,
                    leaf_data,
                    right_child_id,
                    &mut len,
                    0,
                    Self::RIGHT,
                );
            } else {
                target[0].right = source[0].right;
                target[0].right.data.resolve_pending_change();
                // NOTE: updating the leaf_data shouldn’t be needed here since the root
                //       is always at 0.
                // *self.leaf_data.get_mut_unknown_gen(right_child_id).unwrap() = SahLeafData {
                //     node: 0,
                //     left_or_right: Self::RIGHT,
                // };
            }

            source.truncate(len as usize);
            target.truncate(len as usize);
        }
    }

    fn refit_recurse(
        source: &AVec<SahTreeNodeWide>,
        target: &mut AVec<SahTreeNodeWide>,
        leaf_data: &mut Coarena<SahLeafData>,
        source_id: u32,
        target_id_mut: &mut u32,
        parent_target_id: u32,
        left_or_right: u8,
    ) {
        let target_id = *target_id_mut;
        *target_id_mut += 1;

        let node = &source[source_id as usize];
        let left_is_leaf = node.left.is_leaf();
        let right_is_leaf = node.right.is_leaf();
        let left_source_id = node.left.children;
        let right_source_id = node.right.children;

        if !left_is_leaf {
            Self::refit_recurse(
                source,
                target,
                leaf_data,
                left_source_id,
                target_id_mut,
                target_id,
                Self::LEFT,
            );
        } else {
            let node = &source[source_id as usize];
            target[target_id as usize].left = node.left;
            target[target_id as usize]
                .left
                .data
                .resolve_pending_change();
            *leaf_data.get_mut_unknown_gen(node.left.children).unwrap() = SahLeafData {
                node: target_id,
                left_or_right: Self::LEFT,
            };
        }

        if !right_is_leaf {
            Self::refit_recurse(
                source,
                target,
                leaf_data,
                right_source_id,
                target_id_mut,
                target_id,
                Self::RIGHT,
            );
        } else {
            let node = &source[source_id as usize];
            target[target_id as usize].right = node.right;
            target[target_id as usize]
                .right
                .data
                .resolve_pending_change();
            *leaf_data.get_mut_unknown_gen(node.right.children).unwrap() = SahLeafData {
                node: target_id,
                left_or_right: Self::RIGHT,
            };
        }

        let node = &target[target_id as usize];
        *target[parent_target_id as usize].as_array_mut()[left_or_right as usize] =
            node.left.merged(&node.right, target_id as Uint);
    }

    pub fn refit_without_opt(&mut self) {
        if self.leaf_count() > 2 {
            let root = &self.nodes[0];
            let left = root.left.children;
            let right = root.right.children;
            let left_is_leaf = root.left.is_leaf();
            let right_is_leaf = root.right.is_leaf();

            if !left_is_leaf {
                self.recurse_refit_without_opt(left, 0, Self::LEFT);
            }
            if !right_is_leaf {
                self.recurse_refit_without_opt(right, 0, Self::RIGHT);
            }
        }
    }

    fn recurse_refit_without_opt(&mut self, node_id: u32, parent: u32, right_or_left: u8) {
        let node = &self.nodes[node_id as usize];
        let left = &node.left;
        let right = &node.right;
        let left_is_leaf = left.is_leaf();
        let right_is_leaf = right.is_leaf();
        let left_children = left.children;
        let right_children = right.children;

        if !left_is_leaf {
            self.recurse_refit_without_opt(left_children, node_id, Self::LEFT);
        } else {
            self.nodes[node_id as usize]
                .left
                .data
                .resolve_pending_change();
        }
        if !right_is_leaf {
            self.recurse_refit_without_opt(right_children, node_id, Self::RIGHT);
        } else {
            self.nodes[node_id as usize]
                .right
                .data
                .resolve_pending_change();
        }

        let node = &self.nodes[node_id as usize];
        let left = &node.left;
        let right = &node.right;
        let merged = left.merged(right, node_id);

        let parent_wide = &mut self.nodes[parent as usize];
        *parent_wide.as_array_mut()[right_or_left as usize] = merged;
    }
}
