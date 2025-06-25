use super::{SahTree, SahTreeNode, SahWorkspace};
use crate::geometry::broad_phase_sah::sah_tree::{SahLeafData, SahTreeNodeWide, Uint};

impl SahTree {
    pub(super) const LEFT: u8 = 0;
    pub(super) const RIGHT: u8 = 1;

    pub fn refit(&mut self, workspace: &mut SahWorkspace) {
        if !self.nodes.is_empty() && self.nodes[0].leaf_count() > 2 {
            workspace.refit_tmp.resize(
                self.nodes.len(),
                SahTreeNodeWide {
                    left: SahTreeNode::zeros(),
                    right: SahTreeNode::zeros(),
                },
            );

            let mut len = 1;

            // Start with a special case for the root then recurse.
            let left_child_id = self.nodes[0].left.children;
            let right_child_id = self.nodes[0].right.children;

            if !self.nodes[0].left.is_leaf() {
                self.refit_recurse(workspace, left_child_id, &mut len, 0, Self::LEFT);
            } else {
                workspace.refit_tmp[0].left = self.nodes[0].left;
                workspace.refit_tmp[0].left.data.resolve_pending_change();

                // NOTE: updating the leaf_data shouldn’t be needed here since the root
                //       is always at 0.
                // *self.leaf_data.get_mut_unknown_gen(left_child_id).unwrap() = SahLeafData {
                //     node: 0,
                //     left_or_right: Self::LEFT,
                // };
            }

            if !self.nodes[0].right.is_leaf() {
                self.refit_recurse(workspace, right_child_id, &mut len, 0, Self::RIGHT);
            } else {
                workspace.refit_tmp[0].right = self.nodes[0].right;
                workspace.refit_tmp[0].right.data.resolve_pending_change();
                // NOTE: updating the leaf_data shouldn’t be needed here since the root
                //       is always at 0.
                // *self.leaf_data.get_mut_unknown_gen(right_child_id).unwrap() = SahLeafData {
                //     node: 0,
                //     left_or_right: Self::RIGHT,
                // };
            }

            // Swap the old nodes with the refitted ones.
            std::mem::swap(&mut self.nodes, &mut workspace.refit_tmp);
            self.nodes.truncate(len as usize);
            workspace.refit_tmp.truncate(len as usize);
        }
    }

    fn refit_recurse(
        &mut self,
        workspace: &mut SahWorkspace,
        source_id: u32,
        target_id_mut: &mut u32,
        parent_target_id: u32,
        left_or_right: u8,
    ) {
        let target_id = *target_id_mut;
        *target_id_mut += 1;

        let node = &self.nodes[source_id as usize];
        let left_is_leaf = node.left.is_leaf();
        let right_is_leaf = node.right.is_leaf();
        let left_source_id = node.left.children;
        let right_source_id = node.right.children;

        if !left_is_leaf {
            self.refit_recurse(
                workspace,
                left_source_id,
                target_id_mut,
                target_id,
                Self::LEFT,
            );
        } else {
            let node = &self.nodes[source_id as usize];
            workspace.refit_tmp[target_id as usize].left = node.left;
            workspace.refit_tmp[target_id as usize]
                .left
                .data
                .resolve_pending_change();
            *self
                .leaf_data
                .get_mut_unknown_gen(node.left.children)
                .unwrap() = SahLeafData {
                node: target_id,
                left_or_right: Self::LEFT,
            };
        }

        if !right_is_leaf {
            self.refit_recurse(
                workspace,
                right_source_id,
                target_id_mut,
                target_id,
                Self::RIGHT,
            );
        } else {
            let node = &self.nodes[source_id as usize];
            workspace.refit_tmp[target_id as usize].right = node.right;
            workspace.refit_tmp[target_id as usize]
                .right
                .data
                .resolve_pending_change();
            *self
                .leaf_data
                .get_mut_unknown_gen(node.right.children)
                .unwrap() = SahLeafData {
                node: target_id,
                left_or_right: Self::RIGHT,
            };
        }

        let node = &workspace.refit_tmp[target_id as usize];
        *workspace.refit_tmp[parent_target_id as usize].as_array_mut()[left_or_right as usize] =
            node.left.merged(&node.right, target_id as Uint);
    }
}
