use crate::data::Index;
use crate::math::Real;
use parry::bounding_volume::BoundingVolume;

use super::{SahTree, SahTreeNode, SahWorkspace};

impl SahTree {
    pub fn refit(&mut self, workspace: &mut SahWorkspace) {
        if !self.nodes.is_empty() {
            workspace
                .refit_tmp
                .resize(self.nodes.len(), SahTreeNode::zeros());
            self.refit_recurse(workspace, &mut 0, 0);
            std::mem::swap(&mut self.nodes, &mut workspace.refit_tmp);
            workspace.free_list.clear();
        }
    }

    fn refit_recurse(&mut self, workspace: &mut SahWorkspace, id: &mut u32, node: u32) -> u32 {
        let curr_node = &self.nodes[node as usize];
        if curr_node.is_leaf() {
            let moved_node_id = *id;
            workspace.refit_tmp[moved_node_id as usize] = SahTreeNode {
                aabb: curr_node.aabb,
                children: curr_node.children,
                data: curr_node.data.resolve_pending_change(),
            };

            self.leaf_data
                .get_mut(Index::from_raw_parts(
                    curr_node.children[0],
                    curr_node.children[1],
                ))
                .unwrap()
                .node = moved_node_id;
            *id += 1;
            moved_node_id
        } else {
            let moved_node_id = *id;
            *id += 1;

            let [left, right] = curr_node.children;
            let moved_left_id = self.refit_recurse(workspace, id, left);
            let moved_right_id = self.refit_recurse(workspace, id, right);

            let left_node = workspace.refit_tmp[moved_left_id as usize];
            let right_node = workspace.refit_tmp[moved_right_id as usize];
            workspace.refit_tmp[moved_node_id as usize] = SahTreeNode {
                aabb: left_node.aabb.merged(&right_node.aabb),
                children: [moved_left_id, moved_right_id],
                data: left_node.data.merged(right_node.data),
            };
            moved_node_id
        }
    }
}
