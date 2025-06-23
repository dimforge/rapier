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
            let mut len = 1;
            self.refit_recurse(workspace, &mut len, 0, 0);
            std::mem::swap(&mut self.nodes, &mut workspace.refit_tmp);
            self.nodes.truncate(len as usize);
            workspace.refit_tmp.truncate(len as usize);
        }
    }

    fn refit_recurse(
        &mut self,
        workspace: &mut SahWorkspace,
        id: &mut u32,
        parent_source: u32,
        parent_target: u32,
    ) {
        let parent = &self.nodes[parent_source as usize]; // PERF: pass directly the reference to the `node: &SahTreeNode` instead of u32.
        let [left_source_id, right_source_id] = parent.children;

        // Write the children.
        let left_target = *id;
        let right_target = *id + 1;
        *id += 2;

        // Recurse or update leaf/handle association.
        let left_source = &self.nodes[left_source_id as usize];
        if left_source.is_leaf() {
            workspace.refit_tmp[left_target as usize] = SahTreeNode {
                aabb: left_source.aabb,
                children: left_source.children,
                data: left_source.data.resolve_pending_change(),
            };

            self.leaf_data
                .get_mut(Index::from_raw_parts(
                    left_source.children[0],
                    left_source.children[1],
                ))
                .unwrap()
                .node = left_target;
        } else {
            self.refit_recurse(workspace, id, left_source_id, left_target);
        };

        let right_source = &self.nodes[right_source_id as usize];
        if right_source.is_leaf() {
            workspace.refit_tmp[right_target as usize] = SahTreeNode {
                aabb: right_source.aabb,
                children: right_source.children,
                data: right_source.data.resolve_pending_change(),
            };

            self.leaf_data
                .get_mut(Index::from_raw_parts(
                    right_source.children[0],
                    right_source.children[1],
                ))
                .unwrap()
                .node = right_target;
        } else {
            self.refit_recurse(workspace, id, right_source_id, right_target);
        };

        let left = &workspace.refit_tmp[left_target as usize];
        let right = &workspace.refit_tmp[right_target as usize];
        workspace.refit_tmp[parent_target as usize] = SahTreeNode {
            aabb: left.aabb.merged(&right.aabb),
            children: [left_target, right_target],
            data: left.data.merged(right.data),
        };
    }
}
