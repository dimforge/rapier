use super::{SahTree, SahTreeNode, SahWorkspace};

impl SahTree {
    pub fn traverse_bvtt_single_tree(
        &self,
        workspace: &mut SahWorkspace,
        f: &mut impl FnMut(u32, u32),
    ) {
        if self.nodes.is_empty() || self.nodes[0].left.leaf_count() == 0 {
            // Not enough nodes for any overlap.
            return;
        }

        workspace.traversal_stack.clear();
        self.self_intersect_node(workspace, 0, f)
    }

    // Traverses overlaps of a single node with itself.
    // This as special case to:
    // - Ensure we don’t traverse the same branch twice.
    // - Only check the left/right overlap. Left/left and right/right checks trivially pass.
    // TODO: take change detection into account.
    fn self_intersect_node(
        &self,
        workspace: &mut SahWorkspace,
        id: u32,
        f: &mut impl FnMut(u32, u32),
    ) {
        let node = &self.nodes[id as usize];

        if Self::CHANGE_DETECTION_ENABLED && !node.right.changed() && !node.left.changed() {
            return;
        }

        let left_right_intersect = node.left.intersects(&node.right);
        let left_child = node.left.children;
        let right_child = node.right.children;
        let left_is_leaf = node.left.is_leaf();
        let right_is_leaf = node.right.is_leaf();

        if (!Self::CHANGE_DETECTION_ENABLED || node.left.changed()) && !left_is_leaf {
            self.self_intersect_node(workspace, left_child, f);
        }

        if (!Self::CHANGE_DETECTION_ENABLED || node.right.changed()) && !right_is_leaf {
            self.self_intersect_node(workspace, right_child, f);
        }

        if left_right_intersect {
            match (left_is_leaf, right_is_leaf) {
                (true, true) => f(left_child, right_child),
                (true, false) => {
                    self.traverse_single_subtree(workspace, &node.left, right_child, f)
                }
                (false, true) => {
                    self.traverse_single_subtree(workspace, &node.right, left_child, f)
                }
                (false, false) => self.traverse_two_branches(workspace, left_child, right_child, f),
            }
        }
    }

    fn traverse_two_branches(
        &self,
        workspace: &mut SahWorkspace,
        a: u32,
        b: u32,
        f: &mut impl FnMut(u32, u32),
    ) {
        let node1 = &self.nodes[a as usize];
        let node2 = &self.nodes[b as usize];

        let left1 = &node1.left;
        let right1 = &node1.right;
        let left2 = &node2.left;
        let right2 = &node2.right;

        let left_left = (!Self::CHANGE_DETECTION_ENABLED || left1.changed() || left2.changed())
            && left1.intersects(&left2);
        let left_right = (!Self::CHANGE_DETECTION_ENABLED || left1.changed() || right2.changed())
            && left1.intersects(&right2);
        let right_left = (!Self::CHANGE_DETECTION_ENABLED || right1.changed() || left2.changed())
            && right1.intersects(&left2);
        let right_right = (!Self::CHANGE_DETECTION_ENABLED || right1.changed() || right2.changed())
            && right1.intersects(&right2);

        macro_rules! dispatch(
            ($check: ident, $child_a: ident, $child_b: ident) => {
                if $check {
                    match ($child_a.is_leaf(), $child_b.is_leaf()) {
                        (true, true) => f($child_a.children, $child_b.children),
                        (true, false) => {
                            self.traverse_single_subtree(workspace, $child_a, $child_b.children, f)
                        }
                        (false, true) => self.traverse_single_subtree(
                            workspace,
                            $child_b,
                            $child_a.children,
                            f,
                        ),
                        (false, false) => self.traverse_two_branches(
                            workspace,
                            $child_a.children,
                            $child_b.children,
                            f,
                        ),
                    }
                }
            }
        );

        dispatch!(left_left, left1, left2);
        dispatch!(left_right, left1, right2);
        dispatch!(right_left, right1, left2);
        dispatch!(right_right, right1, right2);
    }

    fn traverse_single_subtree_recursive(
        &self,
        workspace: &mut SahWorkspace,
        node: &SahTreeNode,
        subtree: u32,
        f: &mut impl FnMut(u32, u32),
    ) {
        let subtree = &self.nodes[subtree as usize];
        let left_check = subtree.left.intersects(node);
        let right_check = subtree.right.intersects(node);

        if left_check {
            if subtree.left.is_leaf() {
                f(node.children, subtree.left.children)
            } else {
                self.traverse_single_subtree(workspace, node, subtree.left.children, f);
            }
        }

        if right_check {
            if subtree.right.is_leaf() {
                f(node.children, subtree.right.children)
            } else {
                self.traverse_single_subtree(workspace, node, subtree.right.children, f);
            }
        }
    }

    // Checks overlap between a single node and a subtree.
    fn traverse_single_subtree(
        &self,
        workspace: &mut SahWorkspace,
        node: &SahTreeNode,
        subtree: u32,
        f: &mut impl FnMut(u32, u32),
    ) {
        debug_assert!(workspace.traversal_stack.is_empty());

        // Since this is traversing against a single node it is more efficient to keep the leaf reference
        // around and traverse the branch using a manual stack. Left branches are traversed by the main
        // loop whereas the right branches are pushed to the stack.
        let mut curr_id = subtree;
        let node_changed = node.changed();

        loop {
            let curr = &self.nodes[curr_id as usize];
            let left = &curr.left;
            let right = &curr.right;
            let left_check = (!Self::CHANGE_DETECTION_ENABLED || node_changed || left.changed())
                && node.intersects(&left);
            let right_check = (!Self::CHANGE_DETECTION_ENABLED || node_changed || right.changed())
                && node.intersects(&right);
            let left_is_leaf = left.is_leaf();
            let right_is_leaf = right.is_leaf();
            let mut found_next = false;

            if left_check {
                if left_is_leaf {
                    f(node.children, left.children)
                } else {
                    curr_id = left.children;
                    found_next = true;
                }
            }

            if right_check {
                if right_is_leaf {
                    f(node.children, right.children)
                } else if !found_next {
                    curr_id = right.children;
                    found_next = true;
                } else {
                    // We already advanced in curr_id once, push the other
                    // branch to the stack.
                    workspace.traversal_stack.push(right.children);
                }
            }

            if !found_next {
                // Pop the stack to find the next candidate.
                if let Some(next_id) = workspace.traversal_stack.pop() {
                    curr_id = next_id;
                } else {
                    // Traversal is finished.
                    return;
                }
            }
        }
    }
}
