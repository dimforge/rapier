use crate::math::Real;
use ordered_float::OrderedFloat;
use std::cmp::Ordering;

use super::{SahTree, SahWorkspace};

impl SahTree {
    fn optimization_config(&self, frame_index: u32) -> OptimizationConfig {
        const TARGET_REBUILD_NODE_PERCENTAGE: u32 = 5;
        let num_leaves = self.nodes[0].leaf_count();
        let num_optimized_leaves = (num_leaves * TARGET_REBUILD_NODE_PERCENTAGE).div_ceil(100);

        let num_leaves_sqrt = (num_leaves as Real).sqrt();
        let root_mode = if frame_index % 2 == 0 {
            RootOptimizationMode::Skip
        } else if (frame_index / 2) % 16 == 0 {
            RootOptimizationMode::BreadthFirst
        } else {
            RootOptimizationMode::PriorityQueue
        };

        let target_root_node_count = num_leaves_sqrt.ceil();
        let target_subtree_leaf_count = (num_leaves_sqrt * 4.0).ceil();
        let root_refinement_cost = target_root_node_count * target_root_node_count.log2()
            / (target_subtree_leaf_count * target_subtree_leaf_count.log2());
        let mut target_optimized_subtree_count =
            (num_optimized_leaves as Real / target_subtree_leaf_count - root_refinement_cost)
                .round()
                .max(0.0) as usize;

        if root_mode == RootOptimizationMode::Skip {
            target_optimized_subtree_count = target_optimized_subtree_count.max(1)
        }

        OptimizationConfig {
            target_root_node_count: target_root_node_count as usize,
            target_subtree_leaf_count: target_subtree_leaf_count as usize,
            target_optimized_subtree_count,
            root_mode,
        }
    }

    pub fn optimize_incremental(&mut self, workspace: &mut SahWorkspace) {
        if self.nodes.is_empty() {
            return;
        }

        workspace.rebuild_leaves.clear();
        workspace.rebuild_frame_index = workspace.rebuild_frame_index.overflowing_add(1).0;
        let config = self.optimization_config(workspace.rebuild_frame_index);

        /*
         * Subtree optimizations.
         */
        // let t0 = std::time::Instant::now();
        let num_leaves = self.nodes[0].leaf_count();
        let mut start_index = workspace.rebuild_start_index;

        // println!("Max candidate leaf count = {}", max_candidate_leaf_count);
        self.find_optimization_roots(
            workspace,
            0,
            &mut start_index,
            config.target_optimized_subtree_count as u32,
            0,
            config.target_subtree_leaf_count as u32,
        );

        if start_index >= num_leaves {
            start_index = 0;
            // TODO: if we hit the end of the tree, wrap back at the beginning
            //       to reach the target subtree count.
        }

        workspace.rebuild_start_index = start_index;

        // println!(
        //     "Num refinement candidates: {}, list: {:?}",
        //     workspace.optimization_roots.len(),
        //     workspace.optimization_roots
        // );

        /*
         * Root optimization.
         */
        workspace.rebuild_leaves.clear();

        // let t0 = std::time::Instant::now();
        match config.root_mode {
            RootOptimizationMode::BreadthFirst => self
                .find_root_optimization_pseudo_leaves_breadth_first(
                    workspace,
                    config.target_root_node_count,
                ),
            RootOptimizationMode::PriorityQueue => self
                .find_root_optimization_pseudo_leaves_pqueue(
                    workspace,
                    config.target_root_node_count,
                ),
            RootOptimizationMode::Skip => {}
        }

        if !workspace.rebuild_leaves.is_empty() {
            self.rebuild_range(0, &mut workspace.rebuild_leaves);
        }
        // println!("Root optimization: {}", t0.elapsed().as_secs_f32() * 1000.0);

        /*
         * Subtree leaf optimizations.
         */
        for i in 0..workspace.optimization_roots.len() {
            let subtree_root_id = workspace.optimization_roots[i];
            workspace.rebuild_leaves.clear();
            self.collect_leaves(workspace, subtree_root_id);

            // let t1 = std::time::Instant::now();
            self.rebuild_range(subtree_root_id, &mut workspace.rebuild_leaves);
            // println!(
            //     "Optimized leaves: {}, time: {}",
            //     workspace.rebuild_leaves.len(),
            //     t1.elapsed().as_secs_f32() * 1000.0
            // );
        }

        // println!(
        //     "Leaf optimization: {}, optimization roots: {}, config: {:?}",
        //     t0.elapsed().as_secs_f32() * 1000.0,
        //     workspace.optimization_roots.len(),
        //     config
        // );
        workspace.optimization_roots.clear();
    }

    fn collect_leaves(&self, workspace: &mut SahWorkspace, subtree_root: u32) {
        let node = &self.nodes[subtree_root as usize];
        let left = &node.left;
        let right = &node.right;

        if left.is_leaf() {
            workspace.rebuild_leaves.push(*left);
        } else {
            self.collect_leaves(workspace, left.children);
        }

        if right.is_leaf() {
            workspace.rebuild_leaves.push(*right);
        } else {
            self.collect_leaves(workspace, right.children);
        }
    }

    fn find_root_optimization_pseudo_leaves_breadth_first(
        &mut self,
        workspace: &mut SahWorkspace,
        target_count: usize,
    ) {
        if self.nodes.len() < 2 {
            return;
        }

        workspace.dequeue.push_back(0);

        while workspace.dequeue.len() + workspace.rebuild_leaves.len() < target_count {
            let Some(curr_node) = workspace.dequeue.pop_front() else {
                break;
            };

            let node = &self.nodes[curr_node as usize];
            let left = &node.left;
            let right = &node.right;

            if left.is_leaf() || left.data.is_change_pending() {
                workspace.rebuild_leaves.push(*left);
            } else {
                workspace.dequeue.push_back(left.children);
            }

            if right.is_leaf() || right.data.is_change_pending() {
                workspace.rebuild_leaves.push(*right);
            } else {
                workspace.dequeue.push_back(right.children);
            }
        }

        for id in workspace.dequeue.drain(..) {
            let node = &self.nodes[id as usize];
            workspace.rebuild_leaves.push(node.left);
            workspace.rebuild_leaves.push(node.right);
        }
    }

    fn find_root_optimization_pseudo_leaves_pqueue(
        &mut self,
        workspace: &mut SahWorkspace,
        target_count: usize,
    ) {
        if self.nodes.len() < 2 {
            return;
        }

        workspace.queue.push(SahOptimizationHeapEntry {
            score: OrderedFloat(Real::MAX),
            id: 0,
        });

        while workspace.queue.len() + workspace.rebuild_leaves.len() < target_count {
            let Some(curr_node) = workspace.queue.pop() else {
                break;
            };

            let node = &self.nodes[curr_node.id as usize];
            let left = &node.left;
            let right = &node.right;

            if left.is_leaf() || left.data.is_change_pending() {
                workspace.rebuild_leaves.push(*left);
            } else {
                let children = self.nodes[left.children as usize];
                let left_score = children.left.volume() * children.left.leaf_count() as Real
                    + children.right.volume() * children.right.leaf_count() as Real;
                workspace.queue.push(SahOptimizationHeapEntry {
                    score: OrderedFloat(left_score),
                    id: left.children,
                });
            }

            if right.is_leaf() || right.data.is_change_pending() {
                workspace.rebuild_leaves.push(*right);
            } else {
                let children = self.nodes[right.children as usize];
                let right_score = children.left.volume() * children.left.leaf_count() as Real
                    + children.right.volume() * children.right.leaf_count() as Real;
                workspace.queue.push(SahOptimizationHeapEntry {
                    score: OrderedFloat(right_score),
                    id: right.children,
                });
            }
        }

        for id in workspace.queue.as_slice() {
            let node = &self.nodes[id.id as usize];
            workspace.rebuild_leaves.push(node.left);
            workspace.rebuild_leaves.push(node.right);
        }
        workspace.queue.clear();
    }

    fn find_optimization_roots(
        &mut self,
        workspace: &mut SahWorkspace,
        curr_node: u32,
        start_index: &mut u32,
        max_optimization_roots: u32,
        mut leaf_count_before: u32,
        max_candidate_leaf_count: u32,
    ) {
        if workspace.optimization_roots.len() == max_optimization_roots as usize {
            // We reached the desired number of collected leaves. Just exit.
            return;
        }

        let node = &mut self.nodes[curr_node as usize];
        let left = &mut node.left;
        let left_leaf_count = left.leaf_count();
        let left_children = left.children;

        if leaf_count_before + left_leaf_count > *start_index {
            // Traverse the left children.
            if left_leaf_count < max_candidate_leaf_count {
                // If the node doesn’t have at least the leaves, it can’t be rebalanced.
                if left_leaf_count > 2 {
                    // Mark the optimization root so that the root pseudo-leaf
                    // extraction knows not to cross this node.
                    // This won’t disturb bvtt traversal because refit will get
                    // rid of this flag.
                    left.data.set_change_pending();
                    workspace.optimization_roots.push(left_children);
                    *start_index += left_leaf_count;
                }
            } else {
                // This node has too many leaves. Recurse.
                self.find_optimization_roots(
                    workspace,
                    left_children,
                    start_index,
                    max_optimization_roots,
                    leaf_count_before,
                    max_candidate_leaf_count,
                );
            }
        }

        leaf_count_before += left_leaf_count;

        if workspace.optimization_roots.len() == max_optimization_roots as usize {
            // We reached the desired number of collected leaves. Just exit.
            return;
        }

        let node = &mut self.nodes[curr_node as usize];
        let right = &mut node.right;
        let right_leaf_count = right.leaf_count();
        let right_children = right.children;

        if leaf_count_before + right_leaf_count > *start_index {
            // Traverse the right children.
            if right_leaf_count < max_candidate_leaf_count {
                if right_leaf_count > 2 {
                    // Mark the optimization root so that the root pseudo-leaf
                    // extraction knows not to cross this node.
                    // This won’t disturb bvtt traversal because refit will get
                    // rid of this flag.
                    right.data.set_change_pending();
                    workspace.optimization_roots.push(right_children);
                    *start_index += right_leaf_count;
                }
            } else {
                self.find_optimization_roots(
                    workspace,
                    right_children,
                    start_index,
                    max_optimization_roots,
                    leaf_count_before,
                    max_candidate_leaf_count,
                );
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum RootOptimizationMode {
    PriorityQueue,
    BreadthFirst,
    Skip,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
struct OptimizationConfig {
    target_root_node_count: usize,
    target_subtree_leaf_count: usize,
    target_optimized_subtree_count: usize,
    root_mode: RootOptimizationMode,
}

#[derive(Copy, Clone, Debug)]
pub struct SahOptimizationHeapEntry {
    score: OrderedFloat<Real>,
    id: u32,
}

impl PartialOrd for SahOptimizationHeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.score.cmp(&other.score))
    }
}

impl Ord for SahOptimizationHeapEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.score.cmp(&other.score)
    }
}

impl PartialEq for SahOptimizationHeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.score == other.score
    }
}

impl Eq for SahOptimizationHeapEntry {}
