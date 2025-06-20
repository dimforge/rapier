use crate::math::Real;
use ordered_float::OrderedFloat;
use std::cmp::Ordering;

use super::{SahTree, SahWorkspace};

impl SahTree {
    fn optimization_config(&self, workspace: &SahWorkspace) -> OptimizationConfig {
        const TARGET_REBUILD_NODE_PERCENTAGE: u32 = 5;
        let num_leaves = self.nodes[0].leaf_count();
        let num_optimized_leaves = (num_leaves * TARGET_REBUILD_NODE_PERCENTAGE).div_ceil(100);

        let num_leaves_sqrt = (num_leaves as f64).sqrt();
        let root_mode = if workspace.rebuild_frame_index % 2 == 0 {
            RootOptimizationMode::Skip
        } else if (workspace.rebuild_frame_index / 2) % 8 == 0 {
            RootOptimizationMode::BreadthFirst
        } else {
            RootOptimizationMode::PriorityQueue
        };

        let target_subtree_leaf_count = (num_leaves_sqrt * 4.0).ceil() as usize;

        // println!(
        //     "Num leaves: {}, to optimize: {}, sqrt: {}",
        //     num_leaves, num_optimized_leaves, num_leaves_sqrt
        // );
        OptimizationConfig {
            target_root_node_count: num_leaves_sqrt.ceil() as usize,
            target_subtree_leaf_count,
            target_optimized_subtree_count: (num_optimized_leaves as usize
                / target_subtree_leaf_count)
                .max(1),
            root_mode,
        }
    }

    pub fn optimize_incremental(&mut self, workspace: &mut SahWorkspace) {
        if self.nodes.is_empty() {
            return;
        }

        workspace.rebuild_leaves.clear();
        workspace.rebuild_frame_index += 1;

        /*
         * 1. Root optimization.
         * TODO: make it so that the root optimization doesn’t cross the boundaries of the
         *       subsequent subtree optimizations (important for parallelism).
         */
        let num_leaves = self.nodes[0].leaf_count();
        let config = self.optimization_config(workspace);
        workspace.rebuild_leaves.clear();

        let t0 = std::time::Instant::now();
        match config.root_mode {
            RootOptimizationMode::BreadthFirst => self
                .find_root_optimization_pseudo_leaves_bfs(workspace, config.target_root_node_count),
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
        println!("Root optimization: {}", t0.elapsed().as_secs_f32() * 1000.0);

        /*
         * 2. Subtree optimizations.
         */
        let t0 = std::time::Instant::now();
        let start_index =
            (workspace.rebuild_frame_index * config.target_subtree_leaf_count as u32) % num_leaves;
        let max_candidate_leaf_count =
            config.target_subtree_leaf_count * config.target_optimized_subtree_count;
        let mut target_optimized_subtree_count = max_candidate_leaf_count as i32;

        // println!("Max candidateleaf count = {}", max_candidate_leaf_count);

        self.find_optimization_roots(
            workspace,
            0,
            start_index,
            &mut target_optimized_subtree_count,
            0,
            config.target_subtree_leaf_count as u32,
        );
        println!(
            "Optimized root pseudo-leaves: {}",
            workspace.rebuild_leaves.len()
        );

        // TODO: if we hit the end of the tree, wrap back at the beginning.

        // println!(
        //     "Num refinement candidates: {}, list: {:?}",
        //     workspace.optimization_roots.len(),
        //     workspace.optimization_roots
        // );

        for i in 0..workspace.optimization_roots.len() {
            let subtree_root_id = workspace.optimization_roots[i];
            workspace.rebuild_leaves.clear();
            // NOTE: we have the guarantee that the subtree root has two children since
            //       `find_optimization_roots` checks the number of children > 2.
            //       We don’t just call `free_subtree_and_collect_leaves` on `subtree_root_id`
            //       since we don’t want to free it.
            let [child0, child1] = self.nodes[subtree_root_id as usize].children;
            self.free_subtree_and_collect_leaves(workspace, child0);
            self.free_subtree_and_collect_leaves(workspace, child1);

            println!("Optimized leaves: {}", workspace.rebuild_leaves.len());

            self.rebuild_range(subtree_root_id, &mut workspace.rebuild_leaves);
        }

        println!(
            "Leaf optimization: {}, optimization roots: {}, config: {:?}",
            t0.elapsed().as_secs_f32() * 1000.0,
            workspace.optimization_roots.len(),
            config
        );

        workspace.optimization_roots.clear();
    }

    fn free_subtree_and_collect_leaves(&mut self, workspace: &mut SahWorkspace, subtree_root: u32) {
        let node = &self.nodes[subtree_root as usize];

        if node.is_leaf() {
            workspace.rebuild_leaves.push(subtree_root);
        } else {
            let children = node.children;

            self.free_subtree_and_collect_leaves(workspace, children[0]);
            self.free_subtree_and_collect_leaves(workspace, children[1]);
        }
    }

    fn find_root_optimization_pseudo_leaves_bfs(
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

            if node.is_leaf() {
                workspace.rebuild_leaves.push(curr_node);
            } else {
                let left_id = node.children[0];
                let right_id = node.children[1];

                // NOTE: the root node for the recursion isn’t freed, only all its descendants.
                workspace.dequeue.push_back(left_id);
                workspace.dequeue.push_back(right_id);
            }
        }

        workspace.rebuild_leaves.extend(workspace.dequeue.drain(..));
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

            if node.is_leaf() {
                workspace.rebuild_leaves.push(curr_node.id);
            } else {
                let left_id = node.children[0];
                let right_id = node.children[1];

                let left_score = self.nodes[left_id as usize].aabb.volume();
                let right_score = self.nodes[right_id as usize].aabb.volume();

                // NOTE: the root node for the recursion isn’t freed, only all its descendants.
                workspace.queue.push(SahOptimizationHeapEntry {
                    score: OrderedFloat(left_score),
                    id: left_id,
                });
                workspace.queue.push(SahOptimizationHeapEntry {
                    score: OrderedFloat(right_score),
                    id: right_id,
                });
            }
        }

        workspace
            .rebuild_leaves
            .extend(workspace.queue.as_slice().iter().map(|e| e.id));
        workspace.queue.clear();
    }

    fn find_optimization_roots(
        &mut self,
        workspace: &mut SahWorkspace,
        curr_node: u32,
        start_index: u32,
        target_count: &mut i32,
        mut leaf_count_before: u32,
        max_candidate_leaf_count: u32,
    ) {
        if *target_count <= 0 {
            // We reached the desired number of collected leaves. Just exit.
            return;
        }

        let node = &self.nodes[curr_node as usize];
        let left_id = node.children[0];
        let right_id = node.children[1];
        let left = &self.nodes[left_id as usize];
        let left_leaf_count = left.leaf_count();

        if leaf_count_before + left_leaf_count > start_index {
            // Traverse the left children.
            if left_leaf_count < max_candidate_leaf_count {
                // If the node doesn’t have at least the leaves, it can’t be rebalanced.
                if left_leaf_count > 2 {
                    workspace.optimization_roots.push(left_id);
                    *target_count -= left_leaf_count as i32;
                }
            } else {
                // This node has too many leaves. Recurse.
                self.find_optimization_roots(
                    workspace,
                    left_id,
                    start_index,
                    target_count,
                    leaf_count_before,
                    max_candidate_leaf_count,
                );
            }
        }

        leaf_count_before += left_leaf_count;

        let right = &self.nodes[right_id as usize];
        let right_leaf_count = right.leaf_count();

        if *target_count > 0 && leaf_count_before + right_leaf_count > start_index {
            // Traverse the right children.
            if right_leaf_count < max_candidate_leaf_count {
                if right_leaf_count > 2 {
                    workspace.optimization_roots.push(right_id);
                    *target_count -= right_leaf_count as i32;
                }
            } else {
                self.find_optimization_roots(
                    workspace,
                    right_id,
                    start_index,
                    target_count,
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
