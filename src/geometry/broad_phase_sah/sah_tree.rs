use crate::data::{Arena, Coarena, Index};
use crate::geometry::Aabb;
use crate::math::{Point, Real};
use ordered_float::OrderedFloat;
use parry::bounding_volume::BoundingVolume;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

#[derive(Copy, Clone, Debug)]
struct OptimizationHeapEntry {
    score: OrderedFloat<Real>,
    id: u32,
}

impl PartialOrd for OptimizationHeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.score.cmp(&other.score))
    }
}

impl Ord for OptimizationHeapEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.score.cmp(&other.score)
    }
}

impl PartialEq for OptimizationHeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.score == other.score
    }
}

impl Eq for OptimizationHeapEntry {}

#[derive(Default, Clone)]
pub struct SahWorkspace {
    refit_tmp: Vec<SahTreeNode>,
    rebuild_leaves: Vec<u32>,
    rebuild_frame_index: u32,
    free_list: Vec<usize>,
    optimization_roots: Vec<u32>,
    queue: BinaryHeap<OptimizationHeapEntry>,
    dequeue: VecDeque<u32>,
}

#[derive(Default, Copy, Clone, Debug)]
#[repr(transparent)]
struct TreeNodeData(u32);
const CHANGED: u32 = 0b01;
const CHANGE_PENDING: u32 = 0b11;

impl TreeNodeData {
    #[inline(always)]
    fn with_leaf_count(leaf_count: u32) -> Self {
        Self(leaf_count)
    }

    #[inline(always)]
    fn with_leaf_count_and_pending_change(leaf_count: u32) -> Self {
        Self(leaf_count | (CHANGE_PENDING << 30))
    }

    #[inline(always)]
    fn leaf_count(self) -> u32 {
        self.0 & 0x3fff_ffff
    }

    #[inline(always)]
    fn is_changed(self) -> bool {
        self.0 >> 30 == CHANGED
    }

    #[inline(always)]
    fn is_change_pending(self) -> bool {
        self.0 >> 30 == CHANGE_PENDING
    }

    #[inline(always)]
    fn add_leaf_count(&mut self, added: u32) {
        self.0 += added;
    }

    #[inline(always)]
    fn set_change_pending(&mut self) {
        self.0 |= CHANGE_PENDING << 30;
    }

    #[inline(always)]
    fn set_leaf_count(&mut self, count: u32) {
        self.0 = (self.0 & !0x3fff_ffff) | count;
    }

    #[inline(always)]
    #[must_use]
    fn resolve_pending_change(self) -> Self {
        if self.is_change_pending() {
            Self((self.0 & 0x3fff_ffff) | (CHANGED << 30))
        } else {
            Self(self.0 & 0x3fff_ffff)
        }
    }

    fn merged(self, other: Self) -> Self {
        let leaf_count = self.leaf_count() + other.leaf_count();
        let changed = (self.0 >> 30) | (other.0 >> 30);
        Self(leaf_count | changed << 30)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SahTreeNode {
    /// Bounding volume of the sub-tree rooted by this node.
    aabb: Aabb,
    /// Children of this node. A node has either 0 (i.e. it’s a leaf) or 2 children.
    ///
    /// If [`Self::leaf_count`] is 1, then the node has 0 children and is a leaf.
    children: [u32; 2],
    /// Packed data associated to this node (leaf count and flags).
    data: TreeNodeData,
}

impl SahTreeNode {
    #[inline(always)]
    fn zeros() -> Self {
        Self {
            aabb: Aabb {
                mins: Point::origin(),
                maxs: Point::origin(),
            },
            children: [0; 2],
            data: TreeNodeData(0),
        }
    }

    #[inline(always)]
    pub fn leaf(aabb: Aabb, leaf_data: [u32; 2]) -> SahTreeNode {
        Self {
            aabb,
            children: leaf_data,
            data: TreeNodeData::with_leaf_count_and_pending_change(1),
        }
    }

    #[inline(always)]
    pub fn is_leaf(&self) -> bool {
        self.leaf_count() == 1
    }

    #[inline(always)]
    fn leaf_count(&self) -> u32 {
        self.data.leaf_count()
    }

    #[inline(always)]
    fn changed(&self) -> bool {
        self.data.is_changed()
    }
}

pub type SahNodeHandle = Index;

#[derive(Copy, Clone, Debug, Default)]
pub struct SahLeafData {
    node: u32,
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

#[derive(Default, Clone)]
pub struct SahTree {
    nodes: Vec<SahTreeNode>,
    leaf_data: Coarena<SahLeafData>,
}

impl SahTree {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            leaf_data: Coarena::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn pre_update_or_insert(&mut self, aabb: Aabb, leaf_data: [u32; 2]) {
        let handle = Index::from_raw_parts(leaf_data[0], leaf_data[1]);
        if let Some(leaf) = self.leaf_data.get(handle) {
            let node = &mut self.nodes[leaf.node as usize];
            if !node.aabb.contains(&aabb) {
                const MARGIN: Real = 1.0e-2;
                node.aabb = aabb.loosened(MARGIN);
                node.data.set_change_pending();
            }
        } else {
            self.insert(aabb, leaf_data);
        }
    }

    pub fn insert(&mut self, aabb: Aabb, leaf_data: [u32; 2]) {
        let new_id = self.nodes.len() as u32;
        self.leaf_data.insert(
            Index::from_raw_parts(leaf_data[0], leaf_data[1]),
            SahLeafData { node: new_id },
        );
        self.nodes.push(SahTreeNode::leaf(aabb, leaf_data));

        if new_id == 0 {
            // We created a new root, nothing else to do.
            return;
        }

        let mut curr_id = 0u32;

        loop {
            let curr_node = &self.nodes[curr_id as usize];

            if curr_node.is_leaf() {
                // Transform `curr_node` to an internal node and
                // attach the existing and new children.
                let moved_child_id = self.nodes.len() as u32;
                let curr_merged_aabb = curr_node.aabb.merged(&aabb);
                let moved_handle =
                    Index::from_raw_parts(curr_node.children[0], curr_node.children[1]);
                self.nodes.push(*curr_node);
                let new_children = [moved_child_id, new_id];
                self.nodes[curr_id as usize] = SahTreeNode {
                    aabb: curr_merged_aabb,
                    children: new_children,
                    data: TreeNodeData::with_leaf_count(2),
                };

                // Adjust the node mapping for the moved leaf.
                let moved_leaf_data = self.leaf_data.get_mut(moved_handle).unwrap();
                moved_leaf_data.node = moved_child_id;

                return;
            } else {
                // Need to determine the best side to insert our node.
                let left = &self.nodes[curr_node.children[0] as usize];
                let right = &self.nodes[curr_node.children[1] as usize];
                let curr_merged_aabb = curr_node.aabb.merged(&aabb);
                let left_merged_aabb = left.aabb.merged(&aabb);
                let right_merged_aabb = right.aabb.merged(&aabb);

                let curr_merged_vol = curr_merged_aabb.volume();
                let left_merged_vol = left_merged_aabb.volume();
                let right_merged_vol = right_merged_aabb.volume();
                let left_vol = left.aabb.volume();
                let right_vol = right.aabb.volume();
                let left_count = left.leaf_count();
                let right_count = right.leaf_count();

                let left_cost = (left_merged_vol * (left_count + 1) as Real
                    + right_vol * right_count as Real)
                    / curr_merged_vol;
                let right_cost = (right_merged_vol * (right_count + 1) as Real
                    + left_vol * left_count as Real)
                    / curr_merged_vol;

                let curr_node = &mut self.nodes[curr_id as usize];
                curr_node.data.add_leaf_count(1);
                curr_node.aabb = curr_merged_aabb;

                if left_cost < right_cost || (left_cost == right_cost && left_count < right_count) {
                    // Insert left.
                    curr_id = curr_node.children[0];
                } else {
                    // Insert right.
                    curr_id = curr_node.children[1];
                }
            }
        }
    }

    pub fn assert_is_depth_first(&self) {
        let mut stack = vec![0];
        let mut loop_id = 0;
        while let Some(id) = stack.pop() {
            assert_eq!(loop_id, id);
            loop_id += 1;
            let node = &self.nodes[id as usize];

            if !node.is_leaf() {
                stack.push(node.children[1]);
                stack.push(node.children[0]);
            }
        }
    }

    pub fn assert_well_formed(&self) {
        self.assert_well_formed_at(0);
    }

    pub fn subtree_height(&self, node_id: u32) -> u32 {
        let node = &self.nodes[node_id as usize];

        if node.is_leaf() {
            1
        } else {
            self.subtree_height(node.children[0])
                .max(self.subtree_height(node.children[1]))
                + 1
        }
    }

    // Returns the expected leaf count.
    fn assert_well_formed_at(&self, node_id: u32) {
        let node = &self.nodes[node_id as usize];
        if node.is_leaf() {
            let leaf_data = self
                .leaf_data
                .get(Index::from_raw_parts(node.children[0], node.children[1]))
                .expect("Leaf not found.");
            assert_eq!(leaf_data.node, node_id);
        } else {
            self.assert_well_formed_at(node.children[0]);
            self.assert_well_formed_at(node.children[1]);
            let left = &self.nodes[node.children[0] as usize];
            let right = &self.nodes[node.children[1] as usize];
            assert!(node.aabb.contains(&left.aabb));
            assert!(node.aabb.contains(&right.aabb));
            assert_eq!(node.leaf_count(), left.leaf_count() + right.leaf_count());
        }
    }

    pub fn remove(&mut self) {}

    pub fn refit_and_cache_optimize(&mut self, workspace: &mut SahWorkspace) {
        if !self.nodes.is_empty() {
            workspace
                .refit_tmp
                .resize(self.nodes.len(), SahTreeNode::zeros());
            self.refit_and_cache_optimize_at(workspace, &mut 0, 0);
            std::mem::swap(&mut self.nodes, &mut workspace.refit_tmp);
            workspace.free_list.clear();
        }
    }

    fn refit_and_cache_optimize_at(
        &mut self,
        workspace: &mut SahWorkspace,
        id: &mut u32,
        node: u32,
    ) -> u32 {
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
            let moved_left_id = self.refit_and_cache_optimize_at(workspace, id, left);
            let moved_right_id = self.refit_and_cache_optimize_at(workspace, id, right);

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

    pub fn rebuild_incremental(&mut self, workspace: &mut SahWorkspace) {
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
            self.rebuild_range(0, &mut workspace.rebuild_leaves, &mut workspace.free_list);
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

            self.rebuild_range(
                subtree_root_id,
                &mut workspace.rebuild_leaves,
                &mut workspace.free_list,
            );
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
            workspace.free_list.push(subtree_root as usize);
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
                if curr_node != 0 {
                    workspace.free_list.push(curr_node as usize);
                }

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

        workspace.queue.push(OptimizationHeapEntry {
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
                if curr_node.id != 0 {
                    workspace.free_list.push(curr_node.id as usize);
                }

                workspace.queue.push(OptimizationHeapEntry {
                    score: OrderedFloat(left_score),
                    id: left_id,
                });
                workspace.queue.push(OptimizationHeapEntry {
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

    pub fn rebalance(&mut self) {}

    pub fn traverse_bvtt_changed_self(
        &self,
        (a, b): (u32, u32),
        f: &mut impl FnMut([u32; 2], [u32; 2]),
    ) {
        let node1 = &self.nodes[a as usize];
        let node2 = &self.nodes[b as usize];

        let [l1, r1] = node1.children;
        let [l2, r2] = node2.children;

        match (node1.is_leaf(), node2.is_leaf()) {
            (false, false) => {
                let left1 = &self.nodes[l1 as usize];
                let right1 = &self.nodes[r1 as usize];
                let left2 = &self.nodes[l2 as usize];
                let right2 = &self.nodes[r2 as usize];

                if left1.changed() && left1.aabb.intersects(&left2.aabb) {
                    self.traverse_bvtt_changed_self((l1, l2), f);
                }
                if left1.changed() && left1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt_changed_self((l1, r2), f);
                }
                if right1.changed() && right1.aabb.intersects(&left2.aabb) {
                    self.traverse_bvtt_changed_self((r1, l2), f);
                }
                if right1.changed() && right1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt_changed_self((r1, r2), f);
                }
            }
            (true, false) => {
                let left2 = &self.nodes[l2 as usize];
                let right2 = &self.nodes[r2 as usize];

                if left2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt_changed_self((a, l2), f);
                }
                if right2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt_changed_self((a, r2), f);
                }
            }
            (false, true) => {
                let left1 = &self.nodes[l1 as usize];
                let right1 = &self.nodes[r1 as usize];

                if left1.changed() && left1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt_changed_self((l1, b), f);
                }
                if right1.changed() && right1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt_changed_self((r1, b), f);
                }
            }
            (true, true) => {
                f(node1.children, node2.children);
            }
        }
    }

    pub fn traverse_bvtt_recursive(
        &self,
        other: &Self,
        (a, b): (u32, u32),
        f: &mut impl FnMut([u32; 2], [u32; 2]),
    ) {
        let node1 = &self.nodes[a as usize];
        let node2 = &other.nodes[b as usize];

        let [l1, r1] = node1.children;
        let [l2, r2] = node2.children;

        match (node1.is_leaf(), node2.is_leaf()) {
            (false, false) => {
                let left1 = &self.nodes[l1 as usize];
                let right1 = &self.nodes[r1 as usize];
                let left2 = &self.nodes[l2 as usize];
                let right2 = &self.nodes[r2 as usize];

                if left1.aabb.intersects(&left2.aabb) {
                    self.traverse_bvtt_recursive(other, (l1, l2), f);
                }
                if left1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt_recursive(other, (l1, r2), f);
                }
                if right1.aabb.intersects(&left2.aabb) {
                    self.traverse_bvtt_recursive(other, (r1, l2), f);
                }
                if right1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt_recursive(other, (r1, r2), f);
                }
            }
            (true, false) => {
                let left2 = &self.nodes[l2 as usize];
                let right2 = &self.nodes[r2 as usize];

                if left2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt_recursive(other, (a, l2), f);
                }
                if right2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt_recursive(other, (a, r2), f);
                }
            }
            (false, true) => {
                let left1 = &self.nodes[l1 as usize];
                let right1 = &self.nodes[r1 as usize];

                if left1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt_recursive(other, (l1, b), f);
                }
                if right1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt_recursive(other, (r1, b), f);
                }
            }
            (true, true) => {
                f(node1.children, node2.children);
            }
        }
    }

    fn sah_cost(&self, node_id: usize) -> Real {
        let node = &self.nodes[node_id];

        if node.leaf_count() == 1 {
            // This is a leaf.
            1.0
        } else {
            let left = &self.nodes[node.children[0] as usize];
            let right = &self.nodes[node.children[1] as usize];
            let my_vol = node.aabb.volume();
            let left_vol = left.aabb.volume();
            let right_vol = right.aabb.volume();
            (left_vol * left.leaf_count() as Real + right_vol * right.leaf_count() as Real) / my_vol
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
