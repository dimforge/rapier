use crate::data::{Coarena, Index};
use crate::geometry::Aabb;
use crate::math::{Point, Real};
use parry::bounding_volume::BoundingVolume;
use std::collections::{BinaryHeap, VecDeque};

use super::SahOptimizationHeapEntry;

#[derive(Default, Clone)]
pub struct SahWorkspace {
    pub(super) refit_tmp: Vec<SahTreeNode>,
    pub(super) rebuild_leaves: Vec<u32>,
    pub(super) rebuild_frame_index: u32,
    pub(super) free_list: Vec<usize>,
    pub(super) optimization_roots: Vec<u32>,
    pub(super) queue: BinaryHeap<SahOptimizationHeapEntry>,
    pub(super) dequeue: VecDeque<u32>,
}

#[derive(Default, Copy, Clone, Debug)]
#[repr(transparent)]
pub struct SahNodData(u32);
const CHANGED: u32 = 0b01;
const CHANGE_PENDING: u32 = 0b11;

impl SahNodData {
    #[inline(always)]
    pub(super) fn with_leaf_count(leaf_count: u32) -> Self {
        Self(leaf_count)
    }

    #[inline(always)]
    pub(super) fn with_leaf_count_and_pending_change(leaf_count: u32) -> Self {
        Self(leaf_count | (CHANGE_PENDING << 30))
    }

    #[inline(always)]
    pub(super) fn leaf_count(self) -> u32 {
        self.0 & 0x3fff_ffff
    }

    #[inline(always)]
    pub(super) fn is_changed(self) -> bool {
        self.0 >> 30 == CHANGED
    }

    #[inline(always)]
    pub(super) fn is_change_pending(self) -> bool {
        self.0 >> 30 == CHANGE_PENDING
    }

    #[inline(always)]
    pub(super) fn add_leaf_count(&mut self, added: u32) {
        self.0 += added;
    }

    #[inline(always)]
    pub(super) fn set_change_pending(&mut self) {
        self.0 |= CHANGE_PENDING << 30;
    }

    #[inline(always)]
    pub(super) fn set_leaf_count(&mut self, count: u32) {
        self.0 = (self.0 & !0x3fff_ffff) | count;
    }

    #[inline(always)]
    #[must_use]
    pub(super) fn resolve_pending_change(self) -> Self {
        if self.is_change_pending() {
            Self((self.0 & 0x3fff_ffff) | (CHANGED << 30))
        } else {
            Self(self.0 & 0x3fff_ffff)
        }
    }

    pub(super) fn merged(self, other: Self) -> Self {
        let leaf_count = self.leaf_count() + other.leaf_count();
        let changed = (self.0 >> 30) | (other.0 >> 30);
        Self(leaf_count | changed << 30)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SahTreeNode {
    /// Bounding volume of the sub-tree rooted by this node.
    pub(super) aabb: Aabb,
    /// Children of this node. A node has either 0 (i.e. it’s a leaf) or 2 children.
    ///
    /// If [`Self::leaf_count`] is 1, then the node has 0 children and is a leaf.
    pub(super) children: [u32; 2],
    /// Packed data associated to this node (leaf count and flags).
    pub(super) data: SahNodData,
}

impl SahTreeNode {
    #[inline(always)]
    pub(super) fn zeros() -> Self {
        Self {
            aabb: Aabb {
                mins: Point::origin(),
                maxs: Point::origin(),
            },
            children: [0; 2],
            data: SahNodData(0),
        }
    }

    #[inline(always)]
    pub fn leaf(aabb: Aabb, leaf_data: [u32; 2]) -> SahTreeNode {
        Self {
            aabb,
            children: leaf_data,
            data: SahNodData::with_leaf_count_and_pending_change(1),
        }
    }

    #[inline(always)]
    pub fn is_leaf(&self) -> bool {
        self.leaf_count() == 1
    }

    #[inline(always)]
    pub(super) fn leaf_count(&self) -> u32 {
        self.data.leaf_count()
    }

    #[inline(always)]
    pub(super) fn changed(&self) -> bool {
        self.data.is_changed()
    }
}

pub type SahNodeHandle = Index;

#[derive(Copy, Clone, Debug, Default)]
pub struct SahLeafData {
    pub(super) node: u32,
}

#[derive(Default, Clone)]
pub struct SahTree {
    pub(super) nodes: Vec<SahTreeNode>,
    pub(super) leaf_data: Coarena<SahLeafData>,
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
                    data: SahNodData::with_leaf_count(2),
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
    pub(super) fn assert_well_formed_at(&self, node_id: u32) {
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

    pub(super) fn sah_cost(&self, node_id: usize) -> Real {
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
