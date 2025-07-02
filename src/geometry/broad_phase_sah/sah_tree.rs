use super::SahOptimizationHeapEntry;
use crate::data::{Coarena, Index};
use crate::geometry::Aabb;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SimdReal;
use crate::math::{Point, Real};
use aligned_vec::AVec;
use parry::bounding_volume::BoundingVolume;
use parry::query::Ray;
use std::collections::{BinaryHeap, HashSet, VecDeque};

// NOTE: this is all temporary data that can be freed between broad-phase updates
//       without affecting simulation results.
#[derive(Clone)]
pub struct SahWorkspace {
    pub(super) refit_tmp: AVec<SahTreeNodeWide>,
    pub(super) rebuild_leaves: Vec<SahTreeNode>,
    pub(super) rebuild_tmp: Vec<SahTreeNode>,
    pub(super) rebuild_frame_index: u32,
    pub(super) rebuild_start_index: u32,
    pub(super) optimization_roots: Vec<u32>,
    pub(super) queue: BinaryHeap<SahOptimizationHeapEntry>,
    pub(super) dequeue: VecDeque<u32>,
    pub(super) traversal_stack: Vec<u32>,
}

impl Default for SahWorkspace {
    fn default() -> Self {
        Self {
            refit_tmp: AVec::new(std::mem::align_of::<SahTreeNodeWide>()),
            rebuild_leaves: Default::default(),
            rebuild_tmp: Default::default(),
            rebuild_frame_index: Default::default(),
            rebuild_start_index: Default::default(),
            traversal_stack: Default::default(),
            optimization_roots: Default::default(),
            queue: Default::default(),
            dequeue: Default::default(),
        }
    }
}

#[cfg(feature = "f32")]
pub(super) type Uint = u32;
#[cfg(feature = "f64")]
pub(super) type Uint = u64;

#[derive(Default, Copy, Clone, Debug)]
#[repr(transparent)]
pub struct SahNodeData(Uint);
const CHANGED: Uint = 0b01;
const CHANGE_PENDING: Uint = 0b11;

impl SahNodeData {
    #[inline(always)]
    pub(super) fn with_leaf_count(leaf_count: Uint) -> Self {
        Self(leaf_count)
    }

    #[inline(always)]
    pub(super) fn with_leaf_count_and_pending_change(leaf_count: Uint) -> Self {
        Self(leaf_count | (CHANGE_PENDING << 30))
    }

    #[inline(always)]
    pub(super) fn leaf_count(self) -> Uint {
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
    pub(super) fn add_leaf_count(&mut self, added: Uint) {
        self.0 += added;
    }

    #[inline(always)]
    pub(super) fn set_change_pending(&mut self) {
        self.0 |= CHANGE_PENDING << 30;
    }

    #[inline(always)]
    pub(super) fn set_leaf_count(&mut self, count: Uint) {
        self.0 = (self.0 & !0x3fff_ffff) | count;
    }

    #[inline(always)]
    pub(super) fn resolve_pending_change(&mut self) {
        if self.is_change_pending() {
            *self = Self((self.0 & 0x3fff_ffff) | (CHANGED << 30));
        } else {
            *self = Self(self.0 & 0x3fff_ffff);
        }
    }

    pub(super) fn merged(self, other: Self) -> Self {
        let leaf_count = self.leaf_count() + other.leaf_count();
        let changed = (self.0 >> 30) | (other.0 >> 30);
        Self(leaf_count | changed << 30)
    }
}

/// A pair of tree nodes.
///
/// Both `left` and `right` are guaranteed to be valid except for the only special-case where the
/// tree contains only a single leaf, in which case only `left` is valid. But in every other
/// cases where the tree contains at least 2 leaves, booth `left` and `right` are guaranteed
/// to be valid.
#[derive(Copy, Clone, Debug)]
#[repr(C)] // SAFETY: needed to ensure SIMD aabb checks rely on the layout.
pub struct SahTreeNodeWide {
    pub(super) left: SahTreeNode,
    pub(super) right: SahTreeNode,
}

impl SahTreeNodeWide {
    #[inline(always)]
    pub fn zeros() -> Self {
        Self {
            left: SahTreeNode::zeros(),
            right: SahTreeNode::zeros(),
        }
    }

    #[inline(always)]
    pub fn as_array(&self) -> [&SahTreeNode; 2] {
        [&self.left, &self.right]
    }

    #[inline(always)]
    pub fn as_array_mut(&mut self) -> [&mut SahTreeNode; 2] {
        [&mut self.left, &mut self.right]
    }

    pub fn merged(&self, my_id: u32) -> SahTreeNode {
        self.left.merged(&self.right, my_id)
    }

    pub fn leaf_count(&self) -> u32 {
        self.left.leaf_count() + self.right.leaf_count()
    }
}

#[repr(C)] // SAFETY: needed to ensure SIMD aabb checks rely on the layout.
#[cfg_attr(feature = "f32", repr(align(32)))]
#[cfg_attr(feature = "f64", repr(align(64)))]
#[cfg(feature = "simd-is-enabled")]
struct SahSimdTreeNode {
    mins: SimdReal,
    maxs: SimdReal,
}

#[derive(Copy, Clone, Debug)]
#[repr(C)] // SAFETY: needed to ensure SIMD aabb checks rely on the layout.
#[cfg_attr(feature = "f32", repr(align(32)))]
#[cfg_attr(feature = "f64", repr(align(64)))]
pub struct SahTreeNode {
    /// Mins coordinates of thes node’s bounding volume.
    pub(super) mins: Point<Real>,
    /// Children of this node. A node has either 0 (i.e. it’s a leaf) or 2 children.
    ///
    /// If [`Self::leaf_count`] is 1, then the node has 0 children and is a leaf.
    pub(super) children: Uint,
    /// Maxs coordinates of this node’s bouding volume.
    pub(super) maxs: Point<Real>,
    /// Packed data associated to this node (leaf count and flags).
    pub(super) data: SahNodeData,
}

impl SahTreeNode {
    #[inline(always)]
    pub(super) fn zeros() -> Self {
        Self {
            mins: Point::origin(),
            children: 0,
            maxs: Point::origin(),
            data: SahNodeData(0),
        }
    }

    #[inline(always)]
    pub fn leaf(aabb: Aabb, leaf_data: Uint) -> SahTreeNode {
        Self {
            mins: aabb.mins,
            maxs: aabb.maxs,
            children: leaf_data,
            data: SahNodeData::with_leaf_count_and_pending_change(1),
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

    #[inline(always)]
    #[cfg(all(feature = "simd-is-enabled", feature = "dim3"))]
    pub(super) fn as_simd(&self) -> &SahSimdTreeNode {
        // SAFETY: SahTreeNode is declared with the alignment
        //         and size of two SimdReal.
        unsafe { std::mem::transmute(self) }
    }

    #[inline(always)]
    pub(super) fn merged(&self, other: &Self, children: Uint) -> Self {
        // TODO PERF: simd optimizations?
        Self {
            mins: self.mins.inf(&other.mins),
            children,
            maxs: self.maxs.sup(&other.maxs),
            data: self.data.merged(other.data),
        }
    }

    pub fn aabb(&self) -> Aabb {
        Aabb {
            mins: self.mins,
            maxs: self.maxs,
        }
    }

    pub fn center(&self) -> Point<Real> {
        na::center(&self.mins, &self.maxs)
    }

    pub fn volume(&self) -> Real {
        // TODO PERF: simd optimizations?
        let extents = self.maxs - self.mins;
        #[cfg(feature = "dim2")]
        return extents.x * extents.y;
        #[cfg(feature = "dim3")]
        return extents.x * extents.y * extents.z;
    }

    pub fn merged_volume(&self, other: &Self) -> Real {
        // TODO PERF: simd optimizations?
        let mins = self.mins.inf(&other.mins);
        let maxs = self.maxs.sup(&other.maxs);
        let extents = maxs - mins;

        #[cfg(feature = "dim2")]
        return extents.x * extents.y;
        #[cfg(feature = "dim3")]
        return extents.x * extents.y * extents.z;
    }

    #[cfg(not(all(feature = "simd-is-enabled", feature = "dim3")))]
    pub fn intersects(&self, other: &Self) -> bool {
        na::partial_le(&self.mins, &other.maxs) && na::partial_ge(&self.maxs, &other.mins)
    }

    #[cfg(all(feature = "simd-is-enabled", feature = "dim3"))]
    pub fn intersects(&self, other: &Self) -> bool {
        use wide::{CmpGe, CmpLe};
        let simd_self = self.as_simd();
        let simd_other = other.as_simd();
        let check =
            simd_self.mins.0.cmp_le(simd_other.maxs.0) & simd_self.maxs.0.cmp_ge(simd_other.mins.0);
        (check.move_mask() & 0b0111) == 0b0111
    }

    #[cfg(not(all(feature = "simd-is-enabled", feature = "dim3")))]
    pub fn contains(&self, other: &Self) -> bool {
        na::partial_le(&self.mins, &other.mins) && na::partial_ge(&self.maxs, &other.maxs)
    }

    #[cfg(all(feature = "simd-is-enabled", feature = "dim3"))]
    pub fn contains(&self, other: &Self) -> bool {
        use wide::{CmpGe, CmpLe};
        let simd_self = self.as_simd();
        let simd_other = other.as_simd();
        let check =
            simd_self.mins.0.cmp_le(simd_other.mins.0) & simd_self.maxs.0.cmp_ge(simd_other.maxs.0);
        (check.move_mask() & 0b0111) == 0b0111
    }

    pub fn contains_aabb(&self, other: &Aabb) -> bool {
        // TODO PERF: simd optimizations?
        na::partial_le(&self.mins, &other.mins) && na::partial_ge(&self.maxs, &other.maxs)
    }
}

pub type SahNodeHandle = Index;

#[derive(Copy, Clone, Debug, Default)]
pub struct SahLeafData {
    pub(super) node: u32,
    // TODO: we could pack that into `Self::node` since
    //       this is just a flag.
    pub(super) left_or_right: u8,
}

#[derive(Clone, Debug)]
pub struct SahTree {
    pub(super) nodes: AVec<SahTreeNodeWide>,
    pub(super) leaf_data: Coarena<SahLeafData>,
}

impl Default for SahTree {
    fn default() -> Self {
        Self::new()
    }
}

impl SahTree {
    // NOTE PERF: change detection doesn’t make a huge difference in 2D (it can
    //            occasionally even make it slower!) Once we support a static/dynamic tree
    //            instead of a single tree, we might want to fully disable change detection
    //            in 2D.
    pub const CHANGE_DETECTION_ENABLED: bool = true;

    pub fn new() -> Self {
        assert_eq!(align_of::<SahTreeNodeWide>(), 32);
        Self {
            nodes: AVec::new(align_of::<SahTreeNodeWide>()),
            leaf_data: Coarena::new(),
        }
    }

    pub fn from_leaves(leaves: &[(u32, Aabb)]) -> Self {
        if leaves.is_empty() {
            return Self::new();
        }

        let mut result = Self::new();
        let mut workspace = SahWorkspace::default();
        workspace.rebuild_leaves.reserve(leaves.len());
        result.leaf_data.reserve(leaves.len());

        for (leaf_id, leaf_aabb) in leaves {
            workspace
                .rebuild_leaves
                .push(SahTreeNode::leaf(*leaf_aabb, *leaf_id));
            result
                .leaf_data
                .insert(Index::from_raw_parts(*leaf_id, 0), SahLeafData::default());
        }
        result.nodes.reserve(leaves.len());
        result.nodes.push(SahTreeNodeWide::zeros());
        result.rebuild_range(0, &mut workspace.rebuild_leaves);
        println!("SAH nodes: {}", result.nodes.len());
        result
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn assert_is_depth_first(&self) {
        let mut stack = vec![0];
        let mut loop_id = 0;
        while let Some(id) = stack.pop() {
            assert_eq!(loop_id, id);
            loop_id += 1;
            let node = &self.nodes[id as usize];

            if !node.right.is_leaf() {
                stack.push(node.right.children);
            }

            if !node.left.is_leaf() {
                stack.push(node.left.children);
            }
        }
    }

    pub fn subtree_height(&self, node_id: u32) -> u32 {
        if node_id == 0 && self.nodes.is_empty() {
            return 0;
        } else if node_id == 0 && self.nodes.len() == 1 {
            return 1 + (self.nodes[0].right.leaf_count() != 0) as u32;
        }

        let node = &self.nodes[node_id as usize];

        let left_height = if node.left.is_leaf() {
            1
        } else {
            self.subtree_height(node.left.children)
        };

        let right_height = if node.right.is_leaf() {
            1
        } else {
            self.subtree_height(node.right.children)
        };

        left_height.max(right_height) + 1
    }

    pub fn leaf_count(&self) -> u32 {
        if self.nodes.is_empty() {
            0
        } else {
            self.nodes[0].leaf_count()
        }
    }

    pub fn reachable_leaf_count(&self, id: u32) -> u32 {
        if self.nodes.is_empty() {
            0
        } else if self.nodes[0].right.leaf_count() == 0 {
            1
        } else {
            let node = &self.nodes[id as usize];
            let left_count = if node.left.is_leaf() {
                1
            } else {
                self.reachable_leaf_count(node.left.children)
            };
            let right_count = if node.right.is_leaf() {
                1
            } else {
                self.reachable_leaf_count(node.right.children)
            };
            left_count + right_count
        }
    }

    pub fn changed_leaf_count(&self, id: u32) -> u32 {
        if self.nodes.is_empty() {
            0
        } else if self.nodes[0].right.leaf_count() == 0 {
            1
        } else {
            let node = &self.nodes[id as usize];
            let left_count = if node.left.is_leaf() {
                node.left.changed() as u32
            } else {
                self.changed_leaf_count(node.left.children)
            };
            let right_count = if node.right.is_leaf() {
                node.right.changed() as u32
            } else {
                self.changed_leaf_count(node.right.children)
            };
            left_count + right_count
        }
    }

    pub fn assert_well_formed(&self) {
        if self.is_empty() {
            return;
        } else if self.nodes[0].right.leaf_count() == 0 {
            assert_eq!(self.nodes[0].leaf_count(), 1);
            assert!(self.nodes[0].left.is_leaf());
            return;
        }

        let mut loop_detection = HashSet::new();
        self.assert_well_formed_at(0, &mut loop_detection);
    }

    // Panics if the tree isn’t well-formed.
    //
    // The tree is well-formed if it is topologically correct (internal indices are all valid) and
    // geometrically correct (node metadata of a parent bound the ones of the children).
    //
    // Returns the calculated leaf count.
    pub(super) fn assert_well_formed_at(
        &self,
        node_id: u32,
        loop_detection: &mut HashSet<u32>,
    ) -> u32 {
        let node = &self.nodes[node_id as usize];

        if !loop_detection.insert(node_id) {
            panic!("Detected loop. Node {} visited twice.", node_id);
        }

        let left_count = if node.left.is_leaf() {
            let leaf_data = self
                .leaf_data
                .get_unknown_gen(node.left.children)
                .expect("Leaf not found.");
            assert_eq!(leaf_data.node, node_id);
            assert_eq!(leaf_data.left_or_right, Self::LEFT);
            1
        } else {
            let calculated_leaf_count =
                self.assert_well_formed_at(node.left.children, loop_detection);
            let child = &self.nodes[node.left.children as usize];
            assert_eq!(
                child.right.changed() || child.left.changed(),
                node.left.changed()
            );
            assert!(node.left.contains(&child.left));
            assert!(node.left.contains(&child.right));
            assert_eq!(
                node.left.leaf_count(),
                child.left.leaf_count() + child.right.leaf_count()
            );
            assert_eq!(node.left.leaf_count(), calculated_leaf_count);
            calculated_leaf_count
        };

        let right_count = if node.right.is_leaf() {
            let leaf_data = self
                .leaf_data
                .get_unknown_gen(node.right.children)
                .expect("Leaf not found.");
            assert_eq!(leaf_data.node, node_id);
            assert_eq!(leaf_data.left_or_right, Self::RIGHT);
            1
        } else {
            let calculated_leaf_count =
                self.assert_well_formed_at(node.right.children, loop_detection);
            let child = &self.nodes[node.right.children as usize];
            assert_eq!(
                child.right.changed() || child.left.changed(),
                node.right.changed()
            );
            assert!(node.right.contains(&child.left));
            assert!(node.right.contains(&child.right));
            assert_eq!(
                node.right.leaf_count(),
                child.left.leaf_count() + child.right.leaf_count()
            );
            assert_eq!(calculated_leaf_count, node.right.leaf_count());
            calculated_leaf_count
        };

        left_count + right_count
    }

    pub(super) fn validate_tree_topology(nodes: &AVec<SahTreeNodeWide>, node_id: u32) -> u32 {
        let node = &nodes[node_id as usize];

        let left_count = if node.left.is_leaf() {
            1
        } else {
            let calculated_leaf_count = Self::validate_tree_topology(nodes, node.left.children);
            let child = &nodes[node.left.children as usize];
            assert_eq!(
                node.left.leaf_count(),
                child.left.leaf_count() + child.right.leaf_count()
            );
            assert_eq!(node.left.leaf_count(), calculated_leaf_count);
            calculated_leaf_count
        };

        let right_count = if node.right.is_leaf() {
            1
        } else {
            let calculated_leaf_count = Self::validate_tree_topology(nodes, node.right.children);
            let child = &nodes[node.right.children as usize];
            assert_eq!(
                node.right.leaf_count(),
                child.left.leaf_count() + child.right.leaf_count()
            );
            assert_eq!(calculated_leaf_count, node.right.leaf_count());
            calculated_leaf_count
        };

        left_count + right_count
    }

    pub fn remove(&mut self) {}

    // pub fn quality_metric(&self) -> Real {
    //     let mut metric = 0.0;
    //     for i in 0..self.nodes.len() {
    //         if !self.nodes[i].is_leaf() {
    //             metric += self.sah_cost(i);
    //         }
    //     }
    //     metric
    // }
    //
    // pub(super) fn sah_cost(&self, node_id: usize) -> Real {
    //     let node = &self.nodes[node_id];
    //
    //     if node.leaf_count() == 1 {
    //         // This is a leaf.
    //         1.0
    //     } else {
    //         let left = &self.nodes[node.children[0] as usize];
    //         let right = &self.nodes[node.children[1] as usize];
    //         let my_vol = node.aabb.volume();
    //         let left_vol = left.aabb.volume();
    //         let right_vol = right.aabb.volume();
    //         (left_vol * left.leaf_count() as Real + right_vol * right.leaf_count() as Real) / my_vol
    //     }
    // }
}
