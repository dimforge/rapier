use crate::geometry::{ColliderHandle, ColliderSet, Ray, AABB};
use crate::geometry::{WRay, WAABB};
use crate::math::{Point, Vector};
use crate::simd::{SimdFloat, SIMD_WIDTH};
use ncollide::bounding_volume::BoundingVolume;
use simba::simd::{SimdBool, SimdValue};
use std::ops::Range;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct NodeIndex {
    index: u32, // Index of the addressed node in the `nodes` array.
    lane: u8,   // SIMD lane of the addressed node.
}

impl NodeIndex {
    fn new(index: u32, lane: u8) -> Self {
        Self { index, lane }
    }

    fn invalid() -> Self {
        Self {
            index: u32::MAX,
            lane: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct WQuadtreeNode {
    waabb: WAABB,
    // Index of the nodes of the 4 nodes represented by self.
    // If this is a leaf, it contains the proxy ids instead.
    children: [u32; 4],
    parent: NodeIndex,
    leaf: bool, // TODO: pack this with the NodexIndex.lane?
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct ColliderNodeIndex {
    node: NodeIndex,
    handle: ColliderHandle, // The collider handle. TODO: only set the collider generation here?
}

impl ColliderNodeIndex {
    fn invalid() -> Self {
        Self {
            node: NodeIndex::invalid(),
            handle: ColliderSet::invalid_handle(),
        }
    }
}

pub struct WQuadtree {
    nodes: Vec<WQuadtreeNode>,
    dirty: Vec<bool>, // TODO: use a bitvec/Vob and check it does not break cross-platform determinism.
    proxies: Vec<ColliderNodeIndex>,
}

impl WQuadtree {
    pub fn new() -> Self {
        WQuadtree {
            nodes: Vec::new(),
            dirty: Vec::new(),
            proxies: Vec::new(),
        }
    }

    pub fn clear_and_rebuild(&mut self, colliders: &ColliderSet) {
        self.nodes.clear();
        self.dirty.clear();
        self.proxies.clear();

        // Create proxies.
        let mut indices = Vec::with_capacity(colliders.len());
        self.proxies = vec![ColliderNodeIndex::invalid(); colliders.len()];

        for (handle, collider) in colliders.iter() {
            let index = handle.into_raw_parts().0;
            if self.proxies.len() < index {
                self.proxies.resize(index + 1, ColliderNodeIndex::invalid());
            }

            self.proxies[index].handle = handle;
            indices.push(index);
        }

        // Compute AABBs.
        let mut aabbs = vec![AABB::new_invalid(); self.proxies.len()];
        for (handle, collider) in colliders.iter() {
            let index = handle.into_raw_parts().0;
            let aabb = collider.compute_aabb();
            aabbs[index] = aabb;
        }

        // Build the tree recursively.
        let root_node = WQuadtreeNode {
            waabb: WAABB::new_invalid(),
            children: [1, u32::MAX, u32::MAX, u32::MAX],
            parent: NodeIndex::invalid(),
            leaf: false,
        };

        self.nodes.push(root_node);
        let root_id = NodeIndex::new(0, 0);
        let (_, aabb) = self.do_recurse_build(&mut indices, &aabbs, root_id);
        self.nodes[0].waabb = WAABB::from([
            aabb,
            AABB::new_invalid(),
            AABB::new_invalid(),
            AABB::new_invalid(),
        ]);
    }

    fn do_recurse_build(
        &mut self,
        indices: &mut [usize],
        aabbs: &[AABB],
        parent: NodeIndex,
    ) -> (u32, AABB) {
        // Leaf case.
        if indices.len() <= 4 {
            let my_id = self.nodes.len();
            let mut my_aabb = AABB::new_invalid();
            let mut leaf_aabbs = [AABB::new_invalid(); 4];
            let mut proxy_ids = [u32::MAX; 4];

            for (k, id) in indices.iter().enumerate() {
                my_aabb.merge(&aabbs[*id]);
                leaf_aabbs[k] = aabbs[*id];
                proxy_ids[k] = *id as u32;
            }

            let node = WQuadtreeNode {
                waabb: WAABB::from(leaf_aabbs),
                children: proxy_ids,
                parent,
                leaf: true,
            };

            self.nodes.push(node);
            return (my_id as u32, my_aabb);
        }

        // Compute the center and variance along each dimension.
        // In 3D we compute the variance to not-subdivide the dimension with lowest variance.
        // Therefore variance computation is not needed in 2D because we only have 2 dimension
        // to split in the first place.
        let mut center = Point::origin();
        #[cfg(feature = "dim3")]
        let mut variance = Vector::zeros();

        let denom = 1.0 / (indices.len() as f32);

        for i in &*indices {
            let coords = aabbs[*i].center().coords;
            center += coords * denom;
            #[cfg(feature = "dim3")]
            {
                variance += coords.component_mul(&coords) * denom;
            }
        }

        #[cfg(feature = "dim3")]
        {
            variance = variance - center.coords.component_mul(&center.coords);
        }

        // Find the axis with minimum variance. This is the axis along
        // which we are **not** subdividing our set.
        let mut subdiv_dims = [0, 1];
        #[cfg(feature = "dim3")]
        {
            let min = variance.imin();
            subdiv_dims[0] = (min + 1) % 3;
            subdiv_dims[1] = (min + 2) % 3;
        }

        // Split the set along the two subdiv_dims dimensions.
        // TODO: should we split wrt. the median instead of the average?
        // TODO: we should ensure each subslice contains at least 4 elements each (or less if
        // indices has less than 16 elements in the first place.
        let (left, right) = split_indices_wrt_dim(indices, &aabbs, &center, subdiv_dims[0]);

        let (left_bottom, left_top) = split_indices_wrt_dim(left, &aabbs, &center, subdiv_dims[1]);
        let (right_bottom, right_top) =
            split_indices_wrt_dim(right, &aabbs, &center, subdiv_dims[1]);

        // println!(
        //     "Recursing on children: {}, {}, {}, {}",
        //     left_bottom.len(),
        //     left_top.len(),
        //     right_bottom.len(),
        //     right_top.len()
        // );

        let node = WQuadtreeNode {
            waabb: WAABB::new_invalid(),
            children: [0; 4], // Will be set after the recursive call
            parent,
            leaf: false,
        };

        let id = self.nodes.len() as u32;
        self.nodes.push(node);

        // Recurse!
        let a = self.do_recurse_build(left_bottom, aabbs, NodeIndex::new(id, 0));
        let b = self.do_recurse_build(left_top, aabbs, NodeIndex::new(id, 1));
        let c = self.do_recurse_build(right_bottom, aabbs, NodeIndex::new(id, 2));
        let d = self.do_recurse_build(right_top, aabbs, NodeIndex::new(id, 3));

        // Now we know the indices of the grand-nodes.
        self.nodes[id as usize].children = [a.0, b.0, c.0, d.0];
        self.nodes[id as usize].waabb = WAABB::from([a.1, b.1, c.1, d.1]);

        // TODO: will this chain of .merged be properly optimized?
        let my_aabb = a.1.merged(&b.1).merged(&c.1).merged(&d.1);
        (id, my_aabb)
    }

    pub fn cast_ray(&self, ray: &Ray, max_toi: f32) -> Vec<ColliderHandle> {
        let mut res = Vec::new();

        if self.nodes.is_empty() {
            return res;
        }

        // Special case for the root.
        let mut stack = vec![0u32];
        let wray = WRay::splat(*ray);
        let wmax_toi = SimdFloat::splat(max_toi);
        while let Some(inode) = stack.pop() {
            let node = self.nodes[inode as usize];
            let hits = node.waabb.intersects_ray(&wray, wmax_toi);
            let bitmask = hits.bitmask();

            for ii in 0..SIMD_WIDTH {
                if (bitmask & (1 << ii)) != 0 {
                    if node.leaf {
                        // We found a leaf!
                        // Unfortunately, invalid AABBs return a hit as well.
                        if let Some(proxy) = self.proxies.get(node.children[ii] as usize) {
                            res.push(proxy.handle);
                        }
                    } else {
                        // Internal node, visit the child.
                        // Un fortunately, we have this check because invalid AABBs
                        // return a hit as well.
                        if node.children[ii] as usize <= self.nodes.len() {
                            stack.push(node.children[ii]);
                        }
                    }
                }
            }
        }

        res
    }
}

struct WQuadtreeIncrementalBuilderStep {
    range: Range<usize>,
    parent: NodeIndex,
}

struct WQuadtreeIncrementalBuilder {
    quadtree: WQuadtree,
    to_insert: Vec<WQuadtreeIncrementalBuilderStep>,
    aabbs: Vec<AABB>,
    indices: Vec<usize>,
}

impl WQuadtreeIncrementalBuilder {
    pub fn new() -> Self {
        Self {
            quadtree: WQuadtree::new(),
            to_insert: Vec::new(),
            aabbs: Vec::new(),
            indices: Vec::new(),
        }
    }

    pub fn update_single_depth(&mut self) {
        if let Some(to_insert) = self.to_insert.pop() {
            let indices = &mut self.indices[to_insert.range];

            // Leaf case.
            if indices.len() <= 4 {
                let id = self.quadtree.nodes.len();
                let mut aabb = AABB::new_invalid();
                let mut leaf_aabbs = [AABB::new_invalid(); 4];
                let mut proxy_ids = [u32::MAX; 4];

                for (k, id) in indices.iter().enumerate() {
                    aabb.merge(&self.aabbs[*id]);
                    leaf_aabbs[k] = self.aabbs[*id];
                    proxy_ids[k] = *id as u32;
                }

                let node = WQuadtreeNode {
                    waabb: WAABB::from(leaf_aabbs),
                    children: proxy_ids,
                    parent: to_insert.parent,
                    leaf: true,
                };

                self.quadtree.nodes[to_insert.parent.index as usize].children
                    [to_insert.parent.lane as usize] = id as u32;
                self.quadtree.nodes[to_insert.parent.index as usize]
                    .waabb
                    .replace(to_insert.parent.lane as usize, aabb);
                self.quadtree.nodes.push(node);
                return;
            }

            // Compute the center and variance along each dimension.
            // In 3D we compute the variance to not-subdivide the dimension with lowest variance.
            // Therefore variance computation is not needed in 2D because we only have 2 dimension
            // to split in the first place.
            let mut center = Point::origin();
            #[cfg(feature = "dim3")]
            let mut variance = Vector::zeros();

            let denom = 1.0 / (indices.len() as f32);
            let mut aabb = AABB::new_invalid();

            for i in &*indices {
                let coords = self.aabbs[*i].center().coords;
                aabb.merge(&self.aabbs[*i]);
                center += coords * denom;
                #[cfg(feature = "dim3")]
                {
                    variance += coords.component_mul(&coords) * denom;
                }
            }

            #[cfg(feature = "dim3")]
            {
                variance = variance - center.coords.component_mul(&center.coords);
            }

            // Find the axis with minimum variance. This is the axis along
            // which we are **not** subdividing our set.
            let mut subdiv_dims = [0, 1];
            #[cfg(feature = "dim3")]
            {
                let min = variance.imin();
                subdiv_dims[0] = (min + 1) % 3;
                subdiv_dims[1] = (min + 2) % 3;
            }

            // Split the set along the two subdiv_dims dimensions.
            // TODO: should we split wrt. the median instead of the average?
            // TODO: we should ensure each subslice contains at least 4 elements each (or less if
            // indices has less than 16 elements in the first place.
            let (left, right) =
                split_indices_wrt_dim(indices, &self.aabbs, &center, subdiv_dims[0]);

            let (left_bottom, left_top) =
                split_indices_wrt_dim(left, &self.aabbs, &center, subdiv_dims[1]);
            let (right_bottom, right_top) =
                split_indices_wrt_dim(right, &self.aabbs, &center, subdiv_dims[1]);

            let node = WQuadtreeNode {
                waabb: WAABB::new_invalid(),
                children: [0; 4], // Will be set after the recursive call
                parent: to_insert.parent,
                leaf: false,
            };

            let id = self.quadtree.nodes.len() as u32;
            self.quadtree.nodes.push(node);

            // Recurse!
            let a = left_bottom.len();
            let b = a + left_top.len();
            let c = b + right_bottom.len();
            let d = c + right_top.len();
            self.to_insert.push(WQuadtreeIncrementalBuilderStep {
                range: 0..a,
                parent: NodeIndex::new(id, 0),
            });
            self.to_insert.push(WQuadtreeIncrementalBuilderStep {
                range: a..b,
                parent: NodeIndex::new(id, 1),
            });
            self.to_insert.push(WQuadtreeIncrementalBuilderStep {
                range: b..c,
                parent: NodeIndex::new(id, 2),
            });
            self.to_insert.push(WQuadtreeIncrementalBuilderStep {
                range: c..d,
                parent: NodeIndex::new(id, 3),
            });

            self.quadtree.nodes[to_insert.parent.index as usize].children
                [to_insert.parent.lane as usize] = id as u32;
            self.quadtree.nodes[to_insert.parent.index as usize]
                .waabb
                .replace(to_insert.parent.lane as usize, aabb);
        }
    }
}

fn split_indices_wrt_dim<'a>(
    indices: &'a mut [usize],
    aabbs: &[AABB],
    split_point: &Point<f32>,
    dim: usize,
) -> (&'a mut [usize], &'a mut [usize]) {
    let mut icurr = 0;
    let mut ilast = indices.len() - 1;

    // The loop condition we can just do 0..indices.len()
    // instead of the test icurr < ilast because we know
    // we will iterate exactly once per index.
    for _ in 0..indices.len() {
        let i = indices[icurr];
        let center = aabbs[i].center();

        if center[dim] > split_point[dim] {
            indices.swap(icurr, ilast);
            ilast -= 1;
        } else {
            icurr += 1;
        }
    }

    indices.split_at_mut(icurr)
}
