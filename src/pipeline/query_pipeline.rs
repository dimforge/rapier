use crate::dynamics::RigidBodySet;
use crate::geometry::{Collider, ColliderHandle, ColliderSet, Ray, RayIntersection, AABB, WAABB};
use crate::math::{Point, Vector};
use ncollide::bounding_volume::BoundingVolume;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct NodeIndex {
    index: u32, // Index of the addressed node in the `children` array.
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
struct WAABBHierarchyNodeChildren {
    waabb: WAABB,
    // Index of the children of the 4 nodes represented by self.
    // If this is a leaf, it contains the proxy ids instead.
    grand_children: [u32; 4],
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

struct WAABBHierarchy {
    children: Vec<WAABBHierarchyNodeChildren>,
    dirty: Vec<bool>, // TODO: use a bitvec/Vob and check it does not break cross-platform determinism.
    proxies: Vec<ColliderNodeIndex>,
}

impl WAABBHierarchy {
    pub fn new() -> Self {
        WAABBHierarchy {
            children: Vec::new(),
            dirty: Vec::new(),
            proxies: Vec::new(),
        }
    }

    pub fn clear_and_rebuild(&mut self, colliders: &ColliderSet) {
        self.children.clear();
        self.dirty.clear();
        self.proxies.clear();

        // Create proxies.
        let mut indices = Vec::with_capacity(colliders.len());
        let mut proxies = vec![ColliderNodeIndex::invalid(); colliders.len()];
        for (handle, collider) in colliders.iter() {
            let index = handle.into_raw_parts().0;
            if proxies.len() < handle.into_raw_parts().0 {
                proxies.resize(index + 1, ColliderNodeIndex::invalid());
            }

            proxies[index].handle = handle;
            indices.push(index);
        }

        // Compute AABBs.
        let mut aabbs = vec![AABB::new_invalid(); proxies.len()];
        for (handle, collider) in colliders.iter() {
            let index = handle.into_raw_parts().0;
            let aabb = collider.compute_aabb();
            aabbs[index] = aabb;
        }

        // Build the tree recursively.
        let root_node = WAABBHierarchyNodeChildren {
            waabb: WAABB::new_invalid(),
            grand_children: [1; 4],
            parent: NodeIndex::invalid(),
            leaf: false,
        };

        self.children.push(root_node);
        let root_id = NodeIndex::new(0, 0);
        let (_, aabb) = self.do_recurse_build(&mut indices, &aabbs, root_id);
        self.children[0].waabb = WAABB::splat(aabb);
    }

    fn do_recurse_build(
        &mut self,
        indices: &mut [usize],
        aabbs: &[AABB],
        parent: NodeIndex,
    ) -> (u32, AABB) {
        // Leaf case.
        if indices.len() <= 4 {
            let my_id = self.children.len();
            let mut my_aabb = AABB::new_invalid();
            let mut leaf_aabbs = [AABB::new_invalid(); 4];
            let mut proxy_ids = [u32::MAX; 4];

            for (k, id) in indices.iter().enumerate() {
                my_aabb.merge(&aabbs[*id]);
                leaf_aabbs[k] = aabbs[*id];
                proxy_ids[k] = *id as u32;
            }

            let node = WAABBHierarchyNodeChildren {
                waabb: WAABB::from(leaf_aabbs),
                grand_children: proxy_ids,
                parent,
                leaf: true,
            };

            self.children.push(node);
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
        let (left, right) = split_indices_wrt_dim(indices, &aabbs, subdiv_dims[0]);
        let (left_bottom, left_top) = split_indices_wrt_dim(left, &aabbs, subdiv_dims[1]);
        let (right_bottom, right_top) = split_indices_wrt_dim(right, &aabbs, subdiv_dims[1]);

        let node = WAABBHierarchyNodeChildren {
            waabb: WAABB::new_invalid(),
            grand_children: [0; 4], // Will be set after the recursive call
            parent,
            leaf: false,
        };

        let id = self.children.len() as u32;
        self.children.push(node);

        // Recurse!
        let a = self.do_recurse_build(left_bottom, aabbs, NodeIndex::new(id, 0));
        let b = self.do_recurse_build(left_top, aabbs, NodeIndex::new(id, 1));
        let c = self.do_recurse_build(right_bottom, aabbs, NodeIndex::new(id, 2));
        let d = self.do_recurse_build(right_top, aabbs, NodeIndex::new(id, 3));

        // Now we know the indices of the grand-children.
        self.children[id as usize].grand_children = [a.0, b.0, c.0, d.0];
        self.children[id as usize].waabb = WAABB::from([a.1, b.1, c.1, d.1]);

        // TODO: will this chain of .merged be properly optimized?
        let my_aabb = a.1.merged(&b.1).merged(&c.1).merged(&c.1);
        (id, my_aabb)
    }
}

fn split_indices_wrt_dim<'a>(
    indices: &'a mut [usize],
    aabbs: &[AABB],
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

        if center[dim] > center[dim] {
            indices.swap(icurr, ilast);
            ilast -= 1;
        } else {
            icurr += 1;
        }
    }

    indices.split_at_mut(icurr)
}

/// A pipeline for performing queries on all the colliders of a scene.
pub struct QueryPipeline {
    // hierarchy: WAABBHierarchy,
}

impl Default for QueryPipeline {
    fn default() -> Self {
        Self::new()
    }
}

impl QueryPipeline {
    /// Initializes an empty query pipeline.
    pub fn new() -> Self {
        Self {
            // hierarchy: WAABBHierarchy::new(),
        }
    }

    /// Update the acceleration structure on the query pipeline.
    pub fn update(&mut self, _bodies: &mut RigidBodySet, _colliders: &mut ColliderSet) {}

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `f32::MAX` for an unbounded ray.
    pub fn cast_ray<'a>(
        &self,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: f32,
    ) -> Option<(ColliderHandle, &'a Collider, RayIntersection)> {
        let mut best = f32::MAX;
        let mut result = None;

        // FIXME: this is a brute-force approach.
        for (handle, collider) in colliders.iter() {
            if let Some(inter) = collider.shape().cast_ray(collider.position(), ray, max_toi) {
                if inter.toi < best {
                    best = inter.toi;
                    result = Some((handle, collider, inter));
                }
            }
        }

        result
    }

    /// Find the all intersections between a ray and a set of collider and passes them to a callback.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `f32::MAX` for an unbounded ray.
    /// - `callback`: function executed on each collider for which a ray intersection has been found.
    ///   There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///   this method will exit early, ignory any further raycast.
    pub fn interferences_with_ray<'a>(
        &self,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: f32,
        mut callback: impl FnMut(ColliderHandle, &'a Collider, RayIntersection) -> bool,
    ) {
        // FIXME: this is a brute-force approach.
        for (handle, collider) in colliders.iter() {
            if let Some(inter) = collider.shape().cast_ray(collider.position(), ray, max_toi) {
                if !callback(handle, collider, inter) {
                    return;
                }
            }
        }
    }
}
