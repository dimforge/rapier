use crate::geometry::broad_phase_sah::{SahTree, SahWorkspace};
use crate::geometry::{ColliderHandle, ColliderSet};
use crate::math::Real;
use parry::query::{Ray, RayCast};

impl SahTree {
    pub fn cast_ray(
        &self,
        workspace: &mut SahWorkspace,
        ray: &Ray,
        max_toi: Real,
        primitive_check: impl Fn(u32) -> Real,
    ) -> (u32, Real) {
        // workspace.traversal_stack.clear();
        let mut curr_id = 0;
        let mut best_toi = max_toi;
        let mut best_primitive = u32::MAX;

        loop {
            let node = &self.nodes[curr_id as usize];
            let mut left = &node.left;
            let mut right = &node.right;

            let mut toi_left = left
                .aabb()
                .cast_local_ray(&ray, best_toi, true)
                .unwrap_or(Real::MAX);
            let mut toi_right = right
                .aabb()
                .cast_local_ray(&ray, best_toi, true)
                .unwrap_or(Real::MAX);

            if toi_left > toi_right {
                std::mem::swap(&mut left, &mut right);
                std::mem::swap(&mut toi_left, &mut toi_right);
            }

            let mut found_next = false;

            if toi_left < best_toi {
                if left.is_leaf() {
                    let primitive_toi = primitive_check(left.children);
                    if primitive_toi < best_toi {
                        best_toi = primitive_toi;
                        best_primitive = left.children;
                    }
                } else {
                    curr_id = left.children;
                    found_next = true;
                }
            }

            if toi_right < best_toi {
                if right.is_leaf() {
                    let primitive_toi = primitive_check(right.children);
                    if primitive_toi < best_toi {
                        best_toi = primitive_toi;
                        best_primitive = right.children;
                    }
                } else if found_next {
                    workspace.traversal_stack.push(right.children);
                } else {
                    curr_id = right.children;
                }
            }

            if !found_next {
                if let Some(next) = workspace.traversal_stack.pop() {
                    curr_id = next;
                    continue;
                } else {
                    // Traversal is complete.
                    return (best_primitive, best_toi);
                }
            }
        }
    }
}
