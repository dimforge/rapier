use crate::math::Real;
use parry::bounding_volume::BoundingVolume;

use super::SahTree;

impl SahTree {
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

    pub fn traverse_bvtt(
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
                    self.traverse_bvtt(other, (l1, l2), f);
                }
                if left1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt(other, (l1, r2), f);
                }
                if right1.aabb.intersects(&left2.aabb) {
                    self.traverse_bvtt(other, (r1, l2), f);
                }
                if right1.aabb.intersects(&right2.aabb) {
                    self.traverse_bvtt(other, (r1, r2), f);
                }
            }
            (true, false) => {
                let left2 = &self.nodes[l2 as usize];
                let right2 = &self.nodes[r2 as usize];

                if left2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt(other, (a, l2), f);
                }
                if right2.aabb.intersects(&node1.aabb) {
                    self.traverse_bvtt(other, (a, r2), f);
                }
            }
            (false, true) => {
                let left1 = &self.nodes[l1 as usize];
                let right1 = &self.nodes[r1 as usize];

                if left1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt(other, (l1, b), f);
                }
                if right1.aabb.intersects(&node2.aabb) {
                    self.traverse_bvtt(other, (r1, b), f);
                }
            }
            (true, true) => {
                f(node1.children, node2.children);
            }
        }
    }
}
