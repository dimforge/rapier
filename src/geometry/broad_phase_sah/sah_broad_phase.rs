use crate::dynamics::RigidBodySet;
use crate::geometry::broad_phase_sah::sah_tree::SahWorkspace;
use crate::geometry::broad_phase_sah::SahTree;
use crate::geometry::{BroadPhase, BroadPhasePairEvent, ColliderHandle, ColliderPair, ColliderSet};
use parry::math::Real;
use parry::query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor;

#[derive(Default, Clone)]
pub struct BroadPhaseSah {
    tree: SahTree,
    workspace: SahWorkspace,
}

impl BroadPhase for BroadPhaseSah {
    fn update(
        &mut self,
        _dt: Real,
        prediction_distance: Real,
        colliders: &mut ColliderSet,
        _bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        if modified_colliders.is_empty() {
            return;
        }

        let first_pass = self.tree.is_empty();

        let t0 = std::time::Instant::now();
        for modified in modified_colliders {
            let aabb = colliders[*modified].compute_collision_aabb(prediction_distance / 2.0);
            self.tree
                .pre_update_or_insert(aabb, modified.into_raw_parts().into());
        }
        println!("Pre-update: {}", t0.elapsed().as_secs_f32() * 1000.0);

        if first_pass {
            let t0 = std::time::Instant::now();
            self.tree.rebuild(&mut self.workspace);
            self.tree.refit(&mut self.workspace);
            println!("Rebuild: {}", t0.elapsed().as_secs_f32() * 1000.0);
        }

        // for removed in removed_colliders {
        //     self.tree.remove(*removed);
        // }

        let t0 = std::time::Instant::now();
        self.tree.optimize_incremental(&mut self.workspace);
        // self.tree.rebuild(&mut self.workspace);
        println!(
            "Rebuild incremental: {}",
            t0.elapsed().as_secs_f32() * 1000.0
        );

        // NOTE: we run refit after optimization so we can skip updating internal nodes during
        //       optimization, and so we can reorder the tree in memory to make it more cache
        //       friendly after the rebuild shuffling everything around.
        let t0 = std::time::Instant::now();
        self.tree.refit(&mut self.workspace);
        println!("Refit: {}", t0.elapsed().as_secs_f32() * 1000.0);

        // self.tree.assert_is_depth_first();
        // println!("Checking well formed.");
        // self.tree.assert_well_formed();
        println!(
            "Is well formed. Tree height: {}",
            self.tree.subtree_height(0)
        );

        let mut pairs_collector = |co1: [u32; 2], co2: [u32; 2]| {
            // NOTE: we don’t emit self-intersections.
            if co1[0] != co2[0] {
                let co1 = ColliderHandle::from_raw_parts(co1[0], co1[1]);
                let co2 = ColliderHandle::from_raw_parts(co2[0], co2[1]);
                events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(co1, co2)));
            }
        };

        let t0 = std::time::Instant::now();
        // self.tree
        //     .traverse_bvtt_recursive(&self.tree, (0, 0), &mut pairs_collector);
        self.tree
            .traverse_bvtt_changed_self((0, 0), &mut pairs_collector);
        println!("Detection: {}", t0.elapsed().as_secs_f32() * 1000.0);
        println!(">>>>>> Num events: {}", events.iter().len());
    }
}
