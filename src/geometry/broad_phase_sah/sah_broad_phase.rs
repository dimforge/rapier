use crate::dynamics::RigidBodySet;
use crate::geometry::broad_phase_sah::sah_tree::SahWorkspace;
use crate::geometry::broad_phase_sah::SahTree;
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderHandle, ColliderPair, ColliderSet, Ray,
};
use parry::math::Real;

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
        _removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        if modified_colliders.is_empty() {
            return;
        }

        let first_pass = self.tree.is_empty();

        // let t0 = std::time::Instant::now();
        for modified in modified_colliders {
            let aabb = colliders[*modified].compute_collision_aabb(prediction_distance / 2.0);
            self.tree
                .pre_update_or_insert(aabb, modified.into_raw_parts().0);
        }
        // println!("Pre-update: {}", t0.elapsed().as_secs_f32() * 1000.0);

        if first_pass {
            let t0 = std::time::Instant::now();
            self.tree.rebuild(&mut self.workspace);
            println!("Rebuild: {}", t0.elapsed().as_secs_f32() * 1000.0);
            self.tree.refit(&mut self.workspace);
        }

        // for removed in removed_colliders {
        //     self.tree.remove(*removed);
        // }

        // let t0 = std::time::Instant::now();
        self.tree.optimize_incremental(&mut self.workspace);
        // println!(
        //     "Rebuild incremental: {}",
        //     t0.elapsed().as_secs_f32() * 1000.0
        // );
        // NOTE: we run refit after optimization so we can skip updating internal nodes during
        //       optimization, and so we can reorder the tree in memory (in depth-first order)
        //       to make it more cache friendly after the rebuild shuffling everything around.
        // let t0 = std::time::Instant::now();
        self.tree.refit(&mut self.workspace);
        // println!("Refit: {}", t0.elapsed().as_secs_f32() * 1000.0);

        // println!(
        //     "leaf count: {}/{} (changed: {})",
        //     self.tree.leaf_count(),
        //     self.tree.reachable_leaf_count(0),
        //     self.tree.changed_leaf_count(0),
        // );

        // self.tree.assert_is_depth_first();
        // self.tree.assert_well_formed();
        // println!(
        //     "Is well formed. Tree height: {}",
        //     self.tree.subtree_height(0),
        // );
        // // println!("Tree quality: {}", self.tree.quality_metric());

        let mut pairs_collector = |co1: u32, co2: u32| {
            assert_ne!(co1, co2);
            // FIXME: store the generation number somewhere in the sah tree.
            let Some((_, handle1)) = colliders.get_unknown_gen(co1) else {
                unreachable!();
                return;
            };
            let Some((_, handle2)) = colliders.get_unknown_gen(co2) else {
                unreachable!();
                return;
            };
            events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                handle1, handle2,
            )));
        };

        let t0 = std::time::Instant::now();
        self.tree
            .traverse_bvtt_single_tree(&mut self.workspace, &mut pairs_collector);
        println!("Detection: {}", t0.elapsed().as_secs_f32() * 1000.0);
        // println!(">>>>>> Num events: {}", events.iter().len());
    }
}

impl BroadPhaseSah {
    pub fn cast_ray(
        &self,
        workspace: &mut SahWorkspace,
        ray: &Ray,
        max_toi: Real,
        colliders: &ColliderSet,
    ) -> Option<(ColliderHandle, Real)> {
        let best = self.tree.cast_ray(workspace, ray, max_toi, |id| {
            let Some(co) = colliders.get_unknown_gen(id) else {
                return Real::MAX;
            };
            co.0.shape
                .cast_ray(&co.0.position(), ray, Real::MAX, true)
                .unwrap_or(Real::MAX)
        });

        (best.1 < max_toi).then(|| (colliders.get_unknown_gen(best.0).unwrap().1, best.1))
    }
}
