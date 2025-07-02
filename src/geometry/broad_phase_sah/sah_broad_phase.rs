use crate::dynamics::RigidBodySet;
use crate::geometry::broad_phase_sah::sah_binned_build_incremental::BinnedRebuildState;
use crate::geometry::broad_phase_sah::sah_tree::SahWorkspace;
use crate::geometry::broad_phase_sah::SahTree;
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderHandle, ColliderPair, ColliderSet, Ray,
};
use parry::math::Real;
use parry::utils::hashmap::{Entry, HashMap};

#[derive(Default, Clone)]
pub struct BroadPhaseSah {
    tree: SahTree,
    workspace: SahWorkspace,
    pairs: HashMap<(ColliderHandle, ColliderHandle), u32>,
    frame_index: u32,
    rebuild_state: BinnedRebuildState,
    rebuild_strategy: SahRebuildStrategy,
}

#[derive(Default, PartialEq, Eq, Copy, Clone)]
pub enum SahRebuildStrategy {
    #[default]
    SubtreeOptimizer,
    IncrementalBinning,
    None,
}

impl BroadPhaseSah {
    pub fn new(rebuild_strategy: SahRebuildStrategy) -> Self {
        Self {
            rebuild_strategy,
            ..Default::default()
        }
    }

    fn update_with_strategy(
        &mut self,
        _dt: Real,
        prediction_distance: Real,
        colliders: &mut ColliderSet,
        _bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        _removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
        strategy: SahRebuildStrategy,
    ) {
        self.frame_index = self.frame_index.overflowing_add(1).0;
        if modified_colliders.is_empty() {
            return;
        }

        let first_pass = self.tree.is_empty();

        let t0 = std::time::Instant::now();
        for modified in modified_colliders {
            let aabb = colliders[*modified].compute_collision_aabb(prediction_distance / 2.0);
            self.tree
                .pre_update_or_insert(aabb, modified.into_raw_parts().0);
        }

        if first_pass {
            println!("Pre-update: {}", t0.elapsed().as_secs_f32() * 1000.0);
        }

        // if strategy == RebuildStrategy::SubtreeOptimizer && first_pass {
        //     println!("Tree depth after insert: {}", self.tree.subtree_height(0));
        //     let t0 = std::time::Instant::now();
        //     self.tree.rebuild(&mut self.workspace);
        //     println!("Rebuild: {}", t0.elapsed().as_secs_f32() * 1000.0);
        //     self.tree.refit(&mut self.workspace);
        //     println!("Tree depth after rebuild: {}", self.tree.subtree_height(0));
        // }
        //
        // for removed in removed_colliders {
        //     self.tree.remove(*removed);
        // }

        // let t0 = std::time::Instant::now();
        let need_refit = match strategy {
            SahRebuildStrategy::SubtreeOptimizer => {
                self.tree.optimize_incremental(&mut self.workspace);
                true
            }
            SahRebuildStrategy::IncrementalBinning => {
                !self.rebuild_state.step_rebuild(&mut self.tree)
            }
            SahRebuildStrategy::None => true,
        };
        // println!(
        //     "Incremental optimization: {}",
        //     t0.elapsed().as_secs_f32() * 1000.0
        // );

        // NOTE: we run refit after optimization so we can skip updating internal nodes during
        //       optimization, and so we can reorder the tree in memory (in depth-first order)
        //       to make it more cache friendly after the rebuild shuffling everything around.
        // let t0 = std::time::Instant::now();
        if need_refit {
            if strategy == SahRebuildStrategy::SubtreeOptimizer {
                self.tree.refit(&mut self.workspace);
            } else {
                self.tree.refit_without_opt();
            }
        }

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
            let Some((_, mut handle1)) = colliders.get_unknown_gen(co1) else {
                unreachable!();
                return;
            };
            let Some((_, mut handle2)) = colliders.get_unknown_gen(co2) else {
                unreachable!();
                return;
            };

            if co1 > co2 {
                std::mem::swap(&mut handle1, &mut handle2);
            }

            match self.pairs.entry((handle1, handle2)) {
                Entry::Occupied(e) => *e.into_mut() = self.frame_index,
                Entry::Vacant(e) => {
                    e.insert(self.frame_index);
                    events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                        handle1, handle2,
                    )));
                }
            }
        };

        // let t0 = std::time::Instant::now();
        // println!("Traverse");
        self.tree
            .traverse_bvtt_single_tree(&mut self.workspace, &mut pairs_collector);
        // println!("Detection: {}", t0.elapsed().as_secs_f32() * 1000.0);
        // println!(">>>>>> Num events: {}", events.iter().len());

        // Find outdated entries.
        // TODO PERF:
        // Currently, the narrow-phase isn’t capable of removing its own outdated
        // collision pairs. So we need to run a pass here to find aabbs that are
        // no longer overlapping. This, and the pair deduplication happening in
        // the `pairs_collector` is expensive and should be done more efficiently
        // by the narrow-phase itself (or islands) once we rework it.
        //
        // let t0 = std::time::Instant::now();
        self.pairs.retain(|(h0, h1), timestamp| {
            if *timestamp != self.frame_index {
                let Some(data0) = self.tree.leaf_data.get_unknown_gen(h0.into_raw_parts().0) else {
                    return false;
                };
                let Some(data1) = self.tree.leaf_data.get_unknown_gen(h1.into_raw_parts().0) else {
                    return false;
                };

                let node0 =
                    self.tree.nodes[data0.node as usize].as_array()[data0.left_or_right as usize];
                let node1 =
                    self.tree.nodes[data1.node as usize].as_array()[data1.left_or_right as usize];
                if (!SahTree::CHANGE_DETECTION_ENABLED
                    || node0.data.is_changed()
                    || node1.data.is_changed())
                    && !node0.intersects(&node1)
                {
                    events.push(BroadPhasePairEvent::DeletePair(ColliderPair::new(*h0, *h1)));
                    false
                } else {
                    true
                }
            } else {
                // If the timestamps match, we already saw this pair during traversal.
                // There can be rare occurrences where the timestamp will be equal
                // even though we didn’t see the pair during traversal. This happens
                // if the frame index overflowed. But this is fine, we’ll catch it
                // in another frame.
                true
            }
        });
        // println!(
        //     "Post-filtering: {} (added pairs: {}, removed pairs: {})",
        //     t0.elapsed().as_secs_f32() * 1000.0,
        //     added_pairs,
        //     removed_pairs
        // );
    }
}

impl BroadPhase for BroadPhaseSah {
    fn update(
        &mut self,
        dt: Real,
        prediction_distance: Real,
        colliders: &mut ColliderSet,
        bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        self.update_with_strategy(
            dt,
            prediction_distance,
            colliders,
            bodies,
            modified_colliders,
            removed_colliders,
            events,
            self.rebuild_strategy,
        )
    }
}

impl BroadPhaseSah {
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_toi: Real,
        colliders: &ColliderSet,
    ) -> Option<(ColliderHandle, Real)> {
        let best = self.tree.cast_ray(ray, max_toi, |id| {
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
