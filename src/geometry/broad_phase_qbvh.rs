use crate::geometry::{BroadPhasePairEvent, ColliderHandle, ColliderPair, ColliderSet};
use parry::math::Real;
use parry::partitioning::Qbvh;
use parry::partitioning::QbvhUpdateWorkspace;
use parry::query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct BroadPhaseQbvh {
    qbvh: Qbvh<ColliderHandle>,
    stack: Vec<(u32, u32)>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    workspace: QbvhUpdateWorkspace,
}

impl Default for BroadPhaseQbvh {
    fn default() -> Self {
        Self::new()
    }
}

impl BroadPhaseQbvh {
    pub fn new() -> Self {
        Self {
            qbvh: Qbvh::new(),
            stack: vec![],
            workspace: QbvhUpdateWorkspace::default(),
        }
    }

    #[allow(dead_code)] // This broad-phase is just experimental right now.
    pub fn update(
        &mut self,
        prediction_distance: Real,
        colliders: &ColliderSet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        let margin = 0.01;

        if modified_colliders.is_empty() {
            return;
        }

        // Visitor to find collision pairs.
        let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
            |co1: &ColliderHandle, co2: &ColliderHandle| {
                events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(*co1, *co2)));
                true
            },
        );

        let full_rebuild = self.qbvh.raw_nodes().is_empty();

        if full_rebuild {
            self.qbvh.clear_and_rebuild(
                colliders.iter().map(|(handle, collider)| {
                    (
                        handle,
                        collider.compute_collision_aabb(prediction_distance / 2.0),
                    )
                }),
                margin,
            );
            self.qbvh
                .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
        } else {
            for modified in modified_colliders {
                self.qbvh.pre_update_or_insert(*modified);
            }

            for removed in removed_colliders {
                self.qbvh.remove(*removed);
            }

            let _ = self.qbvh.refit(margin, &mut self.workspace, |handle| {
                colliders[*handle].compute_collision_aabb(prediction_distance / 2.0)
            });
            self.qbvh
                .traverse_modified_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
            self.qbvh.rebalance(margin, &mut self.workspace);
        }
    }
}
