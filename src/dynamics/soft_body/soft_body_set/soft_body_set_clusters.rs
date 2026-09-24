//! The cluster paths of a `SoftBodySet`: adding, removing and dissolving clusters.
use super::soft_body_set_proxies::spawn_proxy;
use super::{SoftBodyIslandEvent, SoftBodySet};
use crate::alloc_prelude::*;
use crate::dynamics::soft_body::{SoftBodyCluster, SoftBodyHandle, SoftClusterRemoval};
use crate::dynamics::{
    ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use crate::geometry::ColliderSet;
use crate::math::Rotation;

impl SoftBodySet {
    /// Adds a cluster to a soft body: a set of its particles (invalid indices ignored, duplicates
    /// merged) backed by a fresh [`crate::dynamics::RigidBodyType::SoftFrame`] proxy body that
    /// joints and colliders attach to. Returns its stable index, `None` if nothing valid remains.
    pub fn add_cluster(
        &mut self,
        handle: SoftBodyHandle,
        particles: &[u32],
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Option<u32> {
        let sb = self.bodies.get_mut(handle.0)?;
        let n = sb.particles.len() as u32;
        let mut list: Vec<u32> = particles.iter().copied().filter(|&v| v < n).collect();
        list.sort_unstable();
        list.dedup();
        if list.is_empty() {
            return None;
        }
        let index = sb.clusters.len() as u32;
        let proxy = spawn_proxy(&sb.particle_settings, sb.user_data, handle, index, bodies);
        for &v in &list {
            sb.cluster_refs[v as usize] += 1;
        }
        let cell = sb.matching_cell(&list);
        sb.clusters.push(SoftBodyCluster {
            particles: list,
            proxy,
            meshes: Vec::new(),
            rotation: Rotation::IDENTITY,
            cell,
            shape_matching: false,
            shape_matching_target: None,
            prev_shape_matching_target: None,
            last_gather: Default::default(),
            shape_impulses: Vec::new(),
        });
        sb.modified = true;
        // Update right away: a joint attached before the next step must see the cluster's
        // real frame and reduced mass.
        Self::update_cluster_proxies(sb, bodies, colliders);
        self.island_events.push(SoftBodyIslandEvent { handle });
        Some(index)
    }

    /// Removes the `cluster`-th cluster of a soft body, its proxy rigid body, and the particles
    /// only this cluster covered (with their elements and attachments); removing the last cluster
    /// removes the body. Returns what was deleted, or `None` if the body or cluster is missing.
    #[allow(clippy::too_many_arguments)]
    pub fn remove_cluster(
        &mut self,
        handle: SoftBodyHandle,
        cluster: u32,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftClusterRemoval> {
        let (counts, proxy) = self.dissolve_cluster(handle, cluster, islands, bodies, colliders)?;
        // The proxy body itself, with its own colliders and joints (the dissolution above
        // tombstoned the cluster, so the removal hook no-ops).
        bodies.remove(
            proxy,
            islands,
            colliders,
            impulse_joints,
            multibody_joints,
            self,
            true,
        );
        Some(counts)
    }

    /// The proxy-removal hook of [`RigidBodySet::remove`]: dissolves the cluster the removed
    /// proxy stood for. The proxy body itself (and its colliders and joints) is removed by the
    /// caller.
    pub(crate) fn on_proxy_removed(
        &mut self,
        handle: SoftBodyHandle,
        cluster: u32,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        let _ = self.dissolve_cluster(handle, cluster, islands, bodies, colliders);
    }

    /// Dissolves a cluster: tombstones it, removes the particles it uniquely covered (with their
    /// elements and attachments), rebuilds the surface shape, re-points the root body if needed,
    /// and removes the body with its last cluster. Keeps the proxy body; returns counts and its
    /// handle.
    fn dissolve_cluster(
        &mut self,
        handle: SoftBodyHandle,
        cluster: u32,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Option<(SoftClusterRemoval, RigidBodyHandle)> {
        // First pass, on the soft body alone: tombstone the cluster and remove the particles
        // only it covered (with their elements and attachments).
        let (mut counts, proxy, root, last) = {
            let sb = self.bodies.get_mut(handle.0)?;
            let c = sb.clusters.get_mut(cluster as usize)?;
            if !c.is_live() {
                return None;
            }
            let (particles, proxy) = c.tombstone();

            // Reference counts: the particles only this cluster covered die.
            let mut dead = alloc::vec![false; sb.particles.len()];
            for &v in &particles {
                if let Some(r) = sb.cluster_refs.get_mut(v as usize) {
                    *r = r.saturating_sub(1);
                    if *r == 0 {
                        dead[v as usize] = true;
                    }
                }
            }
            let (counts, _) = sb.remove_dead_particles(&dead);

            // The root body follows the first live cluster when cluster 0 goes.
            if sb.root_body == proxy {
                let next_root = sb
                    .live_clusters()
                    .next()
                    .map(|(_, c)| c.proxy)
                    .unwrap_or(RigidBodyHandle::invalid());
                sb.root_body = next_root;
            }
            let last = sb.num_live_clusters() == 0;
            (counts, proxy, sb.root_body, last)
        };
        counts.soft_body_removed = last;

        if last {
            // The last cluster: the soft body goes with it (its proxy is removed by the caller).
            let sb = self.bodies.remove(handle.0).unwrap();
            islands.persistent.unlink_soft_body_attachments(handle);
            islands.persistent.unlink_soft_body_proxy_chain(handle);
            self.attached_to_stale |= !sb.attachments.is_empty();
            return Some((counts, proxy));
        }

        if counts.particles > 0 {
            // The surface shape lost the dead particles' vertices: rebuild it.
            Self::update_colliders(&self.bodies[handle.0], bodies, colliders, true);
        }
        self.attached_to_stale |= counts.attachments > 0;
        self.island_events.push(SoftBodyIslandEvent { handle });
        if let Some(rb) = bodies.get_mut(root) {
            rb.wake_up(true);
        }
        Some((counts, proxy))
    }
}
