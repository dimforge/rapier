//! Splitting clusters and soft bodies along the cracks a tear or a cut opened: a cluster comes
//! apart with the material it covers, its meshes and joints following the pieces, and a piece the
//! crack disconnected from the rest of the body becomes a soft body of its own.
use crate::alloc_prelude::*;
use crate::dynamics::soft_body::{
    SoftBody, SoftBodyCluster, SoftBodyHandle, SoftBodyPiece, SoftBodyTearEvent, SoftClusterSplit,
    SoftCollisionMesh, SoftJointMove, SoftMeshId, SoftMeshMapping, SoftMeshRef,
};
use crate::dynamics::{
    ImpulseJointHandle, ImpulseJointSet, IslandManager, RigidBodyHandle, RigidBodySet,
};
use crate::geometry::{ColliderHandle, ColliderSet};
use crate::math::{Pose, Real, Vector};
use super::soft_body_set_proxies::{clone_mesh_collider, spawn_proxy};
use super::{SoftBodyIslandEvent, SoftBodySet};

/// The poses the proxies had before a split, by proxy (a fresh proxy records the pose of the
/// proxy it was split from): what the joints and rigid colliders hung on them are re-based from
/// once the pieces' frames are known (see [`rebase_proxy_attachments`]).
#[derive(Default)]
pub(super) struct ProxyPoses(Vec<(RigidBodyHandle, Pose)>);

impl ProxyPoses {
    fn record(&mut self, proxy: RigidBodyHandle, pose: Pose) {
        if !self.0.iter().any(|(p, _)| *p == proxy) {
            self.0.push((proxy, pose));
        }
    }
}

/// The pieces a cluster or a body splits into: the seeded `components` of its `particles`, the
/// largest by `measure` first (ties to the one with the smallest particle), with the particles no
/// component holds (the parts the tear did not reach) joined to it.
fn pieces_keeping_the_largest(
    sb: &SoftBody,
    particles: &[u32],
    mut components: Vec<Vec<u32>>,
    measure: impl Fn(&SoftBody, &[u32]) -> Real,
) -> Vec<Vec<u32>> {
    let largest = (0..components.len())
        .max_by(|&i, &j| {
            measure(sb, &components[i])
                .total_cmp(&measure(sb, &components[j]))
                .then(j.cmp(&i))
        })
        .unwrap_or(0);
    let mut retained = components.remove(largest);
    let mut reached = vec![false; sb.num_particles()];
    for &v in components.iter().flatten().chain(&retained) {
        reached[v as usize] = true;
    }
    retained.extend(particles.iter().filter(|&&v| !reached[v as usize]));
    retained.sort_unstable();
    let mut pieces = vec![retained];
    pieces.extend(components);
    pieces
}

/// The rest centroid of the given particles (nominal masses): where the frame of a cluster over
/// them sits in the rest shape.
fn rest_centroid(sb: &SoftBody, particles: &[u32]) -> Vector {
    let mut com = Vector::ZERO;
    let mut mass = 0.0;
    for &v in particles {
        let p = &sb.particles[v as usize];
        com += p.rest_position * p.mass;
        mass += p.mass;
    }
    if mass > 0.0 {
        com / mass
    } else {
        com
    }
}

impl SoftBodySet {
    /// Splits every cluster of `handle` whose own graph the tear disconnected (see
    /// [`SoftBody::seeded_components`]): the heaviest piece keeps the cluster, every other becomes
    /// a new cluster. Returns whether some cluster was split.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn split_clusters_along(
        &mut self,
        handle: SoftBodyHandle,
        seeds: &[[u32; 2]],
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        event: &mut SoftBodyTearEvent,
        poses: &mut ProxyPoses,
    ) -> bool {
        let num_clusters = self.bodies[handle.0].clusters.len();
        let mut any = false;
        for ci in 0..num_clusters as u32 {
            let pieces = {
                let sb = &self.bodies[handle.0];
                let Some(cluster) = sb.cluster(ci) else {
                    continue;
                };
                let components = sb.seeded_components(Some(cluster.particles()), seeds);
                if components.len() < 2 {
                    continue;
                }
                pieces_keeping_the_largest(sb, cluster.particles(), components, SoftBody::mass_of)
            };
            self.split_cluster(
                handle,
                ci,
                &pieces,
                islands,
                bodies,
                colliders,
                impulse_joints,
                event,
                poses,
            );
            any = true;
        }
        any
    }

    /// Splits the `cluster`-th cluster of `handle` into `pieces` (disjoint sorted particle sets
    /// covering it): the first keeps the slot and proxy, the others get fresh clusters and proxies,
    /// meshes restricted per piece, joints following the nearest piece; frames not updated here.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn split_cluster(
        &mut self,
        handle: SoftBodyHandle,
        cluster: u32,
        pieces: &[Vec<u32>],
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        event: &mut SoftBodyTearEvent,
        poses: &mut ProxyPoses,
    ) {
        let sb = &mut self.bodies[handle.0];
        let source = &sb.clusters[cluster as usize];
        let proxy = source.proxy;
        let old_pose = bodies.get(proxy).map_or(Pose::IDENTITY, |rb| *rb.position());
        poses.record(proxy, old_pose);
        let shape_matching = source.shape_matching;
        let rest_com = rest_centroid(sb, &source.particles);
        let source_meshes: Vec<Option<SoftCollisionMesh>> = source.meshes.clone();

        // The new clusters, one per extra piece.
        let mut new_clusters: Vec<u32> = Vec::new();
        for piece in &pieces[1..] {
            let index = sb.clusters.len() as u32;
            let new_proxy =
                spawn_proxy(&sb.particle_settings, sb.user_data, handle, index, bodies);
            let mut c = SoftBodyCluster::new(piece.clone(), new_proxy, shape_matching);
            c.cell = sb.matching_cell(piece);
            sb.clusters.push(c);
            new_clusters.push(index);
            poses.record(new_proxy, old_pose);
        }
        // The retained piece: the warm shape impulses follow their particles.
        let cell = sb.matching_cell(&pieces[0]);
        {
            let c = &mut sb.clusters[cluster as usize];
            if c.shape_impulses.len() == c.particles.len() {
                let mut keep = c.particles.iter().map(|v| pieces[0].binary_search(v).is_ok());
                c.shape_impulses.retain(|_| keep.next().unwrap());
            } else {
                c.shape_impulses.clear();
            }
            c.particles = pieces[0].clone();
            c.cell = cell;
            c.meshes.clear();
        }

        // The meshes, restricted to every piece holding some of their elements.
        let mut piece_of_particle = vec![u32::MAX; sb.particles.len()];
        for (k, piece) in pieces.iter().enumerate() {
            for &v in piece {
                piece_of_particle[v as usize] = k as u32;
            }
        }
        for mesh in &source_meshes {
            let Some(mesh) = mesh else {
                sb.clusters[cluster as usize].meshes.push(None);
                continue;
            };
            let vertex_piece = |v: u32| match mesh.binding() {
                SoftMeshMapping::Direct { particles } => {
                    piece_of_particle[particles[v as usize] as usize]
                }
                SoftMeshMapping::Skinned { bindings } => sb
                    .cells
                    .get(bindings[v as usize].cell as usize)
                    .and_then(|c| {
                        c.vertices
                            .iter()
                            .map(|&w| piece_of_particle[w as usize])
                            .find(|p| *p != u32::MAX)
                    })
                    .unwrap_or(0),
            };
            for (k, _) in pieces.iter().enumerate() {
                let restricted = mesh.restricted_to(sb, |v| vertex_piece(v) == k as u32);
                if k == 0 {
                    // The retained slot keeps its id and its collider (rebuilt by the caller).
                    let slot = &mut sb.clusters[cluster as usize].meshes;
                    match restricted {
                        Some(mut m) => {
                            m.set_id(mesh.id());
                            m.collider = mesh.collider;
                            m.collision_enabled = mesh.collision_enabled;
                            slot.push(Some(m));
                        }
                        None => {
                            // No element left on this piece: the mesh dies with its collider.
                            slot.push(None);
                            if mesh.collider != ColliderHandle::invalid() {
                                colliders.remove_internal(mesh.collider, islands, bodies, false);
                            }
                        }
                    }
                    continue;
                }
                let Some(mut m) = restricted else {
                    continue;
                };
                let ci = new_clusters[k - 1];
                let new_proxy = sb.clusters[ci as usize].proxy;
                let id = SoftMeshId {
                    cluster: ci,
                    mesh: sb.clusters[ci as usize].meshes.len() as u32,
                };
                m.set_id(id);
                m.collision_enabled =
                    mesh.collision_enabled && mesh.collider != ColliderHandle::invalid();
                if m.collision_enabled {
                    match clone_mesh_collider(
                        mesh.collider,
                        handle,
                        &m,
                        sb,
                        new_proxy,
                        &old_pose,
                        bodies,
                        colliders,
                    ) {
                        Some(co) => m.collider = co,
                        None => m.collision_enabled = false,
                    }
                }
                sb.clusters[ci as usize].meshes.push(Some(m));
            }
        }

        // The joints: each follows the piece closest to its anchor in the rest shape.
        let attached: Vec<(RigidBodyHandle, RigidBodyHandle, ImpulseJointHandle, Vector)> =
            impulse_joints
                .attached_joints(proxy)
                .filter(|(_, _, _, j)| (j.body1 == proxy) != (j.body2 == proxy))
                .map(|(_, _, h, j)| {
                    let local = if j.body1 == proxy {
                        j.data.local_frame1.translation
                    } else {
                        j.data.local_frame2.translation
                    };
                    (j.body1, j.body2, h, local)
                })
                .collect();
        for (b1, b2, joint, local) in attached {
            let anchor = rest_com + local;
            let closest = pieces
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    sb.rest_distance_to(a, anchor)
                        .total_cmp(&sb.rest_distance_to(b, anchor))
                        .then(a[0].cmp(&b[0]))
                })
                .map_or(0, |(k, _)| k);
            if closest == 0 {
                continue;
            }
            let to = sb.clusters[new_clusters[closest - 1] as usize].proxy;
            let n1 = if b1 == proxy { to } else { b1 };
            let n2 = if b2 == proxy { to } else { b2 };
            impulse_joints.set_bodies(joint, n1, n2, true);
            event.moved_joints.push(SoftJointMove {
                joint,
                from: proxy,
                to,
            });
        }

        // A cluster split by the crack and again by the body split keeps one retained entry.
        let retained = SoftClusterSplit {
            source_cluster: cluster,
            soft_body: handle,
            cluster,
            proxy,
            keeps_proxy: true,
        };
        if !event.clusters.contains(&retained) {
            event.clusters.push(retained);
        }
        for &ci in &new_clusters {
            event.clusters.push(SoftClusterSplit {
                source_cluster: cluster,
                soft_body: handle,
                cluster: ci,
                proxy: sb.clusters[ci as usize].proxy,
                keeps_proxy: false,
            });
        }
        sb.modified = true;
        self.island_events.push(SoftBodyIslandEvent { handle });
    }
}

/// Retained particles after `remap` (old to new, `u32::MAX` if gone), as `particles[new] = old`.
fn kept_particles(remap: &[u32], num_particles: usize) -> Vec<u32> {
    if remap.is_empty() {
        return (0..num_particles as u32).collect();
    }
    (0..remap.len() as u32)
        .filter(|&old| remap[old as usize] != u32::MAX)
        .collect()
}

impl SoftBodySet {
    /// Splits the soft body `handle` along the pieces the tear disconnected (see
    /// [`SoftBody::seeded_components`]): the largest piece (by rest measure) keeps the body, every
    /// other becomes its own soft body. Returns the new handles, recording the pieces in `event`.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn split_body_along(
        &mut self,
        handle: SoftBodyHandle,
        seeds: &[[u32; 2]],
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        event: &mut SoftBodyTearEvent,
        poses: &mut ProxyPoses,
    ) -> Vec<SoftBodyHandle> {
        let pieces = {
            let sb = &self.bodies[handle.0];
            let components = sb.seeded_components(None, seeds);
            if components.len() < 2 {
                return Vec::new();
            }
            let all: Vec<u32> = (0..sb.particles.len() as u32).collect();
            pieces_keeping_the_largest(sb, &all, components, SoftBody::rest_measure_of)
        };
        let mut piece_of_particle = vec![0u32; self.bodies[handle.0].particles.len()];
        for (k, piece) in pieces.iter().enumerate() {
            for &v in piece {
                piece_of_particle[v as usize] = k as u32;
            }
        }

        // A cluster straddling several pieces (one disconnected at creation, since a cluster the
        // tear reached was split already) is partitioned by piece, the heaviest part keeping it.
        let num_clusters = self.bodies[handle.0].clusters.len();
        for ci in 0..num_clusters as u32 {
            let groups = {
                let sb = &self.bodies[handle.0];
                let Some(cluster) = sb.cluster(ci) else {
                    continue;
                };
                let mut groups: Vec<Vec<u32>> = vec![Vec::new(); pieces.len()];
                for &v in cluster.particles() {
                    groups[piece_of_particle[v as usize] as usize].push(v);
                }
                groups.retain(|group| !group.is_empty());
                if groups.len() < 2 {
                    continue;
                }
                let heaviest = (0..groups.len())
                    .max_by(|&i, &j| {
                        sb.mass_of(&groups[i])
                            .total_cmp(&sb.mass_of(&groups[j]))
                            .then(groups[j][0].cmp(&groups[i][0]))
                    })
                    .unwrap_or(0);
                groups.swap(0, heaviest);
                groups
            };
            self.split_cluster(
                handle,
                ci,
                &groups,
                islands,
                bodies,
                colliders,
                impulse_joints,
                event,
                poses,
            );
        }

        // Every other piece becomes a soft body: a copy of the torn body reduced to the piece's
        // clusters and particles, its proxies and colliders re-tagged.
        let mut new_handles = Vec::new();
        let mut moved: Vec<(u32, SoftBodyHandle, u32)> = Vec::new();
        for k in 1..pieces.len() as u32 {
            let (piece_body, moves, remap) = {
                let sb = &self.bodies[handle.0];
                extract_piece(sb, handle, &piece_of_particle, k)
            };
            let num_particles = remap.len();
            let new_handle = SoftBodyHandle(self.bodies.insert(piece_body));
            for &(old_ci, new_ci) in &moves {
                let cluster = &self.bodies[new_handle.0].clusters[new_ci as usize];
                if let Some(rb) = bodies.get_mut_internal(cluster.proxy) {
                    rb.soft_body = new_handle;
                    rb.soft_cluster = new_ci;
                }
                for mesh in cluster.meshes.iter().flatten() {
                    if let Some(co) = colliders.get_mut_internal(mesh.collider) {
                        co.deformable_mesh_ref = Some(SoftMeshRef {
                            body: new_handle,
                            id: mesh.id(),
                        });
                    }
                }
                for split in event.clusters.iter_mut() {
                    if split.soft_body == handle && split.cluster == old_ci {
                        split.soft_body = new_handle;
                        split.cluster = new_ci;
                    }
                }
                moved.push((old_ci, new_handle, new_ci));
            }
            event.pieces.push(SoftBodyPiece {
                soft_body: new_handle,
                particles: kept_particles(&remap, num_particles),
                clusters: moves.iter().map(|&(old, new)| [old, new]).collect(),
            });
            self.island_events.push(SoftBodyIslandEvent { handle: new_handle });
            new_handles.push(new_handle);
        }

        // The retained body: the moved clusters become dead slots, their particles leave.
        let sb = &mut self.bodies[handle.0];
        for &(old_ci, _, _) in &moved {
            let _ = sb.clusters[old_ci as usize].tombstone();
        }
        let dead: Vec<bool> = piece_of_particle.iter().map(|&p| p != 0).collect();
        let num_particles = sb.particles.len();
        let (_, remap) = sb.remove_dead_particles(&dead);
        if !sb.live_clusters().any(|(_, c)| c.proxy == sb.root_body) {
            let root = sb
                .live_clusters()
                .next()
                .map_or(RigidBodyHandle::invalid(), |(_, c)| c.proxy);
            sb.root_body = root;
        }
        sb.pieces.extend(new_handles.iter().copied());
        sb.attachments_modified = true;
        event.pieces.insert(
            0,
            SoftBodyPiece {
                soft_body: handle,
                particles: kept_particles(&remap, num_particles),
                clusters: sb.live_clusters().map(|(ci, _)| [ci, ci]).collect(),
            },
        );
        self.attached_to_stale = true;
        if !self.island_events.iter().any(|e| e.handle == handle) {
            self.island_events.push(SoftBodyIslandEvent { handle });
        }
        new_handles
    }
}

/// A copy of `sb` reduced to piece `k` of `piece_of_particle`: clusters renumbered (the caller
/// re-tags their proxies and colliders), particles compacted, per-body state reset with `origin`.
/// Returns the body, its clusters `(in sb, in copy)` and the particle remap (`u32::MAX`: dropped).
fn extract_piece(
    sb: &SoftBody,
    origin: SoftBodyHandle,
    piece_of_particle: &[u32],
    k: u32,
) -> (SoftBody, Vec<(u32, u32)>, Vec<u32>) {
    let mut body = sb.clone();
    body.clusters = Vec::new();
    let mut moves = Vec::new();
    for (ci, cluster) in sb.clusters.iter().enumerate() {
        let inside = cluster
            .particles
            .first()
            .is_some_and(|&v| piece_of_particle[v as usize] == k);
        if !cluster.is_live() || !inside {
            continue;
        }
        let new_ci = body.clusters.len() as u32;
        let mut cluster = cluster.clone();
        for (mi, mesh) in cluster.meshes.iter_mut().enumerate() {
            if let Some(mesh) = mesh {
                mesh.set_id(SoftMeshId {
                    cluster: new_ci,
                    mesh: mi as u32,
                });
            }
        }
        body.clusters.push(cluster);
        moves.push((ci as u32, new_ci));
    }
    let dead: Vec<bool> = piece_of_particle.iter().map(|&p| p != k).collect();
    let (_, remap) = body.remove_dead_particles(&dead);
    let root = body
        .live_clusters()
        .next()
        .map_or(RigidBodyHandle::invalid(), |(_, c)| c.proxy);
    body.root_body = root;
    body.origin = Some(origin);
    body.pieces = Vec::new();
    body.sleeping = false;
    body.positions_modified = false;
    body.attachments_modified = true;
    body.modified = true;
    body.tearing_pending = false;
    body.contact_approach_speeds = [None; 2];
    body.contact_load = 0.0;
    body.load_extra_substeps = 0;
    (body, moves, remap)
}

/// Re-bases what hangs on the proxies of `poses` onto their fresh frames: the local frames of
/// their impulse joints and of the rigid (non-deformable) colliders a user hung on them, so each
/// keeps its world pose. A fresh proxy was recorded with the pose of the proxy it was split from.
pub(super) fn rebase_proxy_attachments(
    poses: &ProxyPoses,
    bodies: &RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
) {
    for &(proxy, old_pose) in &poses.0 {
        let Some(rb) = bodies.get(proxy) else {
            continue;
        };
        let new_pose = *rb.position();
        if new_pose.translation == old_pose.translation && new_pose.rotation == old_pose.rotation
        {
            continue;
        }
        let delta = new_pose.inverse() * old_pose;
        impulse_joints.map_attached_joints_mut(proxy, |_, _, _, joint| {
            if joint.body1 == proxy {
                joint.data.local_frame1 = delta * joint.data.local_frame1;
            }
            if joint.body2 == proxy {
                joint.data.local_frame2 = delta * joint.data.local_frame2;
            }
        });
        for co_handle in rb.colliders().to_vec() {
            let Some(co) = colliders.get_mut(co_handle) else {
                continue;
            };
            if co.is_deformable_collider() {
                continue;
            }
            if let Some(pos) = co.position_wrt_parent().copied() {
                co.set_position_wrt_parent(delta * pos);
            }
        }
    }
}
