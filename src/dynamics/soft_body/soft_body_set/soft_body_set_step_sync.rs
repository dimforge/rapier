//! Per-step maintenance of the set: applying user changes, updating proxies and colliders, syncing particle positions, tearing and cutting.
use super::soft_body_set_proxies::{rebuilt_surface_shape, sync_soft_body};
use super::soft_body_set_split::{ProxyPoses, rebase_proxy_attachments};
use super::{SoftBodyIslandEvent, SoftBodySet};
use crate::alloc_prelude::*;
use crate::dynamics::soft_body::{SoftBody, SoftBodyHandle, SoftBodyTearEvent};
use crate::dynamics::{
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet, RigidBodySet,
};
use crate::geometry::{ColliderHandle, ColliderPosition, ColliderSet};
use crate::math::{DIM, Pose, Real, Vector};
use crate::pipeline::EventHandler;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

impl SoftBodySet {
    /// Applies the user's changes since the last step: wakes the soft bodies whose settings
    /// changed, enables or disables their root body, moves the colliders of moved particles, and
    /// updates attachment index and island links. A non-finite particle quarantines its body.
    pub(crate) fn apply_user_changes(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        params: &IntegrationParameters,
        quarantined: &mut Vec<SoftBodyHandle>,
    ) {
        // The colliders' shapes say which surfaces are oriented (the user may have changed them).
        for (_, sb) in self.bodies.iter_mut() {
            for mesh in sb.meshes_mut() {
                mesh.read_orientation(colliders);
            }
        }
        let mut relink = false;
        let handles: Vec<SoftBodyHandle> = self
            .bodies
            .iter()
            .filter(|(_, sb)| sb.modified || sb.positions_modified || sb.attachments_modified)
            .map(|(h, _)| SoftBodyHandle(h))
            .collect();
        for handle in handles {
            let sb = &mut self.bodies[handle.0];
            if core::mem::take(&mut sb.modified) {
                if sb.enabled && !sb.is_finite() {
                    sb.sanitize_dynamics();
                    sb.enabled = false;
                    quarantined.push(handle);
                }
                if let Some(rb) = bodies.get_mut(sb.root_body) {
                    if rb.is_enabled() != sb.enabled {
                        rb.set_enabled(sb.enabled);
                    }
                    if sb.enabled {
                        rb.wake_up(true);
                    }
                }
            }
            if core::mem::take(&mut sb.positions_modified) && sb.enabled {
                Self::update_colliders(sb, bodies, colliders, false);
                for mesh in sb.meshes_mut() {
                    // A teleport can author a tangle with no accumulated travel: the next
                    // self-crossing sweep must run.
                    mesh.crossing_sweep_travel = Real::MAX;
                }
            }
            // Velocities authored between steps outrun the speculative margin computed at the
            // end of the last step, so the margin is raised here to cover the coming step's reach.
            if sb.enabled && params.soft_bodies.recovery.authored_velocity_margin {
                let mut max_speed: Real = 0.0;
                for p in &sb.particles {
                    max_speed = max_speed.max(p.velocity.length());
                }
                let margin = max_speed * params.dt;
                if margin > 0.0 {
                    for cluster in sb.clusters.iter().filter(|cluster| cluster.is_live()) {
                        if let Some(rb) = bodies.get_mut_internal(cluster.proxy()) {
                            rb.soft_motion_margin = rb.soft_motion_margin.max(margin);
                        }
                        // The margin only pads the deformable colliders: flagged as modified so
                        // the broad phase updates their AABBs with it.
                        for mesh in cluster.meshes() {
                            let _ = colliders.get_mut(mesh.collider());
                        }
                    }
                }
            }
            if core::mem::take(&mut sb.attachments_modified) {
                relink = true;
                self.island_events.push(SoftBodyIslandEvent { handle });
            }
        }
        if relink || self.attached_to_stale {
            self.attached_to_stale = false;
            self.attached_to.clear();
            for (h, sb) in self.bodies.iter() {
                for (i, a) in sb.attachments.iter().enumerate() {
                    self.attached_to
                        .entry(a.body)
                        .or_default()
                        .push((SoftBodyHandle(h), i as u32));
                }
            }
        }
    }

    /// Updates every live cluster's proxy rigid body from the particles: pose from the cluster's
    /// frame, velocity from its rigid fit, mass properties from its reduced mass matrix. The
    /// proxies' colliders keep their world pose (local pose re-based) until the frame-local phase.
    pub(crate) fn update_cluster_proxies(
        sb: &mut SoftBody,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) {
        let _ = colliders;
        for ci in 0..sb.clusters.len() {
            let Some(mut frame) = sb.cluster_frame(ci) else {
                continue;
            };
            if !frame.pose.is_finite() || !frame.linvel.is_finite() || !frame.angvel.is_finite() {
                // A non-finite fit means non-finite particles: the quarantine handles the body;
                // the proxy keeps its previous (finite) state meanwhile.
                continue;
            }
            let proxy = sb.clusters[ci].proxy;
            let Some(rb) = bodies.get_mut_internal(proxy) else {
                continue;
            };
            // Dead zone: a pose change below the frame fit's rest noise keeps the previous pose
            // bit-for-bit, so the frozen contact anchors are not re-linearized by fit noise.
            let prev = rb.pos.position;
            let dt = (frame.pose.translation - prev.translation).length();
            // The 2D angle is signed: take its magnitude so either direction leaves the dead zone.
            #[cfg(feature = "dim2")]
            let dr = frame.pose.rotation.angle_between(&prev.rotation).abs();
            #[cfg(feature = "dim3")]
            let dr = frame.pose.rotation.angle_between(prev.rotation);
            if dt < 1.0e-6 && dr < 1.0e-6 {
                frame.pose = prev;
            }
            sb.clusters[ci].rotation = frame.pose.rotation;
            sb.clusters[ci].last_gather = (frame.linvel, frame.angvel);
            rb.pos.position = frame.pose;
            rb.pos.next_position = frame.pose;
            rb.vels.linvel = frame.linvel;
            rb.vels.angvel = frame.angvel;
            rb.mprops.local_mprops.inv_mass = frame.inv_mass;
            rb.mprops.local_mprops.local_com = Vector::ZERO;
            rb.mprops.world_com = frame.pose.translation;
            rb.mprops.effective_inv_mass = Vector::splat(frame.inv_mass);
            rb.mprops.effective_world_inv_inertia = frame.inv_inertia;
        }
    }

    /// Moves the soft body's colliders to its current particle positions (the surface shape, or
    /// the particle balls). `rebuild`: the surface topology changed (a tear), the surface shape is
    /// rebuilt instead of deformed in place.
    pub(super) fn update_colliders(
        sb: &SoftBody,
        bodies: &RigidBodySet,
        colliders: &mut ColliderSet,
        rebuild: bool,
    ) {
        // The mesh geometry is expressed in its collider's frame (proxy pose composed with the
        // collider's local pose); any shared frame works, so a proxy pose one update stale is fine.
        for cluster in sb.clusters.iter().filter(|cluster| cluster.is_live()) {
            let frame = bodies
                .get(cluster.proxy())
                .map(|rb| rb.pos.position)
                .unwrap_or(Pose::IDENTITY);
            for mesh in cluster.meshes() {
                let Some(co) = colliders.get_mut(mesh.collider()) else {
                    continue;
                };
                co.deform_pose(frame);
                let pose = *co.position();
                if rebuild {
                    match rebuilt_surface_shape(co.shape(), mesh.local_vertices(sb, &pose), mesh) {
                        Some(shape) => co.replace_deformed_shape(shape),
                        None => co.set_enabled(false),
                    }
                } else {
                    co.deform_shape(|shape| mesh.deform_shape(sb, &pose, shape));
                }
            }
        }
    }

    /// Updates each soft body's derived state at the end of a step: surface orientation, sleep
    /// state, colliders and particle balls of the awake ones, the coming step's speculative margin
    /// and the impact-adaptive substep request. A body with a non-finite particle is quarantined.
    pub fn sync_particle_positions(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        params: &IntegrationParameters,
        quarantined: &mut Vec<SoftBodyHandle>,
    ) {
        self.sync_particle_positions_and_collect(
            bodies,
            colliders,
            params,
            params.dt,
            quarantined,
            &mut Vec::new(),
        );
    }

    /// [`Self::sync_particle_positions`], also collecting into `synced_colliders` the deformed
    /// surface colliders and the rigid colliders moved to their proxy's fresh pose, whose
    /// broad-phase AABBs must follow. `params` are the full step's (sizing the coming step's
    /// margin and substeps); `last_pass_dt` is the length of the step's last CCD pass.
    pub(crate) fn sync_particle_positions_and_collect(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        params: &IntegrationParameters,
        last_pass_dt: Real,
        quarantined: &mut Vec<SoftBodyHandle>,
        synced_colliders: &mut Vec<ColliderHandle>,
    ) {
        // Every body reads its own particles and writes only itself: updated in parallel, then
        // the writes to the shared sets are applied in body order. Disabled bodies are left
        // alone, like sleeping ones.
        let mut handles = Vec::with_capacity(self.bodies.len());
        let mut soft_bodies: Vec<(&mut SoftBody, bool)> = self
            .bodies
            .iter_mut()
            .map(|(h, sb)| {
                handles.push(SoftBodyHandle(h));
                let inactive = bodies
                    .get(sb.root_body)
                    .is_none_or(|rb| rb.is_sleeping() || !rb.is_enabled());
                (sb, inactive)
            })
            .collect();
        let mut outcomes = Vec::with_capacity(soft_bodies.len());
        let sync = |(sb, inactive): &mut (&mut SoftBody, bool)| {
            sync_soft_body(sb, *inactive, params, last_pass_dt)
        };
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            soft_bodies
                .par_iter_mut()
                .map(sync)
                .collect_into_vec(&mut outcomes);
        }
        #[cfg(not(feature = "parallel"))]
        outcomes.extend(soft_bodies.iter_mut().map(sync));

        for (((sb, _), outcome), handle) in soft_bodies.iter_mut().zip(outcomes).zip(handles) {
            let Some(margin) = outcome.margin else {
                continue;
            };
            if !outcome.finite {
                // Non-finite state: contained here, before it reaches the colliders.
                sb.sanitize_dynamics();
                sb.enabled = false;
                if let Some(rb) = bodies.get_mut(sb.root_body) {
                    rb.set_enabled(false);
                }
                quarantined.push(handle);
                continue;
            }
            let Some(rb) = bodies.get_mut_internal(sb.root_body) else {
                continue;
            };
            rb.additional_solver_iterations = outcome.additional_solver_iterations;
            rb.additional_pgs_iterations = sb.particle_settings.additional_pgs_iterations;
            // The cluster proxies first (pose, velocity, reduced mass): the colliders are
            // expressed in the fresh whole-body frame.
            Self::update_cluster_proxies(sb, bodies, colliders);
            for cluster in sb.clusters.iter().filter(|cluster| cluster.is_live()) {
                let frame = bodies
                    .get(cluster.proxy())
                    .map(|rb| rb.pos.position)
                    .unwrap_or(Pose::IDENTITY);
                for mesh in cluster.meshes() {
                    if let Some(co) = colliders.get_mut(mesh.collider()) {
                        // Deformed in place, in the collider's frame: parry refits the BVH.
                        co.deform_pose(frame);
                        let pose = *co.position();
                        co.deform_shape(|shape| mesh.deform_shape(sb, &pose, shape));
                        synced_colliders.push(mesh.collider());
                    }
                }
                // The speculative margin pads the proxy's deformable colliders only: the rigid
                // colliders a user hung on it get the AABB of any dynamic body's collider.
                let Some(rb) = bodies.get_mut_internal(cluster.proxy()) else {
                    continue;
                };
                rb.soft_motion_margin = margin;
                let proxy_pose = rb.pos.position;
                // Flagged as modified: the meshes were deformed and the rigid ones move below.
                for handle in rb.colliders() {
                    let Some(co) = colliders.get_mut(*handle) else {
                        continue;
                    };
                    // The rigid ones follow its fresh pose: the end-of-step advance leaves the
                    // proxies' colliders to this sync.
                    if !co.is_deformable_collider() {
                        if let Some(parent) = co.parent.as_ref() {
                            co.pos = ColliderPosition(proxy_pose * parent.pos_wrt_parent);
                            synced_colliders.push(*handle);
                        }
                    }
                }
            }
        }
    }

    /// Applies the tear marks left by the solver (elements loaded past their tear threshold) and
    /// by `SoftBody::tear_edge`/`tear_cell`, and reports each tear to `events`. Runs at the end
    /// of a step, before [`Self::sync_particle_positions`].
    pub(crate) fn apply_pending_tears(
        &mut self,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        events: &dyn EventHandler,
    ) {
        let pending: Vec<SoftBodyHandle> = self
            .bodies
            .iter()
            .filter(|(_, sb)| sb.tearing_pending)
            .map(|(h, _)| SoftBodyHandle(h))
            .collect();
        let mut torn = Vec::new();
        for handle in pending {
            let (edges, cells) = self.bodies[handle.0].take_torn();
            if let Some(event) = self.tear(
                handle,
                &edges,
                &cells,
                islands,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
            ) {
                torn.push(event);
            }
        }
        for event in &torn {
            events.handle_soft_body_tear_event(self, event);
        }
    }

    /// Tears a soft body right away and rebuilds its colliders, losing no material: a crack opens
    /// through a particle of every torn edge and cell (see [`SoftBody::tear_edge`]), disconnected
    /// pieces become soft bodies, crossed clusters split. Returns the event, `None` if unchanged.
    #[allow(clippy::too_many_arguments)]
    pub fn tear(
        &mut self,
        handle: SoftBodyHandle,
        edges: &[u32],
        cells: &[u32],
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftBodyTearEvent> {
        let _ = multibody_joints;
        self.change_topology(
            handle,
            islands,
            bodies,
            colliders,
            impulse_joints,
            |sb, event| sb.tear_topology(edges, cells, event),
        )
    }

    /// Applies a topology change to a soft body (`change` returns whether anything changed and
    /// records it in the event), then splits the clusters the change disconnected, rebuilds the
    /// colliders and wakes the body up.
    fn change_topology(
        &mut self,
        handle: SoftBodyHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        change: impl FnOnce(&mut SoftBody, &mut SoftBodyTearEvent) -> bool,
    ) -> Option<SoftBodyTearEvent> {
        let sb = self.bodies.get_mut(handle.0)?;
        let mut event = SoftBodyTearEvent {
            soft_body: handle,
            ..Default::default()
        };
        if !change(sb, &mut event) {
            return None;
        }

        // The clusters the crack ran through come apart with their material, then so does the
        // body: every piece the crack disconnected becomes a soft body of its own.
        let seeds = event.seeds();
        let mut poses = ProxyPoses::default();
        let split = self.split_clusters_along(
            handle,
            &seeds,
            islands,
            bodies,
            colliders,
            impulse_joints,
            &mut event,
            &mut poses,
        );
        let mut handles = self.split_body_along(
            handle,
            &seeds,
            islands,
            bodies,
            colliders,
            impulse_joints,
            &mut event,
            &mut poses,
        );
        let split = split || !handles.is_empty();
        handles.insert(0, handle);

        for &piece in &handles {
            let sb = &mut self.bodies[piece.0];
            if split {
                // The pieces' frames, fresh and retained: the colliders are expressed in them.
                Self::update_cluster_proxies(sb, bodies, colliders);
            }
            // The colliders follow the new surface at once: the next step's narrow phase must
            // not report elements that no longer exist.
            Self::update_colliders(sb, bodies, colliders, true);
        }
        if split {
            rebase_proxy_attachments(&poses, bodies, colliders, impulse_joints);
        }
        for &piece in &handles {
            if let Some(rb) = bodies.get_mut(self.bodies[piece.0].root_body) {
                rb.wake_up(true);
            }
        }
        Some(event)
    }

    /// Cuts a soft body right away along a blade, a world segment (2D) or triangle (3D), splitting
    /// the particles of the edges the blade meets (pinned ones never) and losing no material (see
    /// [`SoftBody::rest_measure`]). Returns the tear event, or `None` when nothing changed.
    #[allow(clippy::too_many_arguments)]
    pub fn cut(
        &mut self,
        handle: SoftBodyHandle,
        blade: &[Vector; DIM],
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftBodyTearEvent> {
        let _ = multibody_joints;
        self.change_topology(
            handle,
            islands,
            bodies,
            colliders,
            impulse_joints,
            |sb, event| sb.cut_topology(blade, event),
        )
    }

    /// Refreshes every mesh's vertex cache from the particles, before a narrow-phase update
    /// (see `SoftCollisionMesh::cached_vertex`); in parallel under the `parallel` feature.
    pub(crate) fn refresh_vertex_caches(&mut self) {
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            let mut bodies: Vec<&mut SoftBody> = self.bodies.iter_mut().map(|(_, sb)| sb).collect();
            bodies
                .par_iter_mut()
                .for_each(|sb| sb.refresh_vertex_caches());
        }
        #[cfg(not(feature = "parallel"))]
        for (_, sb) in self.bodies.iter_mut() {
            sb.refresh_vertex_caches();
        }
    }

    /// Wakes up the given soft body (effective at the start of the next step).
    pub fn wake_up(&mut self, handle: SoftBodyHandle, bodies: &mut RigidBodySet, strong: bool) {
        if let Some(sb) = self.bodies.get_mut(handle.0) {
            sb.wake_up();
            if let Some(rb) = bodies.get_mut(sb.root_body) {
                rb.wake_up(strong);
            }
        }
    }
}
