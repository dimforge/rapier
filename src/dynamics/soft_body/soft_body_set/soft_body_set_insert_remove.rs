//! Insertion and removal of the soft bodies of a `SoftBodySet`.
use super::soft_body_set_proxies::{spawn_mesh_collider, spawn_proxy};
use super::{SoftBodyIslandEvent, SoftBodySet};
use crate::alloc_prelude::*;
use crate::dynamics::soft_body::{SoftBody, SoftBodyCluster, SoftBodyHandle, SoftMeshId};
use crate::dynamics::{
    ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
    SoftBodyBuilder,
};
use crate::geometry::ColliderSet;
use crate::math::{Pose, Rotation};

impl SoftBodySet {
    /// Inserts a soft body, creating its root rigid body and the deformable colliders of its
    /// collision meshes from the collider template; its element topology must index valid
    /// particles.
    pub fn insert(
        &mut self,
        soft_body_builder: SoftBodyBuilder,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> SoftBodyHandle {
        let mut soft_body = soft_body_builder.build();
        soft_body.sleeping = false;
        soft_body.enabled = true;
        soft_body.positions_modified = false;
        // Attachments made before the insertion get their island links with the insertion event.
        soft_body.attachments_modified = !soft_body.attachments.is_empty();
        // Reserve the handle first: the root body stores it.
        let handle = SoftBodyHandle(self.bodies.insert_with(|_| SoftBody {
            particles: Vec::new(),
            clusters: Vec::new(),
            cluster_refs: Vec::new(),
            root_body: RigidBodyHandle::invalid(),
            attachments: Vec::new(),
            sleeping: false,
            enabled: true,
            positions_modified: false,
            attachments_modified: false,
            edges: Vec::new(),
            #[cfg(feature = "dim3")]
            dihedrals: Vec::new(),
            cells: Vec::new(),
            boundary: Vec::new(),
            boundary_closed: false,
            boundary_element_cells: Vec::new(),
            material: soft_body.material,
            cell_model: soft_body.cell_model,
            #[cfg(feature = "fem")]
            solver: soft_body.solver,
            volume_preservation: false,
            volume_pieces: Vec::new(),
            volume_factor: 1.0,
            rest_com: soft_body.rest_com,
            particle_radius: soft_body.particle_radius,
            particle_settings: soft_body.particle_settings,
            num_colors: 0,
            has_overflow_color: false,
            modified: false,
            plastic_flowing: false,
            rest_fit_pending: false,
            sleep_speed: 0.0,
            tearing_pending: false,
            topology_version: 0,
            contact_approach_speeds: [None; 2],
            contact_approach_step_open: false,
            contact_load: 0.0,
            load_extra_substeps: 0,
            origin: None,
            pieces: Vec::new(),
            user_data: 0,
        }));

        soft_body.root_body = spawn_proxy(
            &soft_body.particle_settings,
            soft_body.user_data,
            handle,
            0,
            bodies,
        );
        let meshes = soft_body_builder.build_meshes(&soft_body);
        soft_body.clusters = alloc::vec![SoftBodyCluster {
            particles: (0..soft_body.particles.len() as u32).collect(),
            proxy: soft_body.root_body,
            rotation: Rotation::IDENTITY,
            cell: u32::MAX,
            shape_matching: soft_body_builder.shape_matching,
            shape_matching_target: None,
            prev_shape_matching_target: None,
            last_gather: Default::default(),
            shape_impulses: Vec::new(),
            meshes: meshes.into_iter().map(Some).collect(),
        }];
        soft_body.cluster_refs = alloc::vec![1; soft_body.particles.len()];
        // The whole-body frame first: the colliders are expressed in it.
        Self::update_cluster_proxies(&mut soft_body, bodies, colliders);
        let frame = bodies
            .get(soft_body.root_body)
            .map(|rb| rb.pos.position)
            .unwrap_or(Pose::IDENTITY);

        // The deformable meshes' colliders attach to the root body (so they link islands and wake
        // the soft body), with vertices in the whole-body cluster's frame (the proxy's pose holds
        // the rigid motion). They collide with everything, fixed and kinematic colliders included.
        if let Some(template) = soft_body_builder.collider_template.as_ref() {
            let mut clusters = core::mem::take(&mut soft_body.clusters);
            for (ci, cluster) in clusters.iter_mut().enumerate() {
                for (mi, mesh) in cluster.meshes.iter_mut().enumerate() {
                    let Some(mesh) = mesh else { continue };
                    mesh.set_id(SoftMeshId {
                        cluster: ci as u32,
                        mesh: mi as u32,
                    });
                    if !mesh.collision_enabled() {
                        continue;
                    }
                    let Some(co_handle) = spawn_mesh_collider(
                        template,
                        handle,
                        mesh,
                        &soft_body,
                        cluster.proxy,
                        &frame,
                        bodies,
                        colliders,
                    ) else {
                        // No shape to build (no element): the mesh collides through nothing,
                        // and says so rather than holding an invalid collider.
                        mesh.collision_enabled = false;
                        continue;
                    };
                    mesh.collider = co_handle;
                }
            }
            soft_body.clusters = clusters;
        }

        *self.bodies.get_mut(handle.0).unwrap() = soft_body;
        self.island_events.push(SoftBodyIslandEvent { handle });
        handle
    }

    /// Removes a soft body, its cluster proxies (root rigid body included) and its colliders.
    pub fn remove(
        &mut self,
        handle: SoftBodyHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftBody> {
        let soft_body = self.bodies.remove(handle.0)?;
        // Unlink the attachment and proxy-chain links while the proxies still exist.
        islands.persistent.unlink_soft_body_attachments(handle);
        islands.persistent.unlink_soft_body_proxy_chain(handle);
        self.attached_to_stale |= !soft_body.attachments.is_empty();
        for cluster in &soft_body.clusters {
            if cluster.is_live() {
                // The soft body is out of the arena already, so the proxy-removal hook no-ops.
                bodies.remove(
                    cluster.proxy,
                    islands,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    self,
                    true,
                );
            }
        }
        Some(soft_body)
    }
}
