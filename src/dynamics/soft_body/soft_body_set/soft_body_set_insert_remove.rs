//! Insertion and removal of the soft bodies of a `SoftBodySet`.
use crate::alloc_prelude::*;
use crate::dynamics::{ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodyHandle, RigidBodySet, SoftBodyBuilder};
use crate::geometry::ColliderSet;
use crate::dynamics::soft_body::{SoftBody, SoftBodyCluster, SoftBodyHandle, SoftMeshId};
use crate::math::{Pose, Rotation};
use super::soft_body_set_proxies::{spawn_mesh_collider, spawn_proxy};
use super::{SoftBodyIslandEvent, SoftBodySet};

impl SoftBodySet {
    /// Inserts a soft body, creating its root rigid body and its colliders from the collider
    /// template (the deformable surface collider, or one ball per surface particle for a body
    /// colliding through its particles); its element topology must index valid particles.
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
            volume_factor: 1.0,
            rest_com: soft_body.rest_com,
            particle_radius: soft_body.particle_radius,
            particle_settings: soft_body.particle_settings,
            num_colors: 0,
            has_overflow_color: false,
            modified: false,
            plastic_flowing: false,
            sleep_speed: 0.0,
            tearing_pending: false,
            topology_version: 0,
            contact_approach_speeds: [None; 2],
            contact_load: 0.0,
            load_extra_substeps: 0,
            user_data: 0,
        }));

        // The deformable meshes' colliders attach to the root body (so they link islands and wake
        // the soft body), with vertices in the whole-body cluster's frame (the proxy's pose holds
        // the rigid motion). They collide with everything, fixed and kinematic colliders included.
        if let Some(template) = soft_body_builder.collider_template.as_ref() {
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
        islands.persistent.unlink_soft_body_attachments(handle);
        self.attached_to_stale |= !soft_body.attachments.is_empty();
    }
}
