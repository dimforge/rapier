//! Spawning the cluster proxies and mesh colliders of a soft body, and the per-body end-of-step particle sync.
use crate::alloc_prelude::*;
use crate::dynamics::{IntegrationParameters, RigidBodyBuilder, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderBuilder, ColliderHandle, ColliderSet, SharedShape};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};
use crate::dynamics::soft_body::{SoftBody, SoftBodyHandle, SoftMeshRef};
use crate::math::{DIM, Pose, Real, Vector};

/// The deformable surface shape (a 2D polyline or 3D triangle mesh over `vertices`), flagged
/// deformable so the narrow phase never trusts contact points cached across a vertex update.
/// A closed 2D polyline is one-sided (pushing outward); an open one (a rope) is two-sided.
pub(crate) fn surface_shape(
    vertices: Vec<Vector>,
    surface: &[[u32; DIM]],
    closed: bool,
) -> Option<SharedShape> {
    if surface.is_empty() || vertices.is_empty() {
        return None;
    }
    #[cfg(feature = "dim2")]
    {
        use parry::shape::{Polyline, PolylineFlags};
        let mut flags = PolylineFlags::DEFORMABLE;
        flags.set(PolylineFlags::ORIENTED, closed);
        Some(SharedShape::new(Polyline::with_flags(
            vertices,
            Some(surface.to_vec()),
            flags,
        )))
    }
    #[cfg(feature = "dim3")]
    {
        use parry::shape::{Polyline, PolylineFlags, TriMeshFlags};
        let _ = closed;
        // A wire's elements are segments (`u32::MAX` fills the unused slot): it collides as a
        // polyline, which parry supports in 3D too.
        if super::soft_body_builder::element_vertices(&surface[0]).len() < DIM {
            let segments: Vec<[u32; 2]> = surface.iter().map(|e| [e[0], e[1]]).collect();
            return Some(SharedShape::new(Polyline::with_flags(
                vertices,
                Some(segments),
                PolylineFlags::DEFORMABLE,
            )));
        }
        SharedShape::trimesh_with_flags(vertices, surface.to_vec(), TriMeshFlags::DEFORMABLE).ok()
    }
}

/// Rebuilds a surface shape whose topology changed (a tear), keeping the replaced shape's flags
/// except those that would edit the vertex or index buffers; a 2D polyline's orientation
/// follows `closed` (a torn open loop is two-sided).
pub(super) fn rebuilt_surface_shape(
    prev: &dyn parry::shape::Shape,
    vertices: Vec<Vector>,
    surface: &[[u32; DIM]],
    closed: bool,
) -> Option<SharedShape> {
    let mut shape = surface_shape(vertices, surface, closed)?;
    let rebuilt = shape.make_mut();
    if let (Some(prev), Some(rebuilt)) = (prev.as_polyline(), rebuilt.as_polyline_mut()) {
        #[allow(unused_mut)]
        let mut flags = prev.flags();
        #[cfg(feature = "dim2")]
        flags.set(parry::shape::PolylineFlags::ORIENTED, closed);
        rebuilt.set_flags(flags);
    }
    #[cfg(feature = "dim3")]
    if let (Some(prev), Some(rebuilt)) = (prev.as_trimesh(), rebuilt.as_trimesh_mut()) {
        use parry::shape::TriMeshFlags;
        let flags = prev.flags()
            & !(TriMeshFlags::MERGE_DUPLICATE_VERTICES
                | TriMeshFlags::DELETE_DEGENERATE_TRIANGLES
                | TriMeshFlags::DELETE_DUPLICATE_TRIANGLES
                | TriMeshFlags::DELETE_BAD_TOPOLOGY_TRIANGLES);
        // A torn mesh may no longer satisfy a topology flag: it falls back to the bare
        // deformable shape.
        if rebuilt.set_flags(flags).is_err() {
            let _ = rebuilt.set_flags(TriMeshFlags::DEFORMABLE);
        }
    }
    Some(shape)
}

/// Creates the collider holding a soft body's collision `mesh`, parented to its cluster's
/// `proxy` and expressed in `frame` (the proxy's pose): the deformable shape follows the
/// particles every step. `None` when the mesh has no shape to build.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_mesh_collider(
    template: &ColliderBuilder,
    handle: SoftBodyHandle,
    mesh: &super::SoftCollisionMesh,
    body: &SoftBody,
    proxy: RigidBodyHandle,
    frame: &Pose,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> Option<ColliderHandle> {
    let shape = surface_shape(
        mesh.local_vertices(body, frame),
        mesh.indices(),
        mesh.is_closed(),
    )?;
    let mut collider = template
        .clone()
        .density(0.0)
        .contact_skin(body.particle_radius())
        .build();
    collider.set_shape(shape);
    collider.set_position(Pose::IDENTITY);
    let co_handle = colliders.insert_with_parent(collider, proxy, bodies);
    // `insert_with_parent` resets the internal references; the collider's world pose is the
    // frame (local pose identity).
    let co = colliders.index_mut_internal(co_handle);
    co.deformable_mesh_ref = Some(SoftMeshRef {
        body: handle,
        id: mesh.id(),
    });
    co.deform_pose(*frame);
    Some(co_handle)
}

/// Creates the collider of a mesh split off another (see `SoftCollisionMesh::restricted_to`): a
/// copy of `source` (material, groups, events, user data) holding the new mesh's shape, parented
/// to `proxy` in `frame` (the proxy's pose). `None` if the source is gone or the mesh has no shape.
#[allow(clippy::too_many_arguments)]
pub(super) fn clone_mesh_collider(
    source: ColliderHandle,
    handle: SoftBodyHandle,
    mesh: &super::SoftCollisionMesh,
    body: &SoftBody,
    proxy: RigidBodyHandle,
    frame: &Pose,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> Option<ColliderHandle> {
    let mut collider = colliders.get(source)?.clone();
    let shape = rebuilt_surface_shape(
        collider.shape(),
        mesh.local_vertices(body, frame),
        mesh.indices(),
        mesh.is_closed(),
    )?;
    collider.set_shape(shape);
    collider.set_position(Pose::IDENTITY);
    collider.set_enabled(true);
    let co_handle = colliders.insert_with_parent(collider, proxy, bodies);
    let co = colliders.index_mut_internal(co_handle);
    co.deformable_mesh_ref = Some(SoftMeshRef {
        body: handle,
        id: mesh.id(),
    });
    co.deform_pose(*frame);
    Some(co_handle)
}

/// Creates the proxy rigid body of cluster `cluster` of soft body `handle`: a
/// [`crate::dynamics::RigidBodyType::SoftFrame`] body standing for the cluster in the islands
/// and joints and holding its colliders. Cluster 0's proxy is the soft body's root body.
pub(super) fn spawn_proxy(
    settings: &super::SoftBodyParticleSettings,
    user_data: u128,
    handle: SoftBodyHandle,
    cluster: u32,
    bodies: &mut RigidBodySet,
) -> RigidBodyHandle {
    let rb = RigidBodyBuilder::dynamic()
        .gravity_scale(0.0)
        .additional_solver_iterations(settings.additional_solver_iterations)
        .additional_pgs_iterations(settings.additional_pgs_iterations)
        .can_sleep(settings.can_sleep)
        .dominance_group(settings.dominance_group)
        .user_data(user_data)
        .build();
    let rb_handle = bodies.insert(rb);
    // Stamped after insertion: `insert` resets the internal references. The body type is set
    // directly: `SoftFrame` cannot be built or converted to through the public API.
    let rb = bodies.index_mut_internal(rb_handle);
    rb.body_type = crate::dynamics::RigidBodyType::SoftFrame;
    rb.soft_body = handle;
    rb.soft_cluster = cluster;
    #[cfg(feature = "dim3")]
    {
        rb.forces.gyroscopic_forces_enabled = false;
    }
    rb_handle
}

/// What `SoftBodySet::sync_particle_positions` computed for one soft body, applied to the shared
/// sets afterwards.
pub(super) struct SyncOutcome {
    /// The speculative margin of the body's colliders for the coming step (`None`: asleep, left
    /// alone).
    pub(super) margin: Option<Real>,
    /// The substep request written on the body's root body.
    pub(super) additional_solver_iterations: usize,
    /// Whether every particle is finite (`false`: the body is quarantined).
    pub(super) finite: bool,
}

/// The per-body part of `SoftBodySet::sync_particle_positions`: updates the state derived
/// from the particle positions, then computes the colliders' speculative margin and the substep
/// request of an active body (`inactive`: asleep or disabled).
pub(super) fn sync_soft_body(
    sb: &mut SoftBody,
    inactive: bool,
    params: &IntegrationParameters,
) -> SyncOutcome {
    let dt = params.dt;
    // Inactive: asleep, or disabled (then not asleep, just out of the simulation).
    let sleeping = inactive && sb.enabled;
    if sb.is_finite() {
        sb.update_orientation();
    }
    if sleeping && !sb.sleeping {
        // Like a rigid body falling asleep: at rest means at rest.
        for p in &mut sb.particles {
            p.velocity = Vector::ZERO;
        }
    }
    sb.sleeping = sleeping;
    // Creep keeps the body awake: the sleep rule sees an infinite speed until the flow stops.
    let plastic_flowing = core::mem::take(&mut sb.plastic_flowing);
    if plastic_flowing {
        sb.sleep_speed = Real::MAX;
        sb.rest_fit_pending = true;
    }
    // The rest positions follow the flow, over the steps it takes them to settle.
    if sb.rest_fit_pending && !sleeping && sb.is_finite() && sb.fit_rest_positions() {
        sb.rest_fit_pending = false;
    }
    let mut outcome = SyncOutcome {
        margin: None,
        additional_solver_iterations: 0,
        finite: true,
    };

    for cluster in &mut sb.clusters {
        cluster.prev_shape_matching_target = cluster.shape_matching_target;
    }

    if inactive {
        return outcome;
    }
    if !sb.is_finite() {
        outcome.finite = false;
        outcome.margin = Some(0.0);
        return outcome;
    }
    // The skin only reads this body's particles, so it rides along with the per-body pass.
    sb.update_meshes();
    let mut max_speed: Real = 0.0;
    for p in &sb.particles {
        max_speed = max_speed.max(p.velocity.length());
    }
    // The soft body's own sleep rule runs on its proxies at the start of the next step
    // (`RigidBodyActivation::update_energy`), on the fastest particle.
    if !plastic_flowing {
        sb.sleep_speed = max_speed;
    }
    outcome.margin = Some(max_speed * dt);

    // Impact-adaptive substeps: while something moves faster than one particle radius per
    // substep at this body's contacts, its island component runs enough substeps to bound that
    // travel. Only bodies with contact constraints this step or the last count.
    let mut speed: Real = 0.0;
    if sb.contact_approach_speeds.iter().any(|s| s.is_some()) {
        speed = max_speed;
        for s in sb.contact_approach_speeds.iter().flatten() {
            speed = speed.max(*s);
        }
    }
    let radius = sb.particle_radius.max(1.0e-6);
    let needed = (speed * dt / radius).ceil().max(0.0) as usize;
    let impact_extra = needed.saturating_sub(params.num_solver_iterations);
    // Load-adaptive substeps: +1 past 20 g of contact load per unit mass, +2 past 60 g (g taken
    // as 10 length units/s^2), released with hysteresis at half those loads so the island's
    // partition does not flicker.
    let mass: Real = sb.particles.iter().map(|p| p.mass).sum();
    let load = sb.contact_load / (dt * mass.max(1.0e-9) * 10.0 * params.length_unit);
    let load_extra = match sb.load_extra_substeps {
        0 if load > 20.0 => 1,
        1 if load > 60.0 => 2,
        1 if load < 10.0 => 0,
        2 if load < 30.0 => 1,
        current => current,
    };
    sb.load_extra_substeps = load_extra;
    let extra = impact_extra
        .max(load_extra as usize)
        .min(params.soft_bodies.max_extra_substeps);
    outcome.additional_solver_iterations =
        sb.particle_settings.additional_solver_iterations.max(extra);
    outcome
}
