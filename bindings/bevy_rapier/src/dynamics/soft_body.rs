//! Components describing soft bodies: deformable bodies made of particles linked by elastic
//! constraints, simulated together with the rigid-bodies.
//!
//! # Entities and conventions
//!
//! A soft body is created by inserting a [`SoftBody`] component on an entity. The positions of
//! the soft body's particles given by its [`SoftBodyBuilder`] are expressed in the local frame of
//! that entity: they are transformed by the entity's [`GlobalTransform`] when the soft body is
//! created. After its creation, the particles live in world space and the plugin drives the
//! entity's [`Transform`]: it follows the pose of the soft body's whole-body proxy (the frame of
//! its whole-body cluster: its origin is the centroid of the free particles, and its rotation the
//! best fit of the particles onto their rest shape, identity at creation), its scale is kept. The
//! colliders of children entities are attached to that proxy, so their entities stay where
//! their colliders are. A mesh synchronized with [`SoftBodyMeshSync`] is expressed in that frame.
//! The mass-weighted center of the particles is written to [`SoftBodyState::center_of_mass`].
//!
//! The soft body entity also stands for the Rapier objects created with the soft body:
//! - its surface colliders (their `user_data` holds the entity bits), so collision events,
//!   contact pairs and scene queries report the soft body entity as the collider entity;
//! - its hidden proxy rigid-bodies, so contact pairs report it as the rigid-body entity;
//! - its whole-body proxy rigid-body in particular (the proxy of its whole-body cluster, see
//!   [`RapierRigidBodySet::soft_body_whole_proxy`]): the entity is mapped to it, so impulse joints
//!   can attach the whole soft body like a rigid-body, with the soft body entity as their
//!   `parent` or by adding them to the soft body entity.
//!
//! [`SoftBodyCluster`] entities stand for their own cluster proxy rigid-body, which impulse joints
//! and colliders can be attached to like to any rigid-body. A cluster (or the whole-body cluster
//! of a soft body entity) can be driven with the [`SoftBodyClusterPinned`],
//! [`SoftBodyClusterKinematicTarget`], [`SoftBodyClusterShapeMatching`] and
//! [`SoftBodyClusterMaterial`] components.
//!
//! When a soft body tears, the particle indices of its [`SoftBodyPinnedParticles`],
//! [`SoftBodyKinematicTargets`], [`SoftBodyAttachments`], [`SoftBodyExternalForce`] and
//! [`SoftBodyExternalImpulse`] components (and the pinned particles of its [`SoftBody`] builder,
//! restored when [`SoftBodyPinnedParticles`] is removed) are remapped to the renumbered
//! particles, the entries of the particles moved to a new piece being moved to the entity of that
//! piece. When a tear splits a cluster, the entity spawned for the new cluster inherits the
//! [`SoftBodyClusterPinned`], [`SoftBodyClusterKinematicTarget`] (shifted so that the particles
//! keep their targets), [`SoftBodyClusterShapeMatching`] and [`SoftBodyClusterMaterial`]
//! components of the split cluster's entity.
//!
//! Modifying the [`SoftBody`] component after the soft body was created has no effect. Use the
//! other components of this module, or the helpers of [`RapierRigidBodySet`] and
//! [`RapierContextMut`], to control it at runtime. Removing the [`SoftBody`] component or
//! despawning its entity removes the soft body from the simulation.
//!
//! [`RapierRigidBodySet`]: crate::plugin::context::RapierRigidBodySet
//! [`RapierRigidBodySet::soft_body_whole_proxy`]: crate::plugin::context::RapierRigidBodySet::soft_body_whole_proxy
//! [`RapierContextMut`]: crate::plugin::context::systemparams::RapierContextMut

use crate::math::{Real, Vect};
use bevy::prelude::*;

#[cfg(feature = "dim3")]
pub use rapier::dynamics::SoftBodyDihedral;
pub use rapier::dynamics::{
    SoftBindingError, SoftBodiesSettings, SoftBody as RapierSoftBody, SoftBodyBuilder,
    SoftBodyCell, SoftBodyCellModel, SoftBodyCluster as RapierSoftBodyCluster, SoftBodyEdge,
    SoftBodyEdgeKind, SoftBodyHandle, SoftBodyMaterial as RapierSoftBodyMaterial, SoftBodyParticle,
    SoftBodyParticleSettings, SoftBodyPiece, SoftBodyTearEvent as RapierSoftBodyTearEvent,
    SoftClusterRemoval, SoftClusterSplit, SoftCollisionMesh, SoftEdgePlasticFlow, SoftJointMove,
    SoftMeshBinding, SoftMeshBindingMode, SoftMeshId, SoftMeshRef, SoftParticleAttachment,
    SoftPatchConstraints, SoftRecoverySettings, SoftVolumePiece,
};
#[cfg(feature = "fem")]
pub use rapier::dynamics::{SoftBodySolver, SoftFemParameters};
pub use rapier::geometry::{PairContacts, SoftContactImpulse, SoftPairContacts};
use rapier::prelude::SpringCoefficients;

#[cfg(doc)]
use crate::prelude::{Collider, ImpulseJoint, RigidBody};

/// Describes a soft body to create.
///
/// The particle positions of [`Self::builder`] are expressed in the local frame of the entity (see
/// the [module documentation](self) for the conventions). Changes made to this component after
/// the soft body was created are ignored.
///
/// The collider-related components of the entity ([`crate::geometry::Friction`],
/// [`crate::geometry::Restitution`], [`crate::geometry::CollisionGroups`],
/// [`crate::geometry::SolverGroups`], [`crate::geometry::ActiveEvents`],
/// [`crate::geometry::ActiveHooks`], [`crate::geometry::ActiveCollisionTypes`],
/// [`crate::geometry::ContactForceEventThreshold`], [`crate::geometry::ContactSkin`] and
/// [`crate::geometry::Sensor`]) configure the soft body's surface colliders (see
/// [`RapierRigidBodySet::soft_body_colliders`]), like for a [`Collider`]: they override the
/// builder's collider template, their changes are applied, and removing them restores the
/// template's values (the particle radius for the contact skin). The `user_data` of the builder
/// and of its collider template are overwritten with the entity bits.
///
/// [`RapierRigidBodySet::soft_body_colliders`]: crate::plugin::context::RapierRigidBodySet::soft_body_colliders
#[derive(Component, Clone, Debug)]
pub struct SoftBody {
    /// The Rapier builder of the soft body, in the local frame of the entity.
    pub builder: SoftBodyBuilder,
}

impl From<SoftBodyBuilder> for SoftBody {
    fn from(builder: SoftBodyBuilder) -> Self {
        Self::new(builder)
    }
}

impl SoftBody {
    /// A soft body created from the given Rapier builder (positions in the entity's local frame).
    pub fn new(builder: SoftBodyBuilder) -> Self {
        Self { builder }
    }

    /// Applies `f` to the builder of this soft body.
    ///
    /// This is useful to chain the builder's methods, e.g.
    /// `SoftBody::rope(a, b, 20).map(|b| b.particle_mass(0.1).pinned_particles([0]))`.
    pub fn map(mut self, f: impl FnOnce(SoftBodyBuilder) -> SoftBodyBuilder) -> Self {
        self.builder = f(self.builder);
        self
    }

    /// A rope of `num_particles` particles from `start` to `end` (see [`SoftBodyBuilder::rope`]).
    pub fn rope(start: Vect, end: Vect, num_particles: usize) -> Self {
        Self::new(SoftBodyBuilder::rope(start, end, num_particles))
    }

    /// A triangle-mesh soft body without volumetric cells (see [`SoftBodyBuilder::trimesh`]).
    ///
    /// Returns `None` if the mesh is empty.
    pub fn trimesh(vertices: Vec<Vect>, indices: Vec<[u32; 3]>) -> Option<Self> {
        SoftBodyBuilder::trimesh(vertices, indices).map(Self::new)
    }

    /// A volumetric soft body filling a closed, outward-oriented mesh (triangles in 3D, segments
    /// in 2D) with cells of size `cell_size` (see [`SoftBodyBuilder::volumetric`]).
    ///
    /// Returns `None` if the mesh is empty, open, or encloses nothing at that cell size.
    #[cfg(feature = "dim3")]
    pub fn volumetric(vertices: &[Vect], indices: &[[u32; 3]], cell_size: Real) -> Option<Self> {
        SoftBodyBuilder::volumetric(vertices, indices, cell_size).map(Self::new)
    }

    /// A volumetric soft body filling a closed, outward-oriented mesh (triangles in 3D, segments
    /// in 2D) with cells of size `cell_size` (see [`SoftBodyBuilder::volumetric`]).
    ///
    /// Returns `None` if the mesh is empty, open, or encloses nothing at that cell size.
    #[cfg(feature = "dim2")]
    pub fn volumetric(vertices: &[Vect], indices: &[[u32; 2]], cell_size: Real) -> Option<Self> {
        SoftBodyBuilder::volumetric(vertices, indices, cell_size).map(Self::new)
    }

    /// Same as [`Self::volumetric`], keeping the given mesh as the soft body's skin (see
    /// [`SoftBodyBuilder::volumetric_skinned`]).
    #[cfg(feature = "dim3")]
    pub fn volumetric_skinned(
        vertices: &[Vect],
        indices: &[[u32; 3]],
        cell_size: Real,
    ) -> Option<Self> {
        SoftBodyBuilder::volumetric_skinned(vertices, indices, cell_size).map(Self::new)
    }

    /// Same as [`Self::volumetric`], keeping the given polyline as the soft body's skin (see
    /// [`SoftBodyBuilder::volumetric_skinned`]).
    #[cfg(feature = "dim2")]
    pub fn volumetric_skinned(
        vertices: &[Vect],
        indices: &[[u32; 2]],
        cell_size: Real,
    ) -> Option<Self> {
        SoftBodyBuilder::volumetric_skinned(vertices, indices, cell_size).map(Self::new)
    }

    /// A rectangular cloth of `nu × nv` particles at `origin + i * du + j * dv` (see
    /// [`SoftBodyBuilder::cloth`]).
    #[cfg(feature = "dim3")]
    pub fn cloth(origin: Vect, du: Vect, dv: Vect, nu: usize, nv: usize) -> Self {
        Self::new(SoftBodyBuilder::cloth(origin, du, dv, nu, nv))
    }

    /// A cloth tube around the segment from `origin` to `origin + axis` (see
    /// [`SoftBodyBuilder::cloth_tube`]).
    #[cfg(feature = "dim3")]
    pub fn cloth_tube(
        origin: Vect,
        axis: Vect,
        radius_start: Real,
        radius_end: Real,
        num_around: usize,
        num_along: usize,
    ) -> Self {
        Self::new(SoftBodyBuilder::cloth_tube(
            origin,
            axis,
            radius_start,
            radius_end,
            num_around,
            num_along,
        ))
    }

    /// A solid box centered at the entity's origin, tetrahedralized on a `nx × ny × nz` particle
    /// grid (see [`SoftBodyBuilder::cuboid`]).
    #[cfg(feature = "dim3")]
    pub fn cuboid(half_extents: Vect, nx: usize, ny: usize, nz: usize) -> Self {
        Self::new(SoftBodyBuilder::cuboid(
            Vect::ZERO,
            half_extents,
            nx,
            ny,
            nz,
        ))
    }

    /// A hollow sphere centered at the entity's origin, with volume preservation (see
    /// [`SoftBodyBuilder::sphere`]).
    #[cfg(feature = "dim3")]
    pub fn sphere(radius: Real, subdivisions: usize) -> Self {
        Self::new(SoftBodyBuilder::sphere(Vect::ZERO, radius, subdivisions))
    }

    /// A closed polygon with counter-clockwise `points`, with area preservation (see
    /// [`SoftBodyBuilder::polygon`]).
    #[cfg(feature = "dim2")]
    pub fn polygon(points: Vec<Vect>) -> Self {
        Self::new(SoftBodyBuilder::polygon(points))
    }

    /// A regular polygon centered at the entity's origin with `num_particles` boundary particles
    /// (see [`SoftBodyBuilder::disk`]).
    #[cfg(feature = "dim2")]
    pub fn disk(radius: Real, num_particles: usize) -> Self {
        Self::new(SoftBodyBuilder::disk(Vect::ZERO, radius, num_particles))
    }

    /// A solid rectangle centered at the entity's origin, triangulated on a `nx × ny` particle
    /// grid (see [`SoftBodyBuilder::grid`]).
    #[cfg(feature = "dim2")]
    pub fn grid(half_extents: Vect, nx: usize, ny: usize) -> Self {
        Self::new(SoftBodyBuilder::grid(Vect::ZERO, half_extents, nx, ny))
    }

    /// A soft body from a polyline (see [`SoftBodyBuilder::polyline`]).
    ///
    /// Returns `None` if the polyline is empty.
    #[cfg(feature = "dim2")]
    pub fn polyline(vertices: Vec<Vect>, indices: Option<Vec<[u32; 2]>>) -> Option<Self> {
        SoftBodyBuilder::polyline(vertices, indices).map(Self::new)
    }
}

/// The Rapier handle of a [`SoftBody`] that was inserted to the physics scene.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Component)]
pub struct RapierSoftBodyHandle(pub SoftBodyHandle);

/// The material (stiffness, damping, plasticity and tearing) of a soft body.
///
/// It overrides the material of the [`SoftBody`] builder, and changes to it are applied to the
/// soft body. When removed, the builder's material is restored.
#[derive(Component, Copy, Clone, Debug, PartialEq, Default, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyMaterial(
    /// The Rapier material.
    #[reflect(remote = crate::reflect::SoftBodyMaterialWrapper)]
    pub RapierSoftBodyMaterial,
);

impl SoftBodyMaterial {
    /// A material with the same softness for every spring family (see
    /// [`RapierSoftBodyMaterial::uniform`]).
    pub fn uniform(natural_frequency: Real, damping_ratio: Real) -> Self {
        Self(RapierSoftBodyMaterial::uniform(
            rapier::dynamics::SpringCoefficients::new(natural_frequency, damping_ratio),
        ))
    }
}

impl From<RapierSoftBodyMaterial> for SoftBodyMaterial {
    fn from(material: RapierSoftBodyMaterial) -> Self {
        Self(material)
    }
}

impl std::ops::Deref for SoftBodyMaterial {
    type Target = RapierSoftBodyMaterial;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for SoftBodyMaterial {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Add this component to disable a soft body: it is left where it is, without colliders or
/// constraints, until this component is removed.
#[derive(Component, Copy, Clone, Debug, Default, Reflect)]
#[reflect(Component, Default)]
pub struct SoftBodyDisabled;

/// The particles of a soft body that are pinned (kinematic).
///
/// While this component is present, exactly the listed particles are pinned (plus the particles
/// of the clusters with a [`SoftBodyClusterPinned`] component): they hold their position, or
/// follow the targets of [`SoftBodyKinematicTargets`]. When removed, the particles pinned by the
/// [`SoftBody`] builder are restored. Invalid indices are ignored.
#[derive(Component, Clone, Debug, Default, PartialEq, Eq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyPinnedParticles(pub Vec<u32>);

/// World-space kinematic targets of pinned particles, as `(particle index, target)` pairs.
///
/// Each time this component changes, every listed pinned particle is moved to its target over
/// the next simulation step (with the matching velocity, so friction and contacts see the
/// motion), then held there. Targets of free particles are ignored.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyKinematicTargets(pub Vec<(u32, Vect)>);

/// A particle of a soft body attached to a rigid-body entity (see [`SoftBodyAttachments`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Reflect)]
pub struct SoftBodyAttachment {
    /// The index of the attached particle.
    pub particle: u32,
    /// The entity of the [`RigidBody`] (or [`SoftBodyCluster`]) the particle is attached to.
    pub body: Entity,
}

/// Particles of a soft body attached to rigid-bodies by two-way point-to-point constraints.
///
/// The anchor of each attachment is the particle's position, relative to the rigid-body, at the
/// time the attachment is created. Each time this component changes, the attachments of the soft
/// body are updated to match it: the particles whose attached bodies changed are detached and
/// attached again (with new anchors), the other attachments keep their anchor. When it is
/// removed, every particle is detached. Attachments to entities that don't have a rigid-body yet
/// are retried on the next frames.
#[derive(Component, Clone, Debug, Default, PartialEq, Eq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyAttachments(pub Vec<SoftBodyAttachment>);

/// Persistent forces applied to the particles of a soft body.
///
/// The forces are applied at each simulation step until this component changes or is removed,
/// like [`crate::dynamics::ExternalForce`]. Pinned particles ignore them.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyExternalForce {
    /// The force applied to every particle.
    pub force: Vect,
    /// Additional forces applied to individual particles, as `(particle index, force)` pairs.
    pub particle_forces: Vec<(u32, Vect)>,
}

/// One-time impulses applied to the particles of a soft body.
///
/// The impulses are applied (and this component reset to zero) at the next simulation step,
/// like [`crate::dynamics::ExternalImpulse`]. Pinned particles ignore them.
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyExternalImpulse {
    /// A velocity change applied to every particle, i.e., an impulse per unit of mass: the whole
    /// soft body is kicked at the same velocity.
    pub velocity_change: Vect,
    /// Impulses applied to individual particles, as `(particle index, impulse)` pairs: the
    /// velocity of each particle changes by its impulse divided by its mass.
    pub particle_impulses: Vec<(u32, Vect)>,
}

impl SoftBodyExternalImpulse {
    /// Resets these impulses to zero.
    pub fn reset(&mut self) {
        self.velocity_change = Vect::ZERO;
        self.particle_impulses.clear();
    }
}

/// The multiplier applied to the rest area (2D) or volume (3D) of a soft body to obtain the
/// target of its volume preservation (`> 1` inflates the soft body).
///
/// It overrides the volume factor of the [`SoftBody`] builder. When removed, the builder's volume
/// factor is restored.
#[derive(Component, Copy, Clone, Debug, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyVolumeFactor(pub Real);

impl Default for SoftBodyVolumeFactor {
    fn default() -> Self {
        Self(1.0)
    }
}

/// Selects the solver simulating the elasticity of a soft body.
///
/// It overrides the solver of the [`SoftBody`] builder. When removed, the builder's solver is
/// restored.
#[cfg(feature = "fem")]
#[derive(Component, Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct SoftBodyElasticitySolver(pub SoftBodySolver);

/// The state of a soft body, written back after each simulation step.
///
/// This component is inserted automatically on every soft body entity. Modifying it has no
/// effect.
#[derive(Component, Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyState {
    /// The world-space, mass-weighted, center of the particles.
    pub center_of_mass: Vect,
    /// Whether the soft body is sleeping.
    pub is_sleeping: bool,
    /// Whether the soft body takes part in the simulation (see [`SoftBodyDisabled`]; Rapier also
    /// disables soft bodies with non-finite state).
    pub is_enabled: bool,
    /// A counter bumped whenever the topology of the soft body changes (e.g. because of a tear).
    pub topology_version: u32,
    /// The number of particles of the soft body.
    pub num_particles: usize,
}

/// A cluster of particles of a soft body, backed by a proxy rigid-body.
///
/// Insert this component on an entity (usually not the soft body entity itself) to create the
/// cluster: the entity then gets a [`crate::dynamics::RapierRigidBodyHandle`] of the cluster's
/// proxy rigid-body, so [`ImpulseJoint`]s and [`Collider`]s can be attached to it like to any
/// rigid-body. The proxy follows the particles of the cluster: its pose is written back to the
/// entity's [`Transform`], but changes of the [`Transform`] (or velocity, forces...) of the
/// entity are not applied to it. Do not add a [`RigidBody`] component to this entity. A
/// [`Transform`] (and a `Visibility` when a rendering feature is enabled) is inserted if missing,
/// so rendered children can be added to the entity.
///
/// Removing this component or despawning its entity removes the cluster (particles covered only
/// by this cluster are removed with it). Changes made to this component after the cluster was
/// created are ignored, except for [`Self::soft_body`] which is updated by the plugin when a tear
/// moves the cluster to another soft body.
#[derive(Component, Clone, Debug, PartialEq, Eq, Reflect)]
#[reflect(Component, PartialEq)]
#[cfg_attr(
    any(
        feature = "debug-render-2d",
        feature = "debug-render-3d",
        feature = "picking-backend",
        feature = "async-collider",
        feature = "to-bevy-mesh"
    ),
    require(Transform, Visibility)
)]
#[cfg_attr(
    not(any(
        feature = "debug-render-2d",
        feature = "debug-render-3d",
        feature = "picking-backend",
        feature = "async-collider",
        feature = "to-bevy-mesh"
    )),
    require(Transform)
)]
pub struct SoftBodyCluster {
    /// The entity of the soft body the cluster belongs to.
    pub soft_body: Entity,
    /// The indices of the particles of the cluster.
    pub particles: Vec<u32>,
}

impl SoftBodyCluster {
    /// A cluster of the given particles of `soft_body`.
    pub fn new(soft_body: Entity, particles: impl IntoIterator<Item = u32>) -> Self {
        Self {
            soft_body,
            particles: particles.into_iter().collect(),
        }
    }
}

/// Pins every particle of a cluster: the cluster becomes a fixed region dragging the rest of its
/// soft body, and can be moved with a [`SoftBodyClusterKinematicTarget`].
///
/// Insert it on a [`SoftBodyCluster`] entity, or on a [`SoftBody`] entity to pin its whole-body
/// cluster. The pinned particles add up to the ones of the soft body's
/// [`SoftBodyPinnedParticles`]. When removed, the particles of the cluster get back the pinned
/// state given by the [`SoftBodyPinnedParticles`] of the soft body (or its [`SoftBody`] builder)
/// and by the other pinned clusters.
#[derive(Component, Copy, Clone, Debug, Default, PartialEq, Eq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyClusterPinned;

/// The world-space pose a pinned cluster is moved to.
///
/// Each time this component changes, the pinned particles of the cluster are moved rigidly over
/// the next simulation step (with matching velocities) so that the rest shape of the cluster,
/// centered on its rest centroid, is placed at this pose; then they are held there. Only the
/// translation and rotation of the [`Transform`] are used. Inserting it also inserts
/// [`SoftBodyClusterPinned`], since only pinned particles follow the target.
///
/// Insert it on a [`SoftBodyCluster`] entity, or on a [`SoftBody`] entity for its whole-body
/// cluster. Removing it leaves the particles where they are.
#[derive(Component, Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
#[require(SoftBodyClusterPinned)]
pub struct SoftBodyClusterKinematicTarget(pub Transform);

/// Enables the shape matching of the particles of a cluster toward the cluster's frame, or
/// toward a target pose.
///
/// Insert it on a [`SoftBodyCluster`] entity, or on a [`SoftBody`] entity for its whole-body
/// cluster (whose shape matching can also be enabled by its [`SoftBody`] builder). Changes are
/// applied to the cluster. When removed, the shape matching of the cluster is disabled (or reset
/// to the builder's setting for a whole-body cluster) and its target is cleared.
#[derive(Component, Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftBodyClusterShapeMatching {
    /// The world-space pose the particles are shape-matched toward (only its translation and
    /// rotation are used), or `None` to shape-match them toward the cluster's own frame.
    pub target: Option<Transform>,
}

/// Regional material overrides of the elements (cells and edges) fully contained in a cluster.
///
/// Insert it on a [`SoftBodyCluster`] entity, or on a [`SoftBody`] entity for its whole-body
/// cluster. Changes are applied to the elements of the cluster. When removed, the default values
/// are applied back, which also discards the per-element tear resistances given by the
/// [`SoftBody`] builder for these elements.
#[derive(Component, Copy, Clone, Debug, PartialEq)]
pub struct SoftBodyClusterMaterial {
    /// Multiplier of the stiffness (Young modulus) of the cells of the cluster (default: `1.0`).
    pub stiffness_scale: Real,
    /// Softness overriding the one of the material for the structural and bending edges of the
    /// cluster (default: `None`, the material's softness).
    pub edge_softness: Option<SpringCoefficients<Real>>,
    /// Multiplier of the tear thresholds of the edges and cells of the cluster (default: `1.0`).
    pub tear_resistance: Real,
}

impl Default for SoftBodyClusterMaterial {
    fn default() -> Self {
        Self {
            stiffness_scale: 1.0,
            edge_softness: None,
            tear_resistance: 1.0,
        }
    }
}

/// Turns the [`Collider`] of this entity into a deformable collider bound to the particles of a
/// soft body.
///
/// The collider shape must be a polyline (2D) or a triangle mesh (3D) with the deformable flag
/// (`PolylineFlags::DEFORMABLE`, `TriMeshFlags::DEFORMABLE`). Its vertices, placed by the
/// [`GlobalTransform`] of the entity when it is created, are bound to the particles of `target`
/// according to `binding`. After its creation, the collider follows the particles: changes to the
/// [`Transform`] and shape of the entity are ignored, but the other collider components (friction,
/// collision groups, events...) apply as usual.
///
/// If the binding fails, an error is logged and a [`DeformableColliderError`] is inserted.
#[derive(Component, Clone, Debug)]
pub struct DeformableCollider {
    /// The entity of the [`SoftBody`] (the collider is bound to its whole-body cluster) or of the
    /// [`SoftBodyCluster`] the collider is bound to.
    pub target: Entity,
    /// How the collider's vertices are bound to the particles.
    pub binding: SoftMeshBinding,
}

impl DeformableCollider {
    /// A deformable collider bound to `target` according to `binding`.
    pub fn new(target: Entity, binding: SoftMeshBinding) -> Self {
        Self { target, binding }
    }
}

/// Inserted on an entity whose [`DeformableCollider`] could not be created.
///
/// Remove it (after fixing the collider or its binding) to try again.
#[derive(Component, Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeformableColliderError(pub SoftBindingError);

/// Add this component to a [`SoftBody`] entity to render the soft body with a mesh kept in sync
/// with its particles.
///
/// The plugin generates the mesh (see [`soft_body_mesh`] for the geometry: the skin or surface
/// of the soft body in 3D, its cells in 2D; never the meshes of [`DeformableCollider`]s) and
/// inserts it as a `Mesh3d` (3D) or `Mesh2d` (2D) component if the entity doesn't have one
/// already (the existing mesh asset is overwritten otherwise). After each simulation step, the
/// vertex positions (and normals in 3D) are updated from the particles, and the mesh is rebuilt
/// when the topology of the soft body changes (e.g. after a tear). The vertices are expressed in
/// the frame of the entity's [`Transform`] (see the [module documentation](self)). A material
/// must be added by the user.
#[cfg(feature = "to-bevy-mesh")]
#[derive(Component, Clone, Debug, Default)]
pub struct SoftBodyMeshSync {
    pub(crate) built: Option<SoftBodyMeshSignature>,
}

/// What a synchronized soft-body mesh was built from: it is rebuilt when this changes.
#[cfg(feature = "to-bevy-mesh")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct SoftBodyMeshSignature {
    pub handle: SoftBodyHandle,
    pub topology_version: u32,
    pub num_vertices: usize,
    pub num_indices: usize,
}

/// The geometry used to render a soft body.
#[cfg(feature = "to-bevy-mesh")]
pub(crate) struct SoftBodyRenderGeometry {
    /// World-space vertex positions.
    pub vertices: Vec<Vect>,
    /// The indices of the triangles (triangle list) or segments (line list).
    pub indices: Vec<u32>,
    /// Whether the indices describe triangles (`true`) or segments.
    pub triangles: bool,
}

#[cfg(feature = "to-bevy-mesh")]
impl SoftBodyRenderGeometry {
    /// The geometry rendering `soft_body`, whose colliders are in `colliders` (see
    /// [`soft_body_mesh`]).
    pub fn new(soft_body: &RapierSoftBody, colliders: &rapier::geometry::ColliderSet) -> Self {
        let vertices: Vec<Vect> = soft_body.particle_positions().collect();

        #[cfg(feature = "dim3")]
        {
            // The meshes created with the soft body (the colliders of the meshes added later,
            // e.g. by `DeformableCollider`s, stand for other entities).
            let own = |mesh: &&SoftCollisionMesh| {
                !mesh.collision_enabled()
                    || colliders
                        .get(mesh.collider())
                        .is_some_and(|co| co.user_data == soft_body.user_data)
            };
            let surface = |skinned: bool| {
                soft_body.meshes().filter(own).find(|mesh| {
                    mesh.arity() == 3 && !mesh.indices().is_empty() && mesh.is_skinned() == skinned
                })
            };
            if let Some(mesh) = surface(true).or_else(|| surface(false)) {
                return Self {
                    vertices: mesh.vertex_positions(soft_body).collect(),
                    indices: mesh.indices().iter().flatten().copied().collect(),
                    triangles: true,
                };
            }

            if !soft_body.boundary().is_empty() {
                return Self {
                    vertices,
                    indices: soft_body.boundary().iter().flatten().copied().collect(),
                    triangles: true,
                };
            }
        }
        #[cfg(feature = "dim2")]
        let _ = colliders;

        #[cfg(feature = "dim2")]
        {
            if !soft_body.cells().is_empty() {
                return Self {
                    vertices,
                    indices: soft_body
                        .cells()
                        .iter()
                        .flat_map(|cell| cell.vertices)
                        .collect(),
                    triangles: true,
                };
            }

            if !soft_body.boundary().is_empty() {
                return Self {
                    vertices,
                    indices: soft_body.boundary().iter().flatten().copied().collect(),
                    triangles: false,
                };
            }
        }

        Self {
            vertices,
            indices: soft_body
                .edges()
                .iter()
                .filter(|edge| edge.kind == SoftBodyEdgeKind::Structural)
                .flat_map(|edge| edge.vertices)
                .collect(),
            triangles: false,
        }
    }

    /// The mesh with this geometry, with its vertices transformed by `transform`.
    pub fn to_mesh(&self, transform: &bevy::math::Affine3A) -> bevy::mesh::Mesh {
        use bevy::asset::RenderAssetUsages;
        use bevy::mesh::{Indices, Mesh, PrimitiveTopology};

        let topology = if self.triangles {
            PrimitiveTopology::TriangleList
        } else {
            PrimitiveTopology::LineList
        };
        let mut mesh = Mesh::new(topology, RenderAssetUsages::default());
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.positions(transform));
        mesh.insert_indices(Indices::U32(self.indices.clone()));
        if self.triangles && !self.indices.is_empty() {
            mesh.compute_smooth_normals();
        }
        mesh
    }

    /// The vertex positions transformed by `transform`, as a Bevy mesh attribute.
    pub fn positions(&self, transform: &bevy::math::Affine3A) -> Vec<[f32; 3]> {
        self.vertices
            .iter()
            .map(|v| {
                #[cfg(feature = "dim2")]
                let v = v.extend(0.0);
                #[cfg(feature = "dim3")]
                let v = *v;
                transform.transform_point3(v).to_array()
            })
            .collect()
    }
}

/// A mesh rendering the given Rapier soft body, with world-space vertices.
///
/// `colliders` is the collider set of the soft body's context: it tells apart the meshes created
/// with the soft body from the ones added later by [`DeformableCollider`]s, which are not
/// rendered. The rendered geometry is chosen in this order:
/// - In 3D: the soft body's skin (see [`SoftBodyBuilder::volumetric_skinned`]), else its surface
///   collision mesh, else its boundary triangles; for a soft body without surface (e.g. a rope),
///   a line list of its structural edges. The mesh has normals.
/// - In 2D: a triangle mesh of its cells (even if its skin is the collision mesh: the skin is
///   embedded in the cells and cannot be filled), else a line list of its boundary, else of its
///   structural edges.
///
/// The mesh can be modified after being added to the assets (it keeps its main-world data).
#[cfg(feature = "to-bevy-mesh")]
pub fn soft_body_mesh(
    soft_body: &RapierSoftBody,
    colliders: &rapier::geometry::ColliderSet,
) -> bevy::mesh::Mesh {
    SoftBodyRenderGeometry::new(soft_body, colliders).to_mesh(&bevy::math::Affine3A::IDENTITY)
}
