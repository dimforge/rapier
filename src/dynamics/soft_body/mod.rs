//! Soft bodies: deformable bodies made of particles linked by elastic constraints, solved
//! together with rigid bodies, contacts and joints.

pub use self::soft_body_cluster::{SoftBodyCluster, SoftClusterRemoval};
pub(crate) use self::soft_body_cluster::{inertia_noise_floor, pseudo_inverse_inertia};
pub use self::collision_mesh::{
    SoftBindingError, SoftCollisionMesh, SoftMeshBinding, SoftMeshBindingMode, SoftMeshCellBinding,
    SoftMeshId, SoftMeshMapping, SoftMeshRef,
};
#[cfg(feature = "dim3")]
pub use self::soft_body_elements::SoftBodyDihedral;
#[cfg(feature = "fem")]
pub use self::soft_body_elements::SoftBodySolver;
pub use self::soft_body_contacts::SoftVolumeContact;
pub use self::soft_body::{SOFT_BODY_MAX_CONSTRAINT_PARTICLES, SoftBody, SoftVolumePiece};
pub use self::soft_body_elements::{
    SoftBodyCell, SoftBodyCellModel, SoftBodyEdge, SoftBodyEdgeKind, SoftBodyParticle,
};
pub use self::soft_body_material::SoftBodyMaterial;
pub use self::soft_body_motion::SoftParticleAttachment;
pub use self::soft_body_builder::{SoftBodyBuilder, SoftBodyParticleSettings};
pub use self::soft_body_handle::SoftBodyHandle;
pub use self::soft_body_set::SoftBodySet;
pub use self::soft_body_plasticity::SoftEdgePlasticFlow;

pub(crate) use self::soft_body::SOFT_BODY_OVERFLOW_COLOR;
pub(crate) use self::soft_body_contacts::{
    SoftEdgeContact, SoftOverlapState, SoftOverlapWarm, SoftVertexContact,
};
use self::soft_body_elements::CELL_IMPULSES;
mod soft_recovery_settings;
pub use soft_recovery_settings::{SoftPatchConstraints, SoftRecoverySettings};
mod soft_body_settings;
#[cfg(feature = "fem")]
pub use soft_body_settings::SoftFemParameters;
pub use soft_body_settings::SoftBodiesSettings;

pub(crate) mod collision_mesh;
mod soft_body;
mod soft_body_accessors;
mod soft_body_builder;
mod soft_body_cluster;
mod soft_body_coloring;
mod soft_body_contacts;
pub(crate) mod soft_body_crossing_tests;
mod soft_body_elements;
mod soft_body_geometry;
mod soft_body_handle;
mod soft_body_material;
mod soft_body_meshes;
mod soft_body_motion;
mod soft_body_plasticity;
mod soft_body_set;
pub(crate) mod soft_body_shape_matching;
mod tearing;
pub use self::tearing::{SoftBodyPiece, SoftBodyTearEvent, SoftClusterSplit, SoftJointMove};
