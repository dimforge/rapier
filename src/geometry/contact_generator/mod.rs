pub use self::ball_ball_contact_generator::generate_contacts_ball_ball;
#[cfg(feature = "simd-is-enabled")]
pub use self::ball_ball_contact_generator::generate_contacts_ball_ball_simd;
pub use self::ball_convex_contact_generator::generate_contacts_ball_convex;
pub use self::capsule_capsule_contact_generator::generate_contacts_capsule_capsule;
pub use self::contact_dispatcher::{ContactDispatcher, DefaultContactDispatcher};
pub use self::contact_generator::{
    ContactGenerationContext, ContactGenerator, ContactPhase, PrimitiveContactGenerationContext,
    PrimitiveContactGenerator,
};
#[cfg(feature = "simd-is-enabled")]
pub use self::contact_generator::{
    ContactGenerationContextSimd, PrimitiveContactGenerationContextSimd,
};
pub use self::cuboid_capsule_contact_generator::generate_contacts_cuboid_capsule;
pub use self::cuboid_cuboid_contact_generator::generate_contacts_cuboid_cuboid;
pub use self::cuboid_triangle_contact_generator::generate_contacts_cuboid_triangle;
pub use self::heightfield_shape_contact_generator::{
    generate_contacts_heightfield_shape, HeightFieldShapeContactGeneratorWorkspace,
};
#[cfg(feature = "dim3")]
pub use self::pfm_pfm_contact_generator::{
    generate_contacts_pfm_pfm, PfmPfmContactManifoldGeneratorWorkspace,
};
// pub use self::polygon_polygon_contact_generator::generate_contacts_polygon_polygon;
pub use self::contact_generator_workspace::ContactGeneratorWorkspace;
pub use self::trimesh_shape_contact_generator::{
    generate_contacts_trimesh_shape, TrimeshShapeContactGeneratorWorkspace,
};

pub(self) use self::serializable_workspace_tag::WorkspaceSerializationTag;

mod ball_ball_contact_generator;
mod ball_convex_contact_generator;
mod ball_polygon_contact_generator;
mod capsule_capsule_contact_generator;
mod contact_dispatcher;
mod contact_generator;
mod contact_generator_workspace;
mod cuboid_capsule_contact_generator;
mod cuboid_cuboid_contact_generator;
mod cuboid_polygon_contact_generator;
mod cuboid_triangle_contact_generator;
mod heightfield_shape_contact_generator;
#[cfg(feature = "dim3")]
mod pfm_pfm_contact_generator;
mod polygon_polygon_contact_generator;
mod serializable_workspace_tag;
mod trimesh_shape_contact_generator;

use crate::geometry::{Contact, ContactManifold};
