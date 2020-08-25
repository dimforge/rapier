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
pub use self::polygon_polygon_contact_generator::generate_contacts_polygon_polygon;
pub use self::trimesh_shape_contact_generator::{
    generate_contacts_trimesh_shape, TrimeshShapeContactGeneratorWorkspace,
};

#[cfg(feature = "dim2")]
pub(crate) use self::polygon_polygon_contact_generator::{
    clip_segments, clip_segments_with_normal,
};

mod ball_ball_contact_generator;
mod ball_convex_contact_generator;
mod ball_polygon_contact_generator;
mod capsule_capsule_contact_generator;
mod contact_dispatcher;
mod contact_generator;
mod cuboid_capsule_contact_generator;
mod cuboid_cuboid_contact_generator;
mod cuboid_polygon_contact_generator;
mod cuboid_triangle_contact_generator;
mod heightfield_shape_contact_generator;
mod polygon_polygon_contact_generator;
mod trimesh_shape_contact_generator;

use crate::geometry::{Contact, ContactManifold};

pub(crate) fn match_contacts(
    manifold: &mut ContactManifold,
    old_contacts: &[Contact],
    swapped: bool,
) {
    for contact in &mut manifold.points {
        if !swapped {
            for old_contact in old_contacts {
                if contact.fid1 == old_contact.fid1 && contact.fid2 == old_contact.fid2 {
                    // Transfer impulse cache.
                    contact.impulse = old_contact.impulse;
                    contact.tangent_impulse = old_contact.tangent_impulse;
                }
            }
        } else {
            for old_contact in old_contacts {
                if contact.fid1 == old_contact.fid2 && contact.fid2 == old_contact.fid1 {
                    // Transfer impulse cache.
                    contact.impulse = old_contact.impulse;
                    contact.tangent_impulse = old_contact.tangent_impulse;
                }
            }
        }
    }
}
