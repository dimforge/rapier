use crate::geometry::contact_generator::{
    ContactGenerator, ContactPhase, HeightFieldShapeContactGeneratorWorkspace,
    PrimitiveContactGenerator, TrimeshShapeContactGeneratorWorkspace,
};
use crate::geometry::Shape;
use std::any::Any;

/// Trait implemented by structures responsible for selecting a collision-detection algorithm
/// for a given pair of shapes.
pub trait ContactDispatcher {
    /// Select the collision-detection algorithm for the given pair of primitive shapes.
    fn dispatch_primitives(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (
        PrimitiveContactGenerator,
        Option<Box<dyn Any + Send + Sync>>,
    );
    /// Select the collision-detection algorithm for the given pair of non-primitive shapes.
    fn dispatch(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (ContactPhase, Option<Box<dyn Any + Send + Sync>>);
}

/// The default contact dispatcher used by Rapier.
pub struct DefaultContactDispatcher;

impl ContactDispatcher for DefaultContactDispatcher {
    fn dispatch_primitives(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (
        PrimitiveContactGenerator,
        Option<Box<dyn Any + Send + Sync>>,
    ) {
        match (shape1, shape2) {
            (Shape::Ball(_), Shape::Ball(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_ball_ball,
                    #[cfg(feature = "simd-is-enabled")]
                    generate_contacts_simd: super::generate_contacts_ball_ball_simd,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Cuboid(_), Shape::Cuboid(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_cuboid_cuboid,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Polygon(_), Shape::Polygon(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_polygon_polygon,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Capsule(_), Shape::Capsule(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_capsule_capsule,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Cuboid(_), Shape::Ball(_))
            | (Shape::Ball(_), Shape::Cuboid(_))
            | (Shape::Triangle(_), Shape::Ball(_))
            | (Shape::Ball(_), Shape::Triangle(_))
            | (Shape::Capsule(_), Shape::Ball(_))
            | (Shape::Ball(_), Shape::Capsule(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_ball_convex,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Capsule(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Capsule(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_cuboid_capsule,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            (Shape::Triangle(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Triangle(_)) => (
                PrimitiveContactGenerator {
                    generate_contacts: super::generate_contacts_cuboid_triangle,
                    ..PrimitiveContactGenerator::default()
                },
                None,
            ),
            _ => (PrimitiveContactGenerator::default(), None),
        }
    }

    fn dispatch(
        &self,
        shape1: &Shape,
        shape2: &Shape,
    ) -> (ContactPhase, Option<Box<dyn Any + Send + Sync>>) {
        match (shape1, shape2) {
            (Shape::Trimesh(_), _) | (_, Shape::Trimesh(_)) => (
                ContactPhase::NearPhase(ContactGenerator {
                    generate_contacts: super::generate_contacts_trimesh_shape,
                    ..ContactGenerator::default()
                }),
                Some(Box::new(TrimeshShapeContactGeneratorWorkspace::new())),
            ),
            (Shape::HeightField(_), _) | (_, Shape::HeightField(_)) => (
                ContactPhase::NearPhase(ContactGenerator {
                    generate_contacts: super::generate_contacts_heightfield_shape,
                    ..ContactGenerator::default()
                }),
                Some(Box::new(HeightFieldShapeContactGeneratorWorkspace::new())),
            ),
            _ => {
                let (gen, workspace) = self.dispatch_primitives(shape1, shape2);
                (ContactPhase::ExactPhase(gen), workspace)
            }
        }
    }
}
