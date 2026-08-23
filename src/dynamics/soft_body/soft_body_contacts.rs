//! The contact records a soft body keeps between steps (edge, vertex, overlap and volume contacts) and their debug-render accessors.
use crate::alloc_prelude::*;
use super::{SoftBody, SoftCollisionMesh, SoftMeshRef};
use crate::math::{AngVector, DIM, Pose, Real, Vector};

/// An edge-vs-edge contact between two surface edges (segments in 2D, triangle edges in 3D):
/// the impulses of the last step, matched by edge ids for warm starting.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SoftEdgeContact {
    /// The other edge's collision mesh (this mesh for a self contact).
    pub other: SoftMeshRef,
    /// Edge id in this body (element id in 2D), and in the other body.
    pub edge: u32,
    pub other_edge: u32,
    /// Normal impulse and world-space friction impulse.
    pub impulse: Real,
    pub tangent_impulse: Vector,
}

/// A vertex-vs-surface contact between a surface vertex of another soft body (or of this body:
/// self contacts) and one of this body's surface elements: the impulses of the last step, matched
/// by ids for warm starting.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SoftVertexContact {
    /// The vertex's collision mesh (this mesh for a self contact).
    pub other: SoftMeshRef,
    /// The vertex (particle index in `other`) and the element (in this body). `flipped`: the
    /// vertex is this body's and the element the other's (the other body is fully pinned and
    /// not simulated, this body assembles both vertex passes).
    pub vertex: u32,
    pub element: u32,
    pub flipped: bool,
    /// Normal impulse and world-space friction impulse.
    pub impulse: Real,
    pub tangent_impulse: Vector,
}


#[cfg(all(test, feature = "dim2"))]
mod test {
    use crate::alloc_prelude::*;
    use crate::dynamics::soft_body::SoftVertexContact;
    use crate::prelude::*;

    /// The closest point of segment `ab` to `p`, derived independently of the code under test.
    fn clamped_projection(p: Vector, a: Vector, b: Vector) -> Vector {
        let ab = b - a;
        let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
        a + ab * t
    }

    /// A debug-render contact segment must end at the contact's witness point on the element (its
    /// closest point to the vertex), not at the element's centroid: on a coarse mesh the two are
    /// far apart, and the contact is drawn where nothing touches.
    #[test]
    fn vertex_contact_segment_ends_at_the_witness_point() {
        let mut world = PhysicsWorld::new();
        let mut positions = Vec::new();
        for i in 0..4u32 {
            for j in 0..4u32 {
                positions.push(Vector::new(i as Real * 1.7, 10.0 + j as Real * 1.3));
            }
        }
        let mut cells = Vec::new();
        for i in 0..3u32 {
            for j in 0..3u32 {
                let (a, b, c, d) = (i * 4 + j, i * 4 + j + 1, (i + 1) * 4 + j, (i + 1) * 4 + j + 1);
                cells.push([a, b, c]);
                cells.push([b, d, c]);
            }
        }
        let handle = world.insert_soft_body(SoftBodyBuilder::new(positions).cells(cells));
        world.step();

        // The (vertex, element) pair whose witness point is furthest from the element's centroid:
        // the configuration the two candidate renderings disagree on the most.
        let sb = &world.soft_bodies[handle];
        let mesh = sb.meshes().next().expect("the body has a collision mesh");
        let id = mesh.id();
        let mut best = (0u32, 0u32, -1.0 as Real);
        for v in 0..mesh.vertex_count() as u32 {
            for (e, element) in mesh.indices().iter().enumerate() {
                if element.contains(&v) {
                    continue;
                }
                let p = mesh.vertex(sb, v as usize);
                let (a, b) = (
                    mesh.vertex(sb, element[0] as usize),
                    mesh.vertex(sb, element[1] as usize),
                );
                let spread = (clamped_projection(p, a, b) - (a + b) * 0.5).length();
                if spread > best.2 {
                    best = (v, e as u32, spread);
                }
            }
        }
        let (vertex, element, spread) = best;
        assert!(spread > 0.5, "no off-center pair in this mesh");

        // Inject that contact, then read back what the debug-renderer would draw for it.
        let sb = world.soft_bodies.get_mut(handle).unwrap();
        let mesh = sb.mesh_mut(id).unwrap();
        mesh.vertex_contacts.clear();
        mesh.vertex_contacts.push(SoftVertexContact {
            other: super::SoftMeshRef { body: handle, id },
            vertex,
            element,
            flipped: false,
            impulse: 0.0,
            tangent_impulse: Vector::ZERO,
        });

        let sb = &world.soft_bodies[handle];
        let mesh = sb.meshes().next().unwrap();
        let segments: Vec<_> = sb.vertex_contact_segments(&world.soft_bodies).collect();
        assert_eq!(segments.len(), 1);

        let idx = mesh.indices()[element as usize];
        let p = mesh.vertex(sb, vertex as usize);
        let (a, b) = (
            mesh.vertex(sb, idx[0] as usize),
            mesh.vertex(sb, idx[1] as usize),
        );
        let (drawn_from, drawn_to) = segments[0];
        assert!((drawn_from - p).length() < 1.0e-5);
        assert!((drawn_to - clamped_projection(p, a, b)).length() < 1.0e-5);
        // What the centroid rendering would have drawn instead.
        assert!((drawn_to - (a + b) * 0.5).length() > 0.5);
    }
}
