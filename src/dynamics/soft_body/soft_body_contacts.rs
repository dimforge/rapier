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

/// The progress of one pair's overlap correction across steps: a pair whose volume estimate stops
/// improving stands its positional correction down after `SoftRecoverySettings::overlap_patience`
/// steps (a static compression) and re-arms when the estimate drops by `overlap_progress_margin`.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct SoftOverlapState {
    /// The other collider of the pair.
    pub other: crate::geometry::ColliderHandle,
    /// Lowest volume estimate reached so far.
    pub best_error: Real,
    /// Steps since the estimate last improved by the margin.
    pub steps_stuck: u16,
    /// Whether the correction has stood down (the patience ran out): re-arming then takes
    /// a clearly better estimate, not the noise of a pair squeezed against what holds it.
    pub stalled: bool,
}

/// The impulses a volume contact accumulated in the last step, per pair and per particle (see
/// `SoftOverlapConstraint`), plus the rigid side's: the hard constraints warm start from them
/// before the elastic constraints of the first substep, so resting springs see the contact force.
#[derive(Clone, Debug)]
pub(crate) struct SoftOverlapWarm {
    pub other: crate::geometry::ColliderHandle,
    /// The owner body's particles and their impulses.
    pub own: Vec<(u32, Vector)>,
    /// The other soft body's particles and their impulses.
    pub other_soft: Vec<(u32, Vector)>,
    /// The rigid side's linear and angular impulse.
    pub rigid: (Vector, AngVector),
}

/// One volume constraint assembled at the last step (a bin of a pair's volume contact, see
/// `SoftOverlapConstraint`), kept for debug rendering: center, normal (from the owner body into the
/// other side), signed volume estimate, and every entry's position and volume gradient.
#[derive(Clone, Debug)]
pub struct SoftVolumeContact {
    /// The area-weighted center of the constraint's patch.
    pub center: Vector,
    /// The contact normal, from the owner body into the other side.
    pub normal: Vector,
    /// The signed volume estimate (negative: speculative slack).
    pub volume: Real,
    /// The entries' positions and volume gradients.
    pub gradients: Vec<(Vector, Vector)>,
}

impl SoftBody {
    /// The world position of the `i`-th vertex of `mesh` (which must belong to `sb`), or `None`
    /// if the mesh does not have that vertex.
    fn mesh_vertex(sb: &SoftBody, mesh: &SoftCollisionMesh, i: u32) -> Option<Vector> {
        ((i as usize) < mesh.vertex_count()).then(|| mesh.vertex(sb, i as usize))
    }

    /// The `i`-th edge of `mesh` (which must belong to `sb`) as a world segment.
    fn mesh_edge(sb: &SoftBody, mesh: &SoftCollisionMesh, i: u32) -> Option<parry::shape::Segment> {
        #[cfg(feature = "dim2")]
        let vertices = *mesh.indices.get(i as usize)?;
        #[cfg(feature = "dim3")]
        let vertices = *mesh.edges.get(i as usize)?;
        Some(parry::shape::Segment::new(
            Self::mesh_vertex(sb, mesh, vertices[0])?,
            Self::mesh_vertex(sb, mesh, vertices[1])?,
        ))
    }

    /// The edge-vs-edge contacts this soft body's collision meshes owned at the last step (self
    /// contacts and contacts with other soft bodies' meshes), as world segments joining the two
    /// edges' closest points, i.e. the contact's witness points (for debug rendering).
    pub fn edge_contact_segments<'a>(
        &'a self,
        soft_bodies: &'a super::SoftBodySet,
    ) -> impl Iterator<Item = (Vector, Vector)> + 'a {
        self.meshes()
            .flat_map(|mesh| mesh.edge_contacts.iter().map(move |c| (mesh, c)))
            .filter_map(move |(mesh, c)| {
                // `edge` indexes the mesh owning the contact, `other_edge` the mesh it hit.
                let other = soft_bodies.get(c.other.body)?;
                let seg1 = Self::mesh_edge(self, mesh, c.edge)?;
                let seg2 = Self::mesh_edge(other, other.mesh(c.other.id)?, c.other_edge)?;
                let (loc1, loc2) =
                    parry::query::details::closest_points_segment_segment_with_locations(
                        &Pose::IDENTITY,
                        &seg1,
                        &seg2,
                    );
                Some((seg1.point_at(&loc1), seg2.point_at(&loc2)))
            })
    }

    /// The volume constraints this soft body's collision meshes assembled at the last step (see
    /// `SoftVolumeContact`; the pair's owner mesh keeps them), for debug rendering.
    pub fn volume_contacts(&self) -> impl Iterator<Item = &SoftVolumeContact> + '_ {
        self.meshes().flat_map(|mesh| mesh.volume_contacts.iter())
    }

    /// The vertex-vs-surface contacts against this soft body's collision meshes at the last step
    /// (self contacts and other soft bodies' vertices), as world segments joining the vertex to
    /// its projection on the element, i.e. the contact's witness points (for debug rendering).
    pub fn vertex_contact_segments<'a>(
        &'a self,
        soft_bodies: &'a super::SoftBodySet,
    ) -> impl Iterator<Item = (Vector, Vector)> + 'a {
        use parry::query::PointQuery;

        self.meshes()
            .flat_map(|mesh| mesh.vertex_contacts.iter().map(move |c| (mesh, c)))
            .filter_map(move |(mesh, c)| {
                let other = soft_bodies.get(c.other.body)?;
                let other_mesh = other.mesh(c.other.id)?;
                // `vertex` indexes the other mesh and `element` the mesh owning the contact,
                // unless the contact is flipped (see `SoftVertexContact`).
                let ((vertex_body, vertex_mesh), (element_body, element_mesh)) = if c.flipped {
                    ((self, mesh), (other, other_mesh))
                } else {
                    ((other, other_mesh), (self, mesh))
                };
                let point = Self::mesh_vertex(vertex_body, vertex_mesh, c.vertex)?;
                let element = *element_mesh.indices.get(c.element as usize)?;
                let mut vtx = [Vector::ZERO; DIM];
                for (dest, v) in vtx.iter_mut().zip(&element) {
                    *dest = Self::mesh_vertex(element_body, element_mesh, *v)?;
                }
                #[cfg(feature = "dim2")]
                let projection = parry::shape::Segment::new(vtx[0], vtx[1]);
                #[cfg(feature = "dim3")]
                let projection = parry::shape::Triangle::new(vtx[0], vtx[1], vtx[2]);
                Some((point, projection.project_local_point(point, false).point))
            })
    }
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
