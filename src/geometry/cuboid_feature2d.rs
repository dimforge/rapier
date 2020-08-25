use crate::geometry::{self, Contact, ContactManifold};
use crate::math::{Isometry, Point, Vector};
use ncollide::shape::Segment;

#[derive(Debug)]
#[allow(dead_code)]
pub enum CuboidFeature {
    Face(CuboidFeatureFace),
    Vertex(CuboidFeatureVertex),
}

#[derive(Debug)]
pub struct CuboidFeatureVertex {
    pub vertex: Point<f32>,
    pub vid: u8,
}

impl CuboidFeatureVertex {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        self.vertex = iso * self.vertex;
    }
}

#[derive(Debug)]
pub struct CuboidFeatureFace {
    pub vertices: [Point<f32>; 2],
    pub vids: [u8; 2],
    pub fid: u8,
}

impl From<Segment<f32>> for CuboidFeatureFace {
    fn from(seg: Segment<f32>) -> Self {
        CuboidFeatureFace {
            vertices: [seg.a, seg.b],
            vids: [0, 2],
            fid: 1,
        }
    }
}

impl CuboidFeatureFace {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        self.vertices[0] = iso * self.vertices[0];
        self.vertices[1] = iso * self.vertices[1];
    }
}

impl CuboidFeature {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        match self {
            CuboidFeature::Face(face) => face.transform_by(iso),
            CuboidFeature::Vertex(vertex) => vertex.transform_by(iso),
        }
    }

    /// Compute contacts points between a face and a vertex.
    ///
    /// This method assume we already know that at least one contact exists.
    pub fn face_vertex_contacts(
        face1: &CuboidFeatureFace,
        sep_axis1: &Vector<f32>,
        vertex2: &CuboidFeatureVertex,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        let tangent1 = face1.vertices[1] - face1.vertices[0];
        let normal1 = Vector::new(-tangent1.y, tangent1.x);
        let denom = -normal1.dot(&sep_axis1);
        let dist = (face1.vertices[0] - vertex2.vertex).dot(&normal1) / denom;
        let local_p2 = vertex2.vertex;
        let local_p1 = vertex2.vertex - dist * sep_axis1;

        manifold.points.push(Contact {
            local_p1,
            local_p2: pos21 * local_p2,
            impulse: 0.0,
            tangent_impulse: Contact::zero_tangent_impulse(),
            fid1: face1.fid,
            fid2: vertex2.vid,
            dist,
        });
    }

    pub fn face_face_contacts(
        _prediction_distance: f32,
        face1: &CuboidFeatureFace,
        normal1: &Vector<f32>,
        face2: &CuboidFeatureFace,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        if let Some((clip_a, clip_b)) = geometry::clip_segments(
            (face1.vertices[0], face1.vertices[1]),
            (face2.vertices[0], face2.vertices[1]),
        ) {
            let fids1 = [face1.vids[0], face1.fid, face1.vids[1]];
            let fids2 = [face2.vids[0], face2.fid, face2.vids[1]];

            let dist = (clip_a.1 - clip_a.0).dot(normal1);
            if true {
                // dist < prediction_distance {
                manifold.points.push(Contact {
                    local_p1: clip_a.0,
                    local_p2: pos21 * clip_a.1,
                    impulse: 0.0,
                    tangent_impulse: Contact::zero_tangent_impulse(),
                    fid1: fids1[clip_a.2],
                    fid2: fids2[clip_a.3],
                    dist,
                });
            }

            let dist = (clip_b.1 - clip_b.0).dot(normal1);
            if true {
                // dist < prediction_distance {
                manifold.points.push(Contact {
                    local_p1: clip_b.0,
                    local_p2: pos21 * clip_b.1,
                    impulse: 0.0,
                    tangent_impulse: Contact::zero_tangent_impulse(),
                    fid1: fids1[clip_b.2],
                    fid2: fids2[clip_b.3],
                    dist,
                });
            }
        }
    }
}
