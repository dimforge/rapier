use crate::geometry::{Contact, ContactManifold};
use crate::math::{Isometry, Point, Vector};
use crate::utils::WBasis;
use na::Point2;

#[derive(Debug)]
#[allow(dead_code)]
pub(crate) enum CuboidFeature {
    Face(CuboidFeatureFace),
    Edge(CuboidFeatureEdge),
    Vertex(CuboidFeatureVertex),
}

#[derive(Debug)]
pub(crate) struct CuboidFeatureVertex {
    pub vertex: Point<f32>,
    pub vid: u8,
}

impl CuboidFeatureVertex {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        self.vertex = iso * self.vertex;
    }
}

#[derive(Debug)]
pub(crate) struct CuboidFeatureEdge {
    pub vertices: [Point<f32>; 2],
    pub vids: [u8; 2],
    pub eid: u8,
}

impl CuboidFeatureEdge {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        self.vertices[0] = iso * self.vertices[0];
        self.vertices[1] = iso * self.vertices[1];
    }
}

#[derive(Debug)]
pub(crate) struct CuboidFeatureFace {
    pub vertices: [Point<f32>; 4],
    pub vids: [u8; 4], // Feature ID of the vertices.
    pub eids: [u8; 4], // Feature ID of the edges.
    pub fid: u8,       // Feature ID of the face.
}

impl CuboidFeatureFace {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        self.vertices[0] = iso * self.vertices[0];
        self.vertices[1] = iso * self.vertices[1];
        self.vertices[2] = iso * self.vertices[2];
        self.vertices[3] = iso * self.vertices[3];
    }
}

impl CuboidFeature {
    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        match self {
            CuboidFeature::Face(face) => face.transform_by(iso),
            CuboidFeature::Edge(edge) => edge.transform_by(iso),
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
        let normal1 =
            (face1.vertices[0] - face1.vertices[1]).cross(&(face1.vertices[2] - face1.vertices[1]));
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

    /// Compute contacts points between a face and an edge.
    ///
    /// This method assume we already know that at least one contact exists.
    pub fn face_edge_contacts(
        prediction_distance: f32,
        face1: &CuboidFeatureFace,
        sep_axis1: &Vector<f32>,
        edge2: &CuboidFeatureEdge,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
        flipped: bool,
    ) {
        // Project the faces to a 2D plane for contact clipping.
        // The plane they are projected onto has normal sep_axis1
        // and contains the origin (this is numerically OK because
        // we are not working in world-space here).
        let basis = sep_axis1.orthonormal_basis();
        let projected_face1 = [
            Point2::new(
                face1.vertices[0].coords.dot(&basis[0]),
                face1.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[1].coords.dot(&basis[0]),
                face1.vertices[1].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[2].coords.dot(&basis[0]),
                face1.vertices[2].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[3].coords.dot(&basis[0]),
                face1.vertices[3].coords.dot(&basis[1]),
            ),
        ];
        let projected_edge2 = [
            Point2::new(
                edge2.vertices[0].coords.dot(&basis[0]),
                edge2.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                edge2.vertices[1].coords.dot(&basis[0]),
                edge2.vertices[1].coords.dot(&basis[1]),
            ),
        ];

        // Now we have to compute the intersection between all pairs of
        // edges from the face 1 with the edge 2
        for i in 0..4 {
            let projected_edge1 = [projected_face1[i], projected_face1[(i + 1) % 4]];

            if let Some(bcoords) = closest_points_line2d(projected_edge1, projected_edge2) {
                if bcoords.0 > 0.0 && bcoords.0 < 1.0 && bcoords.1 > 0.0 && bcoords.1 < 1.0 {
                    // Found a contact between the two edges.
                    let edge1 = [face1.vertices[i], face1.vertices[(i + 1) % 4]];
                    let local_p1 = edge1[0] * (1.0 - bcoords.0) + edge1[1].coords * bcoords.0;
                    let local_p2 = edge2.vertices[0] * (1.0 - bcoords.1)
                        + edge2.vertices[1].coords * bcoords.1;
                    let dist = (local_p2 - local_p1).dot(&sep_axis1);

                    if dist < prediction_distance {
                        if flipped {
                            manifold.points.push(Contact {
                                local_p1: local_p2,
                                // All points are expressed in the locale space of the first shape
                                // (even if there was a flip). So the point we need to transform by
                                // pos21 is the one that will go into local_p2.
                                local_p2: pos21 * local_p1,
                                impulse: 0.0,
                                tangent_impulse: Contact::zero_tangent_impulse(),
                                fid1: edge2.eid,
                                fid2: face1.eids[i],
                                dist,
                            });
                        } else {
                            manifold.points.push(Contact {
                                local_p1,
                                local_p2: pos21 * local_p2,
                                impulse: 0.0,
                                tangent_impulse: Contact::zero_tangent_impulse(),
                                fid1: face1.eids[i],
                                fid2: edge2.eid,
                                dist,
                            });
                        }
                    }
                }
            }
        }

        // Project the two points from the edge into the face.
        let normal1 =
            (face1.vertices[2] - face1.vertices[1]).cross(&(face1.vertices[0] - face1.vertices[1]));

        'point_loop2: for i in 0..2 {
            let p2 = projected_edge2[i];

            let sign = (projected_face1[0] - projected_face1[3]).perp(&(p2 - projected_face1[3]));
            for j in 0..3 {
                let new_sign =
                    (projected_face1[j + 1] - projected_face1[j]).perp(&(p2 - projected_face1[j]));
                if new_sign * sign < 0.0 {
                    // The point lies outside.
                    continue 'point_loop2;
                }
            }

            // All the perp had the same sign: the point is inside of the other shapes projection.
            // Output the contact.
            let denom = -normal1.dot(&sep_axis1);
            let dist = (face1.vertices[0] - edge2.vertices[i]).dot(&normal1) / denom;
            let local_p2 = edge2.vertices[i];
            let local_p1 = edge2.vertices[i] - dist * sep_axis1;

            if dist < prediction_distance {
                if flipped {
                    manifold.points.push(Contact {
                        local_p1: local_p2,
                        // All points are expressed in the locale space of the first shape
                        // (even if there was a flip). So the point we need to transform by
                        // pos21 is the one that will go into local_p2.
                        local_p2: pos21 * local_p1,
                        impulse: 0.0,
                        tangent_impulse: Contact::zero_tangent_impulse(),
                        fid1: edge2.vids[i],
                        fid2: face1.fid,
                        dist,
                    });
                } else {
                    manifold.points.push(Contact {
                        local_p1,
                        local_p2: pos21 * local_p2,
                        impulse: 0.0,
                        tangent_impulse: Contact::zero_tangent_impulse(),
                        fid1: face1.fid,
                        fid2: edge2.vids[i],
                        dist,
                    });
                }
            }
        }
    }

    /// Compute contacts points between two edges.
    ///
    /// This method assume we already know that at least one contact exists.
    pub fn edge_edge_contacts(
        edge1: &CuboidFeatureEdge,
        sep_axis1: &Vector<f32>,
        edge2: &CuboidFeatureEdge,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        let basis = sep_axis1.orthonormal_basis();
        let projected_edge1 = [
            Point2::new(
                edge1.vertices[0].coords.dot(&basis[0]),
                edge1.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                edge1.vertices[1].coords.dot(&basis[0]),
                edge1.vertices[1].coords.dot(&basis[1]),
            ),
        ];
        let projected_edge2 = [
            Point2::new(
                edge2.vertices[0].coords.dot(&basis[0]),
                edge2.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                edge2.vertices[1].coords.dot(&basis[0]),
                edge2.vertices[1].coords.dot(&basis[1]),
            ),
        ];

        if let Some(bcoords) = closest_points_line2d(projected_edge1, projected_edge2) {
            let local_p1 =
                edge1.vertices[0] * (1.0 - bcoords.0) + edge1.vertices[1].coords * bcoords.0;
            let local_p2 =
                edge2.vertices[0] * (1.0 - bcoords.1) + edge2.vertices[1].coords * bcoords.1;
            let dist = (local_p2 - local_p1).dot(&sep_axis1);

            manifold.points.push(Contact {
                local_p1,
                local_p2: pos21 * local_p2, // NOTE: local_p2 is expressed in the local space of cube1.
                impulse: 0.0,
                tangent_impulse: Contact::zero_tangent_impulse(),
                fid1: edge1.eid,
                fid2: edge2.eid,
                dist,
            });
        }
    }

    pub fn face_face_contacts(
        _prediction_distance: f32,
        face1: &CuboidFeatureFace,
        sep_axis1: &Vector<f32>,
        face2: &CuboidFeatureFace,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        // Project the faces to a 2D plane for contact clipping.
        // The plane they are projected onto has normal sep_axis1
        // and contains the origin (this is numerically OK because
        // we are not working in world-space here).
        let basis = sep_axis1.orthonormal_basis();
        let projected_face1 = [
            Point2::new(
                face1.vertices[0].coords.dot(&basis[0]),
                face1.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[1].coords.dot(&basis[0]),
                face1.vertices[1].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[2].coords.dot(&basis[0]),
                face1.vertices[2].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[3].coords.dot(&basis[0]),
                face1.vertices[3].coords.dot(&basis[1]),
            ),
        ];
        let projected_face2 = [
            Point2::new(
                face2.vertices[0].coords.dot(&basis[0]),
                face2.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                face2.vertices[1].coords.dot(&basis[0]),
                face2.vertices[1].coords.dot(&basis[1]),
            ),
            Point2::new(
                face2.vertices[2].coords.dot(&basis[0]),
                face2.vertices[2].coords.dot(&basis[1]),
            ),
            Point2::new(
                face2.vertices[3].coords.dot(&basis[0]),
                face2.vertices[3].coords.dot(&basis[1]),
            ),
        ];

        // Also find all the vertices located inside of the other projected face.
        let normal1 =
            (face1.vertices[2] - face1.vertices[1]).cross(&(face1.vertices[0] - face1.vertices[1]));
        let normal2 =
            (face2.vertices[2] - face2.vertices[1]).cross(&(face2.vertices[0] - face2.vertices[1]));

        // NOTE: The loop iterating on all the vertices for face1 is different from
        // the one iterating on all the vertices of face2. In the second loop, we
        // classify every point wrt. every edge on the other face. This will give
        // us bit masks to filter out several edge-edge intersections.
        'point_loop1: for i in 0..4 {
            let p1 = projected_face1[i];

            let sign = (projected_face2[0] - projected_face2[3]).perp(&(p1 - projected_face2[3]));
            for j in 0..3 {
                let new_sign =
                    (projected_face2[j + 1] - projected_face2[j]).perp(&(p1 - projected_face2[j]));
                if new_sign * sign < 0.0 {
                    // The point lies outside.
                    continue 'point_loop1;
                }
            }

            // All the perp had the same sign: the point is inside of the other shapes projection.
            // Output the contact.
            let denom = normal2.dot(&sep_axis1);
            let dist = (face2.vertices[0] - face1.vertices[i]).dot(&normal2) / denom;
            let local_p1 = face1.vertices[i];
            let local_p2 = face1.vertices[i] + dist * sep_axis1;

            if true {
                // dist <= prediction_distance {
                manifold.points.push(Contact {
                    local_p1,
                    local_p2: pos21 * local_p2,
                    impulse: 0.0,
                    tangent_impulse: Contact::zero_tangent_impulse(),
                    fid1: face1.vids[i],
                    fid2: face2.fid,
                    dist,
                });
            }
        }

        let is_clockwise1 = (projected_face1[0] - projected_face1[1])
            .perp(&(projected_face1[2] - projected_face1[1]))
            >= 0.0;
        let mut vertex_class2 = [0u8; 4];

        for i in 0..4 {
            let p2 = projected_face2[i];

            let sign = (projected_face1[0] - projected_face1[3]).perp(&(p2 - projected_face1[3]));
            vertex_class2[i] |= ((sign >= 0.0) as u8) << 3;

            for j in 0..3 {
                let sign =
                    (projected_face1[j + 1] - projected_face1[j]).perp(&(p2 - projected_face1[j]));
                vertex_class2[i] |= ((sign >= 0.0) as u8) << j;
            }

            if !is_clockwise1 {
                vertex_class2[i] = (!vertex_class2[i]) & 0b01111;
            }

            if vertex_class2[i] == 0 {
                // All the perp had the same sign: the point is inside of the other shapes projection.
                // Output the contact.
                let denom = -normal1.dot(&sep_axis1);
                let dist = (face1.vertices[0] - face2.vertices[i]).dot(&normal1) / denom;
                let local_p2 = face2.vertices[i];
                let local_p1 = face2.vertices[i] - dist * sep_axis1;

                if true {
                    // dist < prediction_distance {
                    manifold.points.push(Contact {
                        local_p1,
                        local_p2: pos21 * local_p2,
                        impulse: 0.0,
                        tangent_impulse: Contact::zero_tangent_impulse(),
                        fid1: face1.fid,
                        fid2: face2.vids[i],
                        dist,
                    });
                }
            }
        }

        // Now we have to compute the intersection between all pairs of
        // edges from the face 1 and from the face2.
        for j in 0..4 {
            let projected_edge2 = [projected_face2[j], projected_face2[(j + 1) % 4]];

            if (vertex_class2[j] & vertex_class2[(j + 1) % 4]) != 0 {
                continue;
            }

            let edge_class2 = vertex_class2[j] | vertex_class2[(j + 1) % 4];

            for i in 0..4 {
                if (edge_class2 & (1 << i)) != 0 {
                    let projected_edge1 = [projected_face1[i], projected_face1[(i + 1) % 4]];
                    if let Some(bcoords) = closest_points_line2d(projected_edge1, projected_edge2) {
                        if bcoords.0 > 0.0 && bcoords.0 < 1.0 && bcoords.1 > 0.0 && bcoords.1 < 1.0
                        {
                            // Found a contact between the two edges.
                            let edge1 = (face1.vertices[i], face1.vertices[(i + 1) % 4]);
                            let edge2 = (face2.vertices[j], face2.vertices[(j + 1) % 4]);
                            let local_p1 = edge1.0 * (1.0 - bcoords.0) + edge1.1.coords * bcoords.0;
                            let local_p2 = edge2.0 * (1.0 - bcoords.1) + edge2.1.coords * bcoords.1;
                            let dist = (local_p2 - local_p1).dot(&sep_axis1);

                            if true {
                                // dist <= prediction_distance {
                                manifold.points.push(Contact {
                                    local_p1,
                                    local_p2: pos21 * local_p2,
                                    impulse: 0.0,
                                    tangent_impulse: Contact::zero_tangent_impulse(),
                                    fid1: face1.eids[i],
                                    fid2: face2.eids[j],
                                    dist,
                                });
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Compute the barycentric coordinates of the intersection between the two given lines.
/// Returns `None` if the lines are parallel.
fn closest_points_line2d(edge1: [Point2<f32>; 2], edge2: [Point2<f32>; 2]) -> Option<(f32, f32)> {
    use approx::AbsDiffEq;

    // Inspired by Real-time collision detection by Christer Ericson.
    let dir1 = edge1[1] - edge1[0];
    let dir2 = edge2[1] - edge2[0];
    let r = edge1[0] - edge2[0];

    let a = dir1.norm_squared();
    let e = dir2.norm_squared();
    let f = dir2.dot(&r);

    let eps = f32::default_epsilon();

    if a <= eps && e <= eps {
        Some((0.0, 0.0))
    } else if a <= eps {
        Some((0.0, f / e))
    } else {
        let c = dir1.dot(&r);
        if e <= eps {
            Some((-c / a, 0.0))
        } else {
            let b = dir1.dot(&dir2);
            let ae = a * e;
            let bb = b * b;
            let denom = ae - bb;

            // Use absolute and ulps error to test collinearity.
            let parallel = denom <= eps || approx::ulps_eq!(ae, bb);

            let s = if !parallel {
                (b * f - c * e) / denom
            } else {
                0.0
            };

            if parallel {
                None
            } else {
                Some((s, (b * s + f) / e))
            }
        }
    }
}
