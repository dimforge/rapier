use crate::approx::AbsDiffEq;
use crate::geometry::{self, Contact, ContactManifold, CuboidFeatureFace, Triangle};
use crate::math::{Isometry, Point, Vector};
use crate::utils::WBasis;
use na::Point2;
use ncollide::shape::Segment;

#[derive(Debug)]
pub struct PolyhedronFace {
    pub vertices: [Point<f32>; 4],
    pub vids: [u8; 4], // Feature ID of the vertices.
    pub eids: [u8; 4], // Feature ID of the edges.
    pub fid: u8,       // Feature ID of the face.
    pub num_vertices: usize,
}

impl From<CuboidFeatureFace> for PolyhedronFace {
    fn from(face: CuboidFeatureFace) -> Self {
        Self {
            vertices: face.vertices,
            vids: face.vids,
            eids: face.eids,
            fid: face.fid,
            num_vertices: 4,
        }
    }
}

impl From<Triangle> for PolyhedronFace {
    fn from(tri: Triangle) -> Self {
        Self {
            vertices: [tri.a, tri.b, tri.c, tri.c],
            vids: [0, 2, 4, 4],
            eids: [1, 3, 5, 5],
            fid: 0,
            num_vertices: 3,
        }
    }
}

impl From<Segment<f32>> for PolyhedronFace {
    fn from(seg: Segment<f32>) -> Self {
        // Vertices have feature ids 0 and 2.
        // The segment interior has feature id 1.
        Self {
            vertices: [seg.a, seg.b, seg.b, seg.b],
            vids: [0, 2, 2, 2],
            eids: [1, 1, 1, 1],
            fid: 0,
            num_vertices: 2,
        }
    }
}

impl PolyhedronFace {
    pub fn new() -> Self {
        Self {
            vertices: [Point::origin(); 4],
            vids: [0; 4],
            eids: [0; 4],
            fid: 0,
            num_vertices: 0,
        }
    }

    pub fn transform_by(&mut self, iso: &Isometry<f32>) {
        for p in &mut self.vertices[0..self.num_vertices] {
            *p = iso * *p;
        }
    }

    pub fn contacts(
        prediction_distance: f32,
        face1: &PolyhedronFace,
        sep_axis1: &Vector<f32>,
        face2: &PolyhedronFace,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        match (face1.num_vertices, face2.num_vertices) {
            (2, 2) => Self::contacts_edge_edge(
                prediction_distance,
                face1,
                sep_axis1,
                face2,
                pos21,
                manifold,
            ),
            _ => Self::contacts_face_face(
                prediction_distance,
                face1,
                sep_axis1,
                face2,
                pos21,
                manifold,
            ),
        }
    }

    fn contacts_edge_edge(
        prediction_distance: f32,
        face1: &PolyhedronFace,
        sep_axis1: &Vector<f32>,
        face2: &PolyhedronFace,
        pos21: &Isometry<f32>,
        manifold: &mut ContactManifold,
    ) {
        // Project the faces to a 2D plane for contact clipping.
        // The plane they are projected onto has normal sep_axis1
        // and contains the origin (this is numerically OK because
        // we are not working in world-space here).
        let basis = sep_axis1.orthonormal_basis();
        let projected_edge1 = [
            Point2::new(
                face1.vertices[0].coords.dot(&basis[0]),
                face1.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                face1.vertices[1].coords.dot(&basis[0]),
                face1.vertices[1].coords.dot(&basis[1]),
            ),
        ];
        let projected_edge2 = [
            Point2::new(
                face2.vertices[0].coords.dot(&basis[0]),
                face2.vertices[0].coords.dot(&basis[1]),
            ),
            Point2::new(
                face2.vertices[1].coords.dot(&basis[0]),
                face2.vertices[1].coords.dot(&basis[1]),
            ),
        ];

        let tangent1 =
            (projected_edge1[1] - projected_edge1[0]).try_normalize(f32::default_epsilon());
        let tangent2 =
            (projected_edge2[1] - projected_edge2[0]).try_normalize(f32::default_epsilon());

        // TODO: not sure what the best value for eps is.
        // Empirically, it appears that an epsilon smaller than 1.0e-3 is too small.
        if let (Some(tangent1), Some(tangent2)) = (tangent1, tangent2) {
            let parallel = tangent1.dot(&tangent2) >= crate::utils::COS_FRAC_PI_8;

            if !parallel {
                let seg1 = (&projected_edge1[0], &projected_edge1[1]);
                let seg2 = (&projected_edge2[0], &projected_edge2[1]);
                let (loc1, loc2) =
                    ncollide::query::closest_points_segment_segment_with_locations_nD(seg1, seg2);

                // Found a contact between the two edges.
                let bcoords1 = loc1.barycentric_coordinates();
                let bcoords2 = loc2.barycentric_coordinates();

                let edge1 = (face1.vertices[0], face1.vertices[1]);
                let edge2 = (face2.vertices[0], face2.vertices[1]);
                let local_p1 = edge1.0 * bcoords1[0] + edge1.1.coords * bcoords1[1];
                let local_p2 = edge2.0 * bcoords2[0] + edge2.1.coords * bcoords2[1];
                let dist = (local_p2 - local_p1).dot(&sep_axis1);

                if dist <= prediction_distance {
                    manifold.points.push(Contact {
                        local_p1,
                        local_p2: pos21 * local_p2,
                        impulse: 0.0,
                        tangent_impulse: Contact::zero_tangent_impulse(),
                        fid1: face1.eids[0],
                        fid2: face2.eids[0],
                        dist,
                    });
                }

                return;
            }
        }

        // The lines are parallel so we are having a conformal contact.
        // Let's use a range-based clipping to extract two contact points.
        // TODO: would it be better and/or more efficient to do the
        //clipping in 2D?
        if let Some(clips) = geometry::clip_segments(
            (face1.vertices[0], face1.vertices[1]),
            (face2.vertices[0], face2.vertices[1]),
        ) {
            manifold.points.push(Contact {
                local_p1: (clips.0).0,
                local_p2: pos21 * (clips.0).1,
                impulse: 0.0,
                tangent_impulse: Contact::zero_tangent_impulse(),
                fid1: 0, // FIXME
                fid2: 0, // FIXME
                dist: ((clips.0).1 - (clips.0).0).dot(&sep_axis1),
            });

            manifold.points.push(Contact {
                local_p1: (clips.1).0,
                local_p2: pos21 * (clips.1).1,
                impulse: 0.0,
                tangent_impulse: Contact::zero_tangent_impulse(),
                fid1: 0, // FIXME
                fid2: 0, // FIXME
                dist: ((clips.1).1 - (clips.1).0).dot(&sep_axis1),
            });
        }
    }

    fn contacts_face_face(
        prediction_distance: f32,
        face1: &PolyhedronFace,
        sep_axis1: &Vector<f32>,
        face2: &PolyhedronFace,
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
        if face2.num_vertices > 2 {
            let normal2 = (face2.vertices[2] - face2.vertices[1])
                .cross(&(face2.vertices[0] - face2.vertices[1]));
            let denom = normal2.dot(&sep_axis1);

            if !relative_eq!(denom, 0.0) {
                let last_index2 = face2.num_vertices as usize - 1;
                'point_loop1: for i in 0..face1.num_vertices as usize {
                    let p1 = projected_face1[i];

                    let sign = (projected_face2[0] - projected_face2[last_index2])
                        .perp(&(p1 - projected_face2[last_index2]));
                    for j in 0..last_index2 {
                        let new_sign = (projected_face2[j + 1] - projected_face2[j])
                            .perp(&(p1 - projected_face2[j]));
                        if new_sign * sign < 0.0 {
                            // The point lies outside.
                            continue 'point_loop1;
                        }
                    }

                    // All the perp had the same sign: the point is inside of the other shapes projection.
                    // Output the contact.
                    let dist = (face2.vertices[0] - face1.vertices[i]).dot(&normal2) / denom;
                    let local_p1 = face1.vertices[i];
                    let local_p2 = face1.vertices[i] + dist * sep_axis1;

                    if dist <= prediction_distance {
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
            }
        }

        if face1.num_vertices > 2 {
            let normal1 = (face1.vertices[2] - face1.vertices[1])
                .cross(&(face1.vertices[0] - face1.vertices[1]));

            let denom = -normal1.dot(&sep_axis1);
            if !relative_eq!(denom, 0.0) {
                let last_index1 = face1.num_vertices as usize - 1;
                'point_loop2: for i in 0..face2.num_vertices as usize {
                    let p2 = projected_face2[i];

                    let sign = (projected_face1[0] - projected_face1[last_index1])
                        .perp(&(p2 - projected_face1[last_index1]));
                    for j in 0..last_index1 {
                        let new_sign = (projected_face1[j + 1] - projected_face1[j])
                            .perp(&(p2 - projected_face1[j]));

                        if new_sign * sign < 0.0 {
                            // The point lies outside.
                            continue 'point_loop2;
                        }
                    }

                    // All the perp had the same sign: the point is inside of the other shapes projection.
                    // Output the contact.
                    let dist = (face1.vertices[0] - face2.vertices[i]).dot(&normal1) / denom;
                    let local_p2 = face2.vertices[i];
                    let local_p1 = face2.vertices[i] - dist * sep_axis1;

                    if true {
                        // dist <= prediction_distance {
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
        }

        // Now we have to compute the intersection between all pairs of
        // edges from the face 1 and from the face2.
        for j in 0..face2.num_vertices {
            let projected_edge2 = [
                projected_face2[j],
                projected_face2[(j + 1) % face2.num_vertices],
            ];

            for i in 0..face1.num_vertices {
                let projected_edge1 = [
                    projected_face1[i],
                    projected_face1[(i + 1) % face1.num_vertices],
                ];
                if let Some(bcoords) = closest_points_line2d(projected_edge1, projected_edge2) {
                    if bcoords.0 > 0.0 && bcoords.0 < 1.0 && bcoords.1 > 0.0 && bcoords.1 < 1.0 {
                        // Found a contact between the two edges.
                        let edge1 = (
                            face1.vertices[i],
                            face1.vertices[(i + 1) % face1.num_vertices],
                        );
                        let edge2 = (
                            face2.vertices[j],
                            face2.vertices[(j + 1) % face2.num_vertices],
                        );
                        let local_p1 = edge1.0 * (1.0 - bcoords.0) + edge1.1.coords * bcoords.0;
                        let local_p2 = edge2.0 * (1.0 - bcoords.1) + edge2.1.coords * bcoords.1;
                        let dist = (local_p2 - local_p1).dot(&sep_axis1);

                        if dist <= prediction_distance {
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

/// Compute the barycentric coordinates of the intersection between the two given lines.
/// Returns `None` if the lines are parallel.
fn closest_points_line2d(edge1: [Point2<f32>; 2], edge2: [Point2<f32>; 2]) -> Option<(f32, f32)> {
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
