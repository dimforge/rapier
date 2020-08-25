use na::{Point2, Real};

use shape::SegmentPointLocation;
use utils::{self, SegmentsIntersection, TriangleOrientation};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum InFlag {
    PIn,
    QIn,
    Unknown,
}

/// Location of a point on a polyline.
pub enum PolylinePointLocation<N> {
    /// Point on a vertex.
    OnVertex(usize),
    /// Point on an edge.
    OnEdge(usize, usize, [N; 2]),
}

impl<N: Real> PolylinePointLocation<N> {
    /// Computes the point corresponding to this location.
    pub fn to_point(&self, pts: &[Point2<N>]) -> Point2<N> {
        match self {
            PolylinePointLocation::OnVertex(i) => pts[*i],
            PolylinePointLocation::OnEdge(i1, i2, bcoords) => {
                pts[*i1] * bcoords[0] + pts[*i2].coords * bcoords[1]
            }
        }
    }

    fn from_segment_point_location(a: usize, b: usize, loc: SegmentPointLocation<N>) -> Self {
        match loc {
            SegmentPointLocation::OnVertex(0) => PolylinePointLocation::OnVertex(a),
            SegmentPointLocation::OnVertex(1) => PolylinePointLocation::OnVertex(b),
            SegmentPointLocation::OnVertex(_) => unreachable!(),
            SegmentPointLocation::OnEdge(bcoords) => PolylinePointLocation::OnEdge(a, b, bcoords),
        }
    }
}

/// Computes the intersection points of two convex polygons.
///
/// The resulting polygon is output vertex-by-vertex to the `out` closure.
pub fn convex_polygons_intersection_points<N: Real>(
    poly1: &[Point2<N>],
    poly2: &[Point2<N>],
    out: &mut Vec<Point2<N>>,
) {
    convex_polygons_intersection(poly1, poly2, |loc1, loc2| {
        if let Some(loc1) = loc1 {
            out.push(loc1.to_point(poly1))
        } else if let Some(loc2) = loc2 {
            out.push(loc2.to_point(poly2))
        }
    })
}

/// Computes the intersection of two convex polygons.
///
/// The resulting polygon is output vertex-by-vertex to the `out` closure.
pub fn convex_polygons_intersection<N: Real>(
    poly1: &[Point2<N>],
    poly2: &[Point2<N>],
    mut out: impl FnMut(Option<PolylinePointLocation<N>>, Option<PolylinePointLocation<N>>),
) {
    // FIXME: this does not handle correctly the case where the
    // first triangle of the polygon is degenerate.
    let rev1 = poly1.len() > 2
        && utils::triangle_orientation(&poly1[0], &poly1[1], &poly1[2])
            == TriangleOrientation::Clockwise;
    let rev2 = poly2.len() > 2
        && utils::triangle_orientation(&poly2[0], &poly2[1], &poly2[2])
            == TriangleOrientation::Clockwise;

    // println!("rev1: {}, rev2: {}", rev1, rev2);

    let n = poly1.len();
    let m = poly2.len();

    let mut a = 0;
    let mut b = 0;
    let mut aa = 0;
    let mut ba = 0;
    let mut inflag = InFlag::Unknown;
    let mut first_point_found = false;

    // Quit when both adv. indices have cycled, or one has cycled twice.
    while (aa < n || ba < m) && aa < 2 * n && ba < 2 * m {
        let (a1, a2) = if rev1 {
            ((n - a) % n, n - a - 1)
        } else {
            ((a + n - 1) % n, a)
        };

        let (b1, b2) = if rev2 {
            ((m - b) % m, m - b - 1)
        } else {
            ((b + m - 1) % m, b)
        };

        // println!("Current indices: ({}, {}), ({}, {})", a1, a2, b1, b2);

        let dir_edge1 = poly1[a2] - poly1[a1];
        let dir_edge2 = poly2[b2] - poly2[b1];

        let cross = utils::triangle_orientation(
            &Point2::origin(),
            &Point2::from_coordinates(dir_edge1),
            &Point2::from_coordinates(dir_edge2),
        );
        let aHB = utils::triangle_orientation(&poly2[b1], &poly2[b2], &poly1[a2]);
        let bHA = utils::triangle_orientation(&poly1[a1], &poly1[a2], &poly2[b2]);

        // If edge1 & edge2 intersect, update inflag.
        if let Some(inter) =
            utils::segments_intersection(&poly1[a1], &poly1[a2], &poly2[b1], &poly2[b2])
        {
            match inter {
                SegmentsIntersection::Point { loc1, loc2 } => {
                    let loc1 = PolylinePointLocation::from_segment_point_location(a1, a2, loc1);
                    let loc2 = PolylinePointLocation::from_segment_point_location(b1, b2, loc2);
                    out(Some(loc1), Some(loc2));

                    if inflag == InFlag::Unknown && !first_point_found {
                        // This is the first point.
                        aa = 0;
                        ba = 0;
                        first_point_found = true;
                    }

                    // Update inflag.
                    if aHB == TriangleOrientation::Counterclockwise {
                        inflag = InFlag::PIn;
                    } else if bHA == TriangleOrientation::Counterclockwise {
                        inflag = InFlag::QIn;
                    }
                }
                SegmentsIntersection::Segment {
                    first_loc1,
                    first_loc2,
                    second_loc1,
                    second_loc2,
                } => {
                    // Special case: edge1 & edge2 overlap and oppositely oriented.
                    if dir_edge1.dot(&dir_edge2) < N::zero() {
                        let loc1 =
                            PolylinePointLocation::from_segment_point_location(a1, a2, first_loc1);
                        let loc2 =
                            PolylinePointLocation::from_segment_point_location(b1, b2, first_loc2);
                        out(Some(loc1), Some(loc2));

                        let loc1 =
                            PolylinePointLocation::from_segment_point_location(a1, a2, second_loc1);
                        let loc2 =
                            PolylinePointLocation::from_segment_point_location(b1, b2, second_loc2);
                        out(Some(loc1), Some(loc2));

                        return;
                    }
                }
            }
        }

        // Special case: edge1 & edge2 parallel and separated.
        if cross == TriangleOrientation::Degenerate
            && aHB == TriangleOrientation::Clockwise
            && bHA == TriangleOrientation::Clockwise
        {
            return;
        }
        // Special case: edge1 & edge2 collinear.
        else if cross == TriangleOrientation::Degenerate
            && aHB == TriangleOrientation::Degenerate
            && bHA == TriangleOrientation::Degenerate
        {
            // Advance but do not output point.
            if inflag == InFlag::PIn {
                b = advance(b, &mut ba, m);
            } else {
                a = advance(a, &mut aa, n);
            }
        }
        // Generic cases.
        else if cross == TriangleOrientation::Counterclockwise {
            if bHA == TriangleOrientation::Counterclockwise {
                if inflag == InFlag::PIn {
                    out(Some(PolylinePointLocation::OnVertex(a2)), None)
                }
                a = advance(a, &mut aa, n);
            } else {
                if inflag == InFlag::QIn {
                    out(None, Some(PolylinePointLocation::OnVertex(b2)))
                }
                b = advance(b, &mut ba, m);
            }
        } else {
            // We have cross == TriangleOrientation::Clockwise.
            if aHB == TriangleOrientation::Counterclockwise {
                if inflag == InFlag::QIn {
                    out(None, Some(PolylinePointLocation::OnVertex(b2)))
                }
                b = advance(b, &mut ba, m);
            } else {
                if inflag == InFlag::PIn {
                    out(Some(PolylinePointLocation::OnVertex(a2)), None)
                }
                a = advance(a, &mut aa, n);
            }
        }
    }

    if !first_point_found {
        // No intersection: test if one polygon completely encloses the other.
        let mut orient = TriangleOrientation::Degenerate;
        let mut ok = true;

        for a in 0..n {
            let a1 = (a + n - 1) % n; // a - 1
            let new_orient = utils::triangle_orientation(&poly1[a1], &poly1[a], &poly2[0]);

            if orient == TriangleOrientation::Degenerate {
                orient = new_orient
            } else if new_orient != orient && new_orient != TriangleOrientation::Degenerate {
                ok = false;
                break;
            }
        }

        if ok {
            for b in 0..m {
                out(None, Some(PolylinePointLocation::OnVertex(b)))
            }
        }

        let mut orient = TriangleOrientation::Degenerate;
        let mut ok = true;

        for b in 0..m {
            let b1 = (b + m - 1) % m; // b - 1
            let new_orient = utils::triangle_orientation(&poly2[b1], &poly2[b], &poly1[0]);

            if orient == TriangleOrientation::Degenerate {
                orient = new_orient
            } else if new_orient != orient && new_orient != TriangleOrientation::Degenerate {
                ok = false;
                break;
            }
        }

        if ok {
            for a in 0..n {
                out(Some(PolylinePointLocation::OnVertex(a)), None)
            }
        }
    }
}

#[inline]
fn advance(a: usize, aa: &mut usize, n: usize) -> usize {
    *aa += 1;
    (a + 1) % n
}
