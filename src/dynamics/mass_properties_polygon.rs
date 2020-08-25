use crate::dynamics::MassProperties;
use crate::math::Point;

impl MassProperties {
    pub(crate) fn from_polygon(density: f32, vertices: &[Point<f32>]) -> MassProperties {
        let (area, com) = convex_polygon_area_and_center_of_mass(vertices);

        if area == 0.0 {
            return MassProperties::new(com, 0.0, 0.0);
        }

        let mut itot = 0.0;
        let factor = 1.0 / 6.0;

        let mut iterpeek = vertices.iter().peekable();
        let firstelement = *iterpeek.peek().unwrap(); // store first element to close the cycle in the end with unwrap_or
        while let Some(elem) = iterpeek.next() {
            let area = triangle_area(&com, elem, iterpeek.peek().unwrap_or(&firstelement));

            // algorithm adapted from Box2D
            let e1 = *elem - com;
            let e2 = **(iterpeek.peek().unwrap_or(&firstelement)) - com;

            let ex1 = e1[0];
            let ey1 = e1[1];
            let ex2 = e2[0];
            let ey2 = e2[1];

            let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            let ipart = factor * (intx2 + inty2);

            itot += ipart * area;
        }

        Self::new(com, area * density, itot * density)
    }
}

fn convex_polygon_area_and_center_of_mass(convex_polygon: &[Point<f32>]) -> (f32, Point<f32>) {
    let geometric_center = convex_polygon
        .iter()
        .fold(Point::origin(), |e1, e2| e1 + e2.coords)
        / convex_polygon.len() as f32;
    let mut res = Point::origin();
    let mut areasum = 0.0;

    let mut iterpeek = convex_polygon.iter().peekable();
    let firstelement = *iterpeek.peek().unwrap(); // Stores first element to close the cycle in the end with unwrap_or.
    while let Some(elem) = iterpeek.next() {
        let (a, b, c) = (
            elem,
            iterpeek.peek().unwrap_or(&firstelement),
            &geometric_center,
        );
        let area = triangle_area(a, b, c);
        let center = (a.coords + b.coords + c.coords) / 3.0;

        res += center * area;
        areasum += area;
    }

    if areasum == 0.0 {
        (areasum, geometric_center)
    } else {
        (areasum, res / areasum)
    }
}

pub fn triangle_area(pa: &Point<f32>, pb: &Point<f32>, pc: &Point<f32>) -> f32 {
    // Kahan's formula.
    let a = na::distance(pa, pb);
    let b = na::distance(pb, pc);
    let c = na::distance(pc, pa);

    let (c, b, a) = sort3(&a, &b, &c);
    let a = *a;
    let b = *b;
    let c = *c;

    let sqr = (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c));

    sqr.sqrt() * 0.25
}

/// Sorts a set of three values in increasing order.
#[inline]
pub fn sort3<'a>(a: &'a f32, b: &'a f32, c: &'a f32) -> (&'a f32, &'a f32, &'a f32) {
    let a_b = *a > *b;
    let a_c = *a > *c;
    let b_c = *b > *c;

    let sa;
    let sb;
    let sc;

    // Sort the three values.
    // FIXME: move this to the utilities?
    if a_b {
        // a > b
        if a_c {
            // a > c
            sc = a;

            if b_c {
                // b > c
                sa = c;
                sb = b;
            } else {
                // b <= c
                sa = b;
                sb = c;
            }
        } else {
            // a <= c
            sa = b;
            sb = a;
            sc = c;
        }
    } else {
        // a < b
        if !a_c {
            // a <= c
            sa = a;

            if b_c {
                // b > c
                sb = c;
                sc = b;
            } else {
                sb = b;
                sc = c;
            }
        } else {
            // a > c
            sa = c;
            sb = a;
            sc = b;
        }
    }

    (sa, sb, sc)
}
