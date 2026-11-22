use crate::geometry::ContactManifold;
use crate::math::Real;

pub(crate) fn reduce_manifold_naive(
    manifold: &ContactManifold,
    selected: &mut [usize; 4],
    num_selected: &mut usize,
    is_solver_contact: impl Fn(usize) -> bool,
) {
    if manifold.points.len() <= 4 {
        return;
    }

    // 1. Find the deepest contact.
    *selected = [usize::MAX; 4];

    let mut deepest_dist = Real::MAX;
    for (i, pt) in manifold.points.iter().enumerate() {
        if pt.dist < deepest_dist {
            deepest_dist = pt.dist;
            selected[0] = i;
        }
    }

    if selected[0] == usize::MAX {
        *num_selected = 0;
        return;
    }

    // 2. Find the point that is the furthest from the deepest point.
    let selected_a = manifold.points[selected[0]].local_p1;
    let mut furthest_dist = -Real::MAX;
    for (i, pt) in manifold.points.iter().enumerate() {
        let dist = (pt.local_p1 - selected_a).length_squared();
        if i != selected[0] && dist > furthest_dist && is_solver_contact(i) {
            furthest_dist = dist;
            selected[1] = i;
        }
    }

    if selected[1] == usize::MAX {
        *num_selected = 1;
        return;
    }

    // 3. Now find the two points furthest from the segment we built so far.
    let selected_b = manifold.points[selected[1]].local_p1;

    if selected_a == selected_b {
        *num_selected = 1;
        return;
    }

    let selected_ab = selected_b - selected_a;
    let tangent = selected_ab.cross(manifold.local_n1);

    // Find the points that minimize and maximize the dot product with the tangent.
    let mut min_dot = Real::MAX;
    let mut max_dot = -Real::MAX;
    for (i, pt) in manifold.points.iter().enumerate() {
        if i == selected[0] || i == selected[1] || !is_solver_contact(i) {
            continue;
        }

        let dot = (pt.local_p1 - selected_a).dot(tangent);
        if dot < min_dot {
            min_dot = dot;
            selected[2] = i;
        }

        if dot > max_dot {
            max_dot = dot;
            selected[3] = i;
        }
    }

    if selected[2] == usize::MAX {
        *num_selected = 2;
        return;
    } else if selected[2] == selected[3] {
        *num_selected = 3;
        return;
    }
    *num_selected = 4;

    // 4. On a flat contact every point has the same depth up to rounding, so the "deepest"
    //    point can be one inside the contact patch (the extra GJK point of a face-face
    //    manifold, at the centre of a cylinder's cap). Anchored on it, steps 2 and 3 leave part
    //    of the patch without support. The depth varies linearly across a flat patch, so a
    //    point surrounded by the kept points needs no contact of its own: replace it with the
    //    point that enlarges the kept polygon most, when that polygon surrounds it. A deepest
    //    point on the patch's boundary is never surrounded, so it is always kept.
    use crate::utils::OrthonormalBasis;
    let basis = manifold.local_n1.orthonormal_basis();
    let proj = |i: usize| {
        let p = manifold.points[i].local_p1;
        (p.dot(basis[0]), p.dot(basis[1]))
    };
    let [b, c, d] = [proj(selected[1]), proj(selected[2]), proj(selected[3])];
    let a = proj(selected[0]);
    let area_a = hull_area4([a, b, c, d]);
    let mut best = usize::MAX;
    let mut best_area = area_a;
    for i in 0..manifold.points.len() {
        if selected.contains(&i) || !is_solver_contact(i) {
            continue;
        }
        let area = hull_area4([proj(i), b, c, d]);
        if area > best_area {
            best_area = area;
            best = i;
        }
    }
    // Gains within rounding (e.g. a duplicate of the deepest point) are not worth losing its
    // warm-started impulse.
    if best != usize::MAX
        && best_area > area_a * (1.0 + 1.0e-3)
        && in_hull4(a, [proj(best), b, c, d])
    {
        selected[0] = best;
    }
}

/// Twice the signed area of the triangle `(a, b, c)`.
fn cross2(a: (Real, Real), b: (Real, Real), c: (Real, Real)) -> Real {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// The area of the convex hull of four points.
fn hull_area4(p: [(Real, Real); 4]) -> Real {
    let quad = |a: usize, b: usize, c: usize, d: usize| {
        (cross2(p[a], p[b], p[c]) + cross2(p[a], p[c], p[d])).abs()
    };
    let tri = |a: usize, b: usize, c: usize| cross2(p[a], p[b], p[c]).abs();
    // The hull is the largest of the three quadrilaterals (convex position) or of the four
    // triangles (one point inside the others).
    quad(0, 1, 2, 3)
        .max(quad(0, 1, 3, 2))
        .max(quad(0, 2, 1, 3))
        .max(tri(0, 1, 2))
        .max(tri(0, 1, 3))
        .max(tri(0, 2, 3))
        .max(tri(1, 2, 3))
        * 0.5
}

/// Whether `x` lies in the convex hull of four points (in one of the triangles they form).
fn in_hull4(x: (Real, Real), p: [(Real, Real); 4]) -> bool {
    let in_tri = |a: (Real, Real), b: (Real, Real), c: (Real, Real)| {
        let d1 = cross2(a, b, x);
        let d2 = cross2(b, c, x);
        let d3 = cross2(c, a, x);
        !((d1 < 0.0 || d2 < 0.0 || d3 < 0.0) && (d1 > 0.0 || d2 > 0.0 || d3 > 0.0))
    };
    in_tri(p[0], p[1], p[2])
        || in_tri(p[0], p[1], p[3])
        || in_tri(p[0], p[2], p[3])
        || in_tri(p[1], p[2], p[3])
}
