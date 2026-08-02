use crate::geometry::ContactManifold;
use crate::math::Real;

pub(crate) fn reduce_manifold_naive(
    manifold: &ContactManifold,
    selected: &mut [usize; 4],
    num_selected: &mut usize,
    prediction_distance: Real,
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
        if i != selected[0] && pt.dist <= prediction_distance && dist > furthest_dist {
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
        if i == selected[0] || i == selected[1] || pt.dist > prediction_distance {
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
    } else if selected[2] == selected[3] {
        *num_selected = 3;
    } else {
        *num_selected = 4;
    }
}
