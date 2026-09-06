//! Membership classification of surface vertices against another closed surface or a rigid collider (patch depths, inside tests).

use crate::alloc_prelude::*;

use crate::dynamics::{SoftBody, SoftCollisionMesh};
use crate::geometry::Collider;
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;
use parry::bounding_volume::BoundingVolume;
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Projects `point` on element `e` of `mesh`: the closest point, and whether it lies in
/// the element's interior (away from its boundary).
pub(super) fn project_on_element(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    e: usize,
    point: Vector,
) -> Option<(Vector, bool)> {
    use crate::geometry::PointQueryWithLocation;
    let element = mesh.element(e);
    if element.len() < DIM {
        return None;
    }
    let positions: [Vector; DIM] = core::array::from_fn(|k| mesh.vertex(body, element[k] as usize));
    #[cfg(feature = "dim2")]
    let (proj, weights) = {
        let (proj, loc) = parry::shape::Segment::new(positions[0], positions[1])
            .project_local_point_and_get_location(point, false);
        (proj, loc.barycentric_coordinates())
    };
    #[cfg(feature = "dim3")]
    let (proj, weights) = {
        let (proj, loc) = parry::shape::Triangle::new(positions[0], positions[1], positions[2])
            .project_local_point_and_get_location(point, false);
        (proj, loc.barycentric_coordinates()?)
    };
    let interior = weights.iter().all(|&w| w > 1.0e-3);
    Some((proj.point, interior))
}

/// Classifies the surface vertices of `mesh` against the closed `other` mesh by signed distance:
/// `depth[v] = rest - d` (`d` negative inside) within `reach` or inside, `NEG_INFINITY` otherwise.
/// Near vertices project on the BVH, others flood per component; returns (any member, any inside).
#[allow(clippy::too_many_arguments)]
pub(super) fn classify_skin(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    crossing: &[bool],
    (other_mesh, other, other_co): (&SoftCollisionMesh, &SoftBody, &Collider),
    reach: Real,
    rest: Real,
    depth: &mut Vec<Real>,
) -> (bool, bool) {
    let n_v = mesh.vertex_count();
    depth.clear();
    depth.resize(n_v, Real::NEG_INFINITY);
    let Some(other_bvh) = other_co.shape().as_composite_shape().map(|c| c.bvh()) else {
        return (false, false);
    };
    let other_inv_pose = other_co.position().inverse();
    let other_aabb = other_co.compute_aabb().loosened(reach);
    let mut any = false;
    let mut deep = false;
    let mut near = vec![false; n_v];
    for v in 0..n_v {
        if !mesh.vertex_on_surface(v) {
            continue;
        }
        let p = mesh.vertex(body, v);
        if !other_aabb.contains_local_point(p) {
            continue;
        }
        let aabb = crate::geometry::Aabb::from_half_extents(p, Vector::splat(reach))
            .transform_by(&other_inv_pose);
        let mut best: Option<(Real, u32, Vector, bool)> = None;
        for e in other_bvh.intersect_aabb(&aabb) {
            let Some((q, interior)) = project_on_element(other_mesh, other, e as usize, p) else {
                continue;
            };
            let d2 = (p - q).length_squared();
            if best.is_none_or(|(bd, ..)| d2 < bd) {
                best = Some((d2, e, q, interior));
            }
        }
        let Some((d2, e, q, interior)) = best.filter(|b| b.0 <= reach * reach) else {
            continue;
        };
        near[v] = true;
        let d = d2.sqrt();
        let inside = if d <= 1.0e-9 * (1.0 + reach) {
            false
        } else {
            match other_mesh.element_outward_normal(other, e as usize) {
                Some(n) => {
                    let along = (p - q).gdot(n);
                    let n_len = n.length();
                    if interior || along.abs() > 0.7 * d * n_len {
                        along < 0.0
                    } else {
                        other_mesh.contains_point_parity(other, p)
                    }
                }
                None => other_mesh.contains_point_parity(other, p),
            }
        };
        depth[v] = if inside { rest + d } else { rest - d };
        any = true;
        deep |= inside;
    }
    // The vertices beyond reach: components over the non-crossing elements made of such
    // vertices only, one parity test each; an inside component's vertices get their
    // distance from the BVH.
    let mut parent: Vec<u32> = (0..n_v as u32).collect();
    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            parent[i as usize] = parent[parent[i as usize] as usize];
            i = parent[i as usize];
        }
        i
    }
    let mut touched = vec![false; n_v];
    for (f, el) in mesh.indices().iter().enumerate() {
        if crossing.get(f).copied().unwrap_or(false) || el.iter().any(|&v| near[v as usize]) {
            continue;
        }
        for &v in el {
            touched[v as usize] = true;
        }
        for k in 1..el.len() {
            let (a, b) = (find(&mut parent, el[0]), find(&mut parent, el[k]));
            if a != b {
                parent[a as usize] = b;
            }
        }
    }
    let mut root_inside: HashMap<u32, bool> = HashMap::default();
    for v in 0..n_v {
        if near[v] || !mesh.vertex_on_surface(v) {
            continue;
        }
        let p = mesh.vertex(body, v);
        if !other_aabb.contains_local_point(p) {
            continue;
        }
        let inside = if touched[v] {
            let root = find(&mut parent, v as u32);
            *root_inside
                .entry(root)
                .or_insert_with(|| other_mesh.contains_point_parity(other, p))
        } else {
            other_mesh.contains_point_parity(other, p)
        };
        if inside {
            let proj = other_co
                .shape()
                .project_point(other_co.position(), p, false);
            depth[v] = rest + (proj.point - p).length();
            any = true;
            deep = true;
        }
    }
    if any {
        ring_depths(mesh, body, other_co, rest, depth);
    }
    (any, deep)
}

/// The depth of the outside vertices sharing an element with a member (see
/// `patch_vertices`): their distance to the other collider, so the elements crossing the
/// reach are clipped where they do, not halfway.
pub(super) fn ring_depths(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    other_co: &Collider,
    rest: Real,
    depth: &mut [Real],
) {
    let member = |v: u32, depth: &[Real]| depth[v as usize] > Real::NEG_INFINITY;
    let mut ring: Vec<u32> = Vec::new();
    for el in mesh.indices() {
        if el.iter().any(|&v| member(v, depth)) {
            for &v in el {
                if !member(v, depth) && !ring.contains(&v) {
                    ring.push(v);
                }
            }
        }
    }
    for v in ring {
        let p = mesh.vertex(body, v as usize);
        let proj = other_co
            .shape()
            .project_point(other_co.position(), p, false);
        depth[v as usize] = rest - (proj.point - p).length();
    }
}

/// Whether the surface vertices of the closed `mesh` lie in their own body's self-intersection
/// region (a mirrored lobe, a fold pushed through): non-crossing patches flood and parity-test as
/// one, crossing vertices individually, each point nudged outward by half the contact `skin`.
pub(super) fn classify_inside_self(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    skin: Real,
    crossing: &[bool],
    vertex_elements: &[Vec<u32>],
    inside: &mut Vec<bool>,
) -> bool {
    let n_v = mesh.vertex_count();
    inside.clear();
    inside.resize(n_v, false);
    let nudge = (0.5 * skin).max(1.0e-4);
    let probe = |v: usize| -> Option<Vector> {
        let mut n = Vector::ZERO;
        for &e in vertex_elements.get(v)? {
            n += mesh.element_outward_normal(body, e as usize)?;
        }
        Some(mesh.vertex(body, v) + n.try_normalize()? * nudge)
    };
    let mut parent: Vec<u32> = (0..n_v as u32).collect();
    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            parent[i as usize] = parent[parent[i as usize] as usize];
            i = parent[i as usize];
        }
        i
    }
    let mut touched = vec![false; n_v];
    for (f, el) in mesh.indices().iter().enumerate() {
        if crossing.get(f).copied().unwrap_or(false) {
            continue;
        }
        for &v in el {
            touched[v as usize] = true;
        }
        for k in 1..el.len() {
            let (a, b) = (find(&mut parent, el[0]), find(&mut parent, el[k]));
            if a != b {
                parent[a as usize] = b;
            }
        }
    }
    let mut root_inside: HashMap<u32, bool> = HashMap::default();
    let mut any = false;
    for v in 0..n_v {
        if !mesh.vertex_on_surface(v) {
            continue;
        }
        let test = |v: usize| probe(v).is_some_and(|p| mesh.contains_point_parity(body, p));
        inside[v] = if touched[v] {
            let root = find(&mut parent, v as u32);
            *root_inside
                .entry(root)
                .or_insert_with(|| test(root as usize))
        } else {
            test(v)
        };
        any |= inside[v];
    }
    any
}

/// Classifies the surface vertices of `mesh` against a rigid collider, like
/// `classify_skin`: by containment alone when `reach` is `None` (the intruding patch), by
/// signed distance within the reach otherwise.
pub(super) fn classify_skin_rigid(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    other_co: &Collider,
    reach: Option<Real>,
    rest: Real,
    depth: &mut Vec<Real>,
) -> (bool, bool) {
    let n_v = mesh.vertex_count();
    depth.clear();
    depth.resize(n_v, Real::NEG_INFINITY);
    let aabb = other_co.compute_aabb().loosened(reach.unwrap_or(0.0));
    let mut any = false;
    let mut deep = false;
    for v in 0..n_v {
        if !mesh.vertex_on_surface(v) {
            continue;
        }
        let p = mesh.vertex(body, v);
        if !aabb.contains_local_point(p) {
            continue;
        }
        let inside = other_co.shape().contains_point(other_co.position(), p);
        if !inside && reach.is_none() {
            continue;
        }
        let proj = other_co
            .shape()
            .project_point(other_co.position(), p, false);
        let d = (proj.point - p).length();
        if !inside && d > reach.unwrap_or(0.0) {
            continue;
        }
        depth[v] = if inside { rest + d } else { rest - d };
        any = true;
        deep |= inside;
    }
    if any {
        ring_depths(mesh, body, other_co, rest, depth);
    }
    (any, deep)
}

/// Whether the vertices of `mesh` lie inside the closed `other` mesh: the patches of
/// non-crossing elements are flooded (union-find through shared vertices) and classified by
/// one parity test each, the crossing elements' vertices individually.
pub(super) fn classify_inside(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    crossing: &[bool],
    (other_mesh, other): (&SoftCollisionMesh, &SoftBody),
    inside: &mut Vec<bool>,
) {
    let n_v = mesh.vertex_count();
    inside.clear();
    inside.resize(n_v, false);
    let mut parent: Vec<u32> = (0..n_v as u32).collect();
    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            parent[i as usize] = parent[parent[i as usize] as usize];
            i = parent[i as usize];
        }
        i
    }
    let mut touched = vec![false; n_v];
    for (f, el) in mesh.indices().iter().enumerate() {
        if crossing.get(f).copied().unwrap_or(false) {
            continue;
        }
        for &v in el {
            touched[v as usize] = true;
        }
        for k in 1..el.len() {
            let (a, b) = (find(&mut parent, el[0]), find(&mut parent, el[k]));
            if a != b {
                parent[a as usize] = b;
            }
        }
    }
    let mut root_inside: HashMap<u32, bool> = HashMap::default();
    for v in 0..n_v {
        if !mesh.vertex_on_surface(v) {
            continue;
        }
        let p = mesh.vertex(body, v);
        inside[v] = if touched[v] {
            let root = find(&mut parent, v as u32);
            *root_inside
                .entry(root)
                .or_insert_with(|| other_mesh.contains_point_parity(other, p))
        } else {
            other_mesh.contains_point_parity(other, p)
        };
    }
}
