//! The volume patches of crossed closed surfaces (soft-soft and soft-rigid) and their binning into volume constraints.

use crate::alloc_prelude::*;

use crate::dynamics::{IntegrationParameters, RigidBodySet, SoftBody, SoftCollisionMesh};
use crate::geometry::Collider;
use crate::math::{DIM, Real, Vector};
use crate::utils::{DotProduct, OrthonormalBasis};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_contacts_classify::{classify_inside, classify_skin, classify_skin_rigid, ring_depths};
use super::{Side, SoftDetectionCtx, SoftVertexPass, SoftRigidPatch, SoftVolumePatch, body_frozen};

/// The multi-volume grid's cells per tangent axis (see `overlap_multi_volume`).
pub(super) fn volume_split(params: &IntegrationParameters) -> u32 {
    if params.soft_bodies.recovery.overlap_multi_volume {
        params.soft_bodies.recovery.overlap_split.max(1)
    } else {
        1
    }
}

/// The depth of the vertices `inside` the other collider: their distance to its surface
/// (`NEG_INFINITY` for the others). Returns whether any vertex is inside.
pub(super) fn fill_depths(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    inside: &[bool],
    co: &Collider,
    depth: &mut Vec<Real>,
) -> bool {
    depth.clear();
    depth.resize(mesh.vertex_count(), Real::NEG_INFINITY);
    let mut any = false;
    for v in 0..mesh.vertex_count() {
        if inside[v] {
            let p = mesh.vertex(body, v);
            let proj = co.shape().project_point(co.position(), p, false);
            depth[v] = (proj.point - p).length();
            any = true;
        }
    }
    any
}

/// Whether a closed mesh is self-crossed this step (part of its winding is mirrored, so its
/// volume gradient points the wrong way there): its volume constraints sit out.
fn self_tangled(m: &SoftCollisionMesh) -> bool {
    m.orientation_unreliable || m.crossed_partners.contains(&m.collider())
}

/// The volume patches of a crossed pair of closed surfaces (see `SoftOverlapConstraint`, Allard et
/// al. 2010): both intruding patches, on the skin-dilated surfaces under `overlap_skin_volume`,
/// binned from `pass` (the vertices of `vb` against the surface of `eb`, the bins' own side).
pub(super) fn detect_volume_patch(
    pass: &SoftVertexPass,
    (eb, eb_mesh, eb_handle, eb_co): Side<'_>,
    (vb, vb_mesh, _, vb_co): Side<'_>,
    ctx: &SoftDetectionCtx,
) -> Option<SoftVolumePatch> {
    let params = ctx.params;
    let recovery = &params.soft_bodies.recovery;
    let closed_pair = vb_mesh.is_closed() && eb_mesh.is_closed();
    let crossed = !pass.cross_pairs.is_empty() && !pass.cross_tangled_vb_elements.is_empty();
    if !recovery.overlap_constraints
        || !closed_pair
        || !crossed
        || (recovery.overlap_skip_self_tangled && (self_tangled(vb_mesh) || self_tangled(eb_mesh)))
    {
        return None;
    }
    let skins = vb_co.contact_skin() + eb_co.contact_skin();
    let mut depth_vb = Vec::new();
    let mut depth_eb = Vec::new();
    let band = params.prediction_distance() + ctx.motion_margin(eb_co) + ctx.motion_margin(vb_co);
    let any = if recovery.overlap_skin_volume {
        let rest = (1.0 - recovery.overlap_kept_depth.clamp(0.0, 1.0)) * skins;
        let reach = skins + band;
        let (a1, _) = classify_skin(
            vb_mesh,
            vb,
            &pass.cross_tangled_vb_elements,
            (eb_mesh, eb, eb_co),
            reach,
            rest,
            &mut depth_vb,
        );
        let (a2, _) = classify_skin(
            eb_mesh,
            eb,
            &pass.cross_tangled_elements,
            (vb_mesh, vb, vb_co),
            reach,
            rest,
            &mut depth_eb,
        );
        a1 || a2
    } else {
        // The geometric intersection alone: the inside vertices, at their distance to the
        // other surface.
        let mut inside_vb = Vec::new();
        let mut inside_eb = Vec::new();
        classify_inside(
            vb_mesh,
            vb,
            &pass.cross_tangled_vb_elements,
            (eb_mesh, eb),
            &mut inside_vb,
        );
        classify_inside(
            eb_mesh,
            eb,
            &pass.cross_tangled_elements,
            (vb_mesh, vb),
            &mut inside_eb,
        );
        let a1 = fill_depths(vb_mesh, vb, &inside_vb, eb_co, &mut depth_vb);
        let a2 = fill_depths(eb_mesh, eb, &inside_eb, vb_co, &mut depth_eb);
        if a1 {
            ring_depths(vb_mesh, vb, eb_co, 0.0, &mut depth_vb);
        }
        if a2 {
            ring_depths(eb_mesh, eb, vb_co, 0.0, &mut depth_eb);
        }
        a1 || a2
    };
    if !any {
        return None;
    }
    let band = if recovery.overlap_skin_volume {
        band
    } else {
        0.0
    };
    let own = VolumeSide {
        body: eb,
        mesh: eb_mesh,
        vertices: patch_vertices(eb_mesh, eb, &depth_eb, band),
    };
    let other = VolumeSide {
        body: vb,
        mesh: vb_mesh,
        vertices: patch_vertices(vb_mesh, vb, &depth_vb, band),
    };
    Some(SoftVolumePatch {
        own: eb_handle,
        bins: volume_bins(&own, Some(&other), volume_split(params)),
        slot_offset: u32::MAX,
    })
}

/// The volume patch of a closed surface against a rigid collider (see `SoftOverlapConstraint`):
/// the surface vertices inside the rigid shape (plus its skin layer under the skin models), found
/// independently of the pair's manifolds (a shape fully inside the surface has no contact).
pub(super) fn detect_rigid_patch(
    (sb, mesh, handle, surface_co): Side<'_>,
    other_co: &Collider,
    bodies: &RigidBodySet,
    ctx: &SoftDetectionCtx,
) -> Option<SoftRigidPatch> {
    let params = ctx.params;
    let recovery = &params.soft_bodies.recovery;
    if !recovery.overlap_constraints
        || !recovery.overlap_rigid
        || body_frozen(sb)
        || !mesh.collision_enabled()
        || other_co.is_sensor()
    {
        return None;
    }
    // A self-crossed mesh sits out: part of its winding is mirrored.
    if !mesh.is_closed()
        || (recovery.overlap_skip_self_tangled
            && (mesh.orientation_unreliable || mesh.crossed_partners.contains(&handle)))
    {
        return None;
    }
    let skin = recovery.overlap_skin_volume;
    let other_rb = other_co.parent().and_then(|h| bodies.get(h));
    let skins = surface_co.contact_skin() + other_co.contact_skin();
    let rest = if skin {
        (1.0 - recovery.overlap_kept_depth.clamp(0.0, 1.0)) * skins
    } else {
        0.0
    };
    let band = params.prediction_distance() + ctx.motion_margin(surface_co);
    let reach = skin.then_some(skins + band);
    let mut depth = Vec::new();
    let (any, _) = classify_skin_rigid(mesh, sb, other_co, reach, rest, &mut depth);
    if !any {
        return None;
    }
    let own = VolumeSide {
        body: sb,
        mesh,
        vertices: patch_vertices(mesh, sb, &depth, if skin { band } else { 0.0 }),
    };
    let bins = volume_bins(&own, None, volume_split(params));
    Some(SoftRigidPatch {
        bins,
        depth: if recovery.overlap_patch_constraints != crate::dynamics::SoftPatchConstraints::Keep {
            depth
        } else {
            Vec::new()
        },
        com: other_rb.map_or(other_co.translation(), |rb| rb.center_of_mass()),
        rot: other_co.position().rotation,
    })
}

/// Accumulates, per vertex of `mesh`, the gradient of the enclosed area/volume over the elements
/// `keep` accepts (weighted as it returns); translation-invariant per element, so a subset of a
/// closed boundary is fine. `sign` flips the gradient (a mirrored lobe raises its negative area).
fn partial_volume_gradients(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    sign: Real,
    mut keep: impl FnMut(usize) -> Option<Real>,
    grads: &mut [Vector],
) {
    for (f, el) in mesh.indices().iter().enumerate() {
        let Some(w) = keep(f) else {
            continue;
        };
        #[cfg(feature = "dim2")]
        {
            let (a, b) = (
                mesh.vertex(body, el[0] as usize),
                mesh.vertex(body, el[1] as usize),
            );
            let d = b - a;
            let g = Vector::new(d.y, -d.x) * (0.5 * w * sign);
            grads[el[0] as usize] += g;
            grads[el[1] as usize] += g;
        }
        #[cfg(feature = "dim3")]
        {
            let (a, b, c) = (
                mesh.vertex(body, el[0] as usize),
                mesh.vertex(body, el[1] as usize),
                mesh.vertex(body, el[2] as usize),
            );
            let n = (b - a).cross(c - a);
            let g = n * (w * sign / 6.0);
            for &v in el {
                grads[v as usize] += g;
            }
        }
    }
}

/// A vertex of a volume contact's patch: its share of the (partial) volume gradient and its
/// depth in the other side's volume (the skin-dilated one under the skin models, where it
/// is negative in the speculative band and above the kept layer).
#[derive(Copy, Clone)]
pub(super) struct PatchVertex {
    vertex: u32,
    grad: Vector,
    depth: Real,
}

/// One side of a volume contact: its patch vertices.
pub(super) struct VolumeSide<'a> {
    pub(super) body: &'a SoftBody,
    pub(super) mesh: &'a SoftCollisionMesh,
    pub(super) vertices: Vec<PatchVertex>,
}

/// One volume constraint of a pair (see `SoftOverlapConstraint`): the whole patch, or a cell of
/// the multi-volume grid (Allard et al. 2010, section 5), as particle entries (particle,
/// gradient) per side.
#[derive(Clone, Debug)]
pub struct VolumeBin {
    /// The own side's entries: (particle, volume gradient).
    pub own: Vec<(u32, Vector)>,
    /// The other side's entries: (particle, volume gradient); empty against a rigid collider.
    pub other: Vec<(u32, Vector)>,
    /// The signed volume estimate, `sum depth |g|` over both sides (negative: slack).
    pub volume: Real,
    /// The contact normal, from the own side into the other (zero: degenerate).
    pub normal: Vector,
    /// The area-weighted center of the bin's vertices.
    pub center: Vector,
    /// Whether the solver builds a constraint for it (the hook clears it to drop the constraint).
    pub enabled: bool,
    /// The impulse the solver applied through it at the last step (its multiplier along
    /// the own side's gradients).
    pub impulse: Real,
}

/// The fraction of an element lying in the volume where the depth (offset by the band,
/// see `patch_vertices`) is positive, the zero crossing interpolated linearly along its
/// edges: exact for a segment, the standard triangle clip in 3D.
fn clipped_fraction(s: &[Real]) -> Real {
    let inside: Vec<usize> = (0..s.len()).filter(|&i| s[i] >= 0.0).collect();
    if inside.len() == s.len() {
        return 1.0;
    }
    if inside.is_empty() {
        return 0.0;
    }
    // The inside vertex's share along an edge to an outside vertex (half the edge when
    // that vertex has no depth: the member-fraction rule).
    let share = |a: usize, b: usize| -> Real {
        if s[b] == Real::NEG_INFINITY {
            0.5
        } else {
            (s[a] / (s[a] - s[b])).clamp(0.0, 1.0)
        }
    };
    #[cfg(feature = "dim2")]
    {
        let a = inside[0];
        share(a, 1 - a)
    }
    #[cfg(feature = "dim3")]
    {
        if inside.len() == 1 {
            let a = inside[0];
            let (b, c) = ((a + 1) % 3, (a + 2) % 3);
            share(a, b) * share(a, c)
        } else {
            let o = (0..3).find(|i| !inside.contains(i)).unwrap();
            let (b, c) = ((o + 1) % 3, (o + 2) % 3);
            let out_share = |i: usize| {
                if s[o] == Real::NEG_INFINITY {
                    0.5
                } else {
                    (s[i] / (s[i] - s[o])).clamp(0.0, 1.0)
                }
            };
            1.0 - (1.0 - out_share(b)) * (1.0 - out_share(c))
        }
    }
}

/// The patch of `mesh` made of its member vertices (`depth[v]` finite): every element weighs the
/// fraction of it within reach (depth offset by the speculative slack `band`, interpolated along
/// its edges), and each of its vertices gets its share of the volume gradient (none if depthless).
pub(super) fn patch_vertices(
    mesh: &SoftCollisionMesh,
    body: &SoftBody,
    depth: &[Real],
    band: Real,
) -> Vec<PatchVertex> {
    let member = |v: usize| depth[v] > Real::NEG_INFINITY;
    let mut grads = vec![Vector::ZERO; mesh.vertex_count()];
    // The gradient follows the geometric orientation, not the raw winding: a loop turned
    // inside out (crushed against a wall, no self-crossing) would otherwise be inflated
    // out of the wall instead of shrunk.
    partial_volume_gradients(
        mesh,
        body,
        mesh.winding_sign(),
        |f| {
            let el = mesh.element(f);
            let s: [Real; DIM] = core::array::from_fn(|k| {
                el.get(k).map_or(Real::NEG_INFINITY, |&v| {
                    let d = depth[v as usize];
                    if d == Real::NEG_INFINITY { d } else { d + band }
                })
            });
            let w = clipped_fraction(&s[..el.len().min(DIM)]);
            (w > 0.0).then_some(w)
        },
        &mut grads,
    );
    // A vertex beyond the reach (a ring vertex, whose depth only places the clip) bounds
    // the patch without adding volume.
    grads
        .iter()
        .enumerate()
        .filter(|(_, g)| **g != Vector::ZERO)
        .map(|(v, g)| PatchVertex {
            vertex: v as u32,
            grad: *g,
            depth: if member(v) && depth[v] >= -band {
                depth[v]
            } else {
                0.0
            },
        })
        .collect()
}

/// The particle-level entries of a side's patch vertices (a skinned vertex spread over its
/// cell), sorted by particle.
fn side_entries(side: &VolumeSide, picked: &[usize]) -> Vec<(u32, Vector)> {
    let mut by_particle: HashMap<u32, Vector> = HashMap::default();
    for &i in picked {
        let pv = side.vertices[i];
        let (anchors, weights) = side.mesh.vertex_anchors(side.body, pv.vertex as usize);
        for (p, w) in anchors.iter().zip(&weights) {
            if *p != u32::MAX && *w != 0.0 {
                *by_particle.entry(*p).or_insert(Vector::ZERO) += pv.grad * *w;
            }
        }
    }
    let mut entries: Vec<(u32, Vector)> = by_particle.into_iter().collect();
    entries.sort_by_key(|e| e.0);
    entries
}

/// Splits a pair's patches into the volume constraints: one bin (mono-volume), or `split` cells
/// per tangent axis of a regular grid over the patches, aligned with the pair's mean normal
/// (multi-volume). A cell missing one side joins the nearest two-sided cell; with none, one bin.
pub(super) fn volume_bins(own: &VolumeSide, other: Option<&VolumeSide>, split: u32) -> Vec<VolumeBin> {
    let n_own = own.vertices.len();
    let n_other = other.map_or(0, |o| o.vertices.len());
    if n_own + n_other == 0 {
        return Vec::new();
    }
    // Every vertex of both sides: position, gradient (the other side's negated, so the
    // sum points from the own side into the other), and its side index.
    let pos = |i: usize| -> Vector {
        if i < n_own {
            own.mesh.vertex(own.body, own.vertices[i].vertex as usize)
        } else {
            let o = other.unwrap();
            o.mesh.vertex(o.body, o.vertices[i - n_own].vertex as usize)
        }
    };
    let pv = |i: usize| -> PatchVertex {
        if i < n_own {
            own.vertices[i]
        } else {
            other.unwrap().vertices[i - n_own]
        }
    };
    let total = n_own + n_other;
    let mut normal_sum = Vector::ZERO;
    for i in 0..total {
        normal_sum += if i < n_own { pv(i).grad } else { -pv(i).grad };
    }
    let normal = normal_sum.try_normalize().unwrap_or(Vector::ZERO);
    // The cells: each vertex's grid coordinates along the tangent axes.
    let mut cells: Vec<Vec<usize>> = Vec::new();
    let split = split.max(1) as usize;
    if split > 1 && normal != Vector::ZERO && total >= 2 {
        let tangents = normal.orthonormal_basis();
        let mut center = Vector::ZERO;
        for i in 0..total {
            center += pos(i);
        }
        center /= total as Real;
        let coords: Vec<[Real; DIM - 1]> = (0..total)
            .map(|i| core::array::from_fn(|j| (pos(i) - center).gdot(tangents[j])))
            .collect();
        let mut lo = [Real::MAX; DIM - 1];
        let mut hi = [-Real::MAX; DIM - 1];
        for c in &coords {
            for j in 0..DIM - 1 {
                lo[j] = lo[j].min(c[j]);
                hi[j] = hi[j].max(c[j]);
            }
        }
        let cell_of = |c: &[Real; DIM - 1]| -> usize {
            let mut idx = 0;
            for j in 0..DIM - 1 {
                let span = hi[j] - lo[j];
                let k = if span > 0.0 {
                    (((c[j] - lo[j]) / span) * split as Real) as usize
                } else {
                    0
                };
                idx = idx * split + k.min(split - 1);
            }
            idx
        };
        let n_cells = split.pow((DIM - 1) as u32);
        cells.resize(n_cells, Vec::new());
        for (i, c) in coords.iter().enumerate() {
            cells[cell_of(c)].push(i);
        }
        let two_sided = |cell: &Vec<usize>| {
            other.is_none() || (cell.iter().any(|&i| i < n_own) && cell.iter().any(|&i| i >= n_own))
        };
        let cell_center = |k: usize| -> [Real; DIM - 1] {
            let mut c = [0.0; DIM - 1];
            let mut idx = k;
            for j in (0..DIM - 1).rev() {
                c[j] = (idx % split) as Real;
                idx /= split;
            }
            c
        };
        let keep: Vec<bool> = cells
            .iter()
            .map(|c| !c.is_empty() && two_sided(c))
            .collect();
        if keep.iter().any(|&k| k) {
            for k in 0..n_cells {
                if keep[k] || cells[k].is_empty() {
                    continue;
                }
                let ck = cell_center(k);
                let mut best = (Real::MAX, usize::MAX);
                for t in 0..n_cells {
                    if !keep[t] {
                        continue;
                    }
                    let ct = cell_center(t);
                    let d2: Real = (0..DIM - 1).map(|j| (ck[j] - ct[j]).powi(2)).sum();
                    if d2 < best.0 {
                        best = (d2, t);
                    }
                }
                let moved = core::mem::take(&mut cells[k]);
                cells[best.1].extend(moved);
            }
            cells.retain(|c| !c.is_empty());
        } else {
            cells.clear();
        }
    }
    if cells.is_empty() {
        cells.push((0..total).collect());
    }
    cells
        .iter()
        .map(|cell| {
            let own_idx: Vec<usize> = cell.iter().copied().filter(|&i| i < n_own).collect();
            let other_idx: Vec<usize> = cell
                .iter()
                .filter(|&&i| i >= n_own)
                .map(|&i| i - n_own)
                .collect();
            let mut volume = 0.0;
            let mut normal_sum = Vector::ZERO;
            let mut center = Vector::ZERO;
            let mut weight = 0.0;
            for &i in cell {
                let v = pv(i);
                let a = v.grad.length();
                volume += v.depth * a;
                normal_sum += if i < n_own { v.grad } else { -v.grad };
                center += pos(i) * a;
                weight += a;
            }
            VolumeBin {
                own: side_entries(own, &own_idx),
                other: other.map_or_else(Vec::new, |o| side_entries(o, &other_idx)),
                volume,
                normal: normal_sum.try_normalize().unwrap_or(Vector::ZERO),
                center: if weight > 0.0 {
                    center / weight
                } else {
                    Vector::ZERO
                },
                enabled: true,
                impulse: 0.0,
            }
        })
        .collect()
}

