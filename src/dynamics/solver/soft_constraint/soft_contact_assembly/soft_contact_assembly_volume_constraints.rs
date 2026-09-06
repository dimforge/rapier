//! Intersection-volume constraints of a pair's patch bins: the bins' emission, the overlap arming across steps, the patch constraint entries and the rigid-side volume contacts.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;

use super::{AssemblyCtx, BodyContacts, MeshContacts, OverlapConstraintWorkspace, material_pace};
use super::super::soft_constraints_set::SoftConstraintsSet;
use crate::dynamics::soft_body::SoftPatchConstraints;
use crate::dynamics::soft_body::{SoftCollisionMesh, SoftOverlapState, SoftVolumeContact};
use crate::dynamics::{IntegrationParameters, SoftBody};
use crate::geometry::ColliderHandle;
use crate::geometry::soft_contacts::VolumeBin;
use crate::math::{AngVector, Real, Rotation, Vector};
use crate::utils::{CrossProduct, DotProduct};

/// The bins of a pair's volume patch seen from its other side: the sides swapped, the normal
/// reversed.
pub(super) fn mirrored_bins(bins: &[VolumeBin]) -> Vec<VolumeBin> {
    bins.iter()
        .map(|b| VolumeBin {
            own: b.other.clone(),
            other: b.own.clone(),
            volume: b.volume,
            normal: -b.normal,
            center: b.center,
            enabled: b.enabled,
            impulse: b.impulse,
        })
        .collect()
}

/// The rigid side of a volume contact.
pub(super) struct RigidSide {
    /// Its solver slot (`u32::MAX`: not simulated this step).
    slot: u32,
    com: Vector,
    rot: Rotation,
}

/// Emits one volume constraint per bin of a pair (see `SoftOverlapConstraint`), the signed volume
/// as correction; `None` slots: side not simulated (fixed anchors); `hard`: no stall fade;
/// `slot_offset`: first bin's contact slot (`u32::MAX`: none). Returns the bins' (center, normal).
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_volume_bins(
    params: &IntegrationParameters,
    out: &mut BodyContacts,
    mesh: &SoftCollisionMesh,
    other_handle: ColliderHandle,
    own: (&SoftBody, Option<&[u32]>),
    other: Option<(&SoftBody, Option<&[u32]>)>,
    rigid: Option<RigidSide>,
    hard: bool,
    bins: &[VolumeBin],
    slot_offset: u32,
) -> Vec<(Vector, Vector)> {
    let recovery = &params.soft_bodies.recovery;
    let mut cells = Vec::with_capacity(bins.len());
    if bins.is_empty() {
        return cells;
    }
    let error: Real = bins.iter().map(|b| b.volume.max(0.0)).sum();
    let scale = if hard {
        1.0
    } else {
        overlap_arming(params, mesh, out.mesh(), other_handle, error).scale
    };
    let warm_coeff = params.warmstart_coefficient;
    // The impulses the pair's particles accumulated in the last step (a hard constraint only).
    let prev_warm = (hard && warm_coeff != 0.0)
        .then(|| mesh.overlap_warm.iter().find(|w| w.other == other_handle))
        .flatten();
    let prev_warm_of = |side: u8, p: u32| -> Vector {
        prev_warm.map_or(Vector::ZERO, |w| {
            let list = if side == 0 { &w.own } else { &w.other_soft };
            list.iter()
                .find(|e| e.0 == p)
                .map_or(Vector::ZERO, |e| e.1 * warm_coeff)
        })
    };
    for (bi, bin) in bins.iter().enumerate() {
        if !bin.enabled {
            continue;
        }
        let mut bin = bin.clone();
        // Along the normal instead of the gradients (see `overlap_normal_push`): each entry
        // keeps its magnitude, the sides take opposite directions.
        if recovery.overlap_normal_push && bin.normal != Vector::ZERO {
            let n = bin.normal;
            for (_, g) in &mut bin.own {
                *g = n * g.length();
            }
            for (_, g) in &mut bin.other {
                *g = -n * g.length();
            }
        }
        // The constraint as drawn by the debug renderer.
        let mut gradients: Vec<(Vector, Vector)> = bin
            .own
            .iter()
            .map(|&(p, g)| (own.0.particles[p as usize].position, g))
            .collect();
        if let Some((o, _)) = other {
            gradients.extend(
                bin.other
                    .iter()
                    .map(|&(p, g)| (o.particles[p as usize].position, g)),
            );
        }
        out.mesh().volume_contacts.push(SoftVolumeContact {
            center: bin.center,
            normal: bin.normal,
            volume: bin.volume,
            gradients,
        });
        if bin.normal != Vector::ZERO {
            cells.push((bin.center, bin.normal));
        }
        // The rigid side takes the reaction: the negated gradient sum (the two surfaces
        // bound one volume) with its torque about the center of mass.
        let mut g_lin = Vector::ZERO;
        let mut g_ang = AngVector::default();
        if let Some(rs) = &rigid {
            for &(p, g) in &bin.own {
                g_lin -= g;
                g_ang -= (own.0.particles[p as usize].position - rs.com).gcross(g);
            }
        }
        let mut grads = Vec::new();
        let mut particles = Vec::new();
        patch_constraint_entries(own.0, 0, own.1, &bin.own, &mut grads, &mut particles);
        if let Some((o, o_slots)) = other {
            patch_constraint_entries(o, 1, o_slots, &bin.other, &mut grads, &mut particles);
        }
        let rigid_part = rigid
            .as_ref()
            .filter(|rs| rs.slot != u32::MAX)
            .map(|rs| (rs.slot, g_lin, g_ang));
        // The warm impulse of every entry, and the multiplier fitted to them (least
        // squares along the new gradients): what the solve starts from.
        let (warm, warm_impulses, warm_rigid) = if hard {
            let warm_impulses: Vec<Vector> =
                particles.iter().map(|&(side, p)| prev_warm_of(side, p)).collect();
            let mut num = 0.0;
            let mut den = 0.0;
            for (&(_, _, g, _), &p) in grads.iter().zip(&warm_impulses) {
                num -= p.gdot(g);
                den += g.length_squared();
            }
            let impulse = if den > 0.0 { (num / den).max(0.0) } else { 0.0 };
            let warm_rigid = prev_warm.map_or((Vector::ZERO, AngVector::default()), |w| {
                (w.rigid.0 * warm_coeff, w.rigid.1 * warm_coeff)
            });
            (Some((other_handle, impulse)), warm_impulses, warm_rigid)
        } else {
            (None, Vec::new(), (Vector::ZERO, AngVector::default()))
        };
        // A hard constraint applies the signed volume (its slack is speculative); a recovery constraint
        // applies the volume, metered by the stall fade.
        let rhs = if hard {
            bin.volume
        } else {
            bin.volume.max(0.0) * scale
        };
        if !grads.is_empty() || rigid_part.is_some() {
            out.overlap_constraints.push(OverlapConstraintWorkspace {
                grads,
                particles,
                rigid: rigid_part,
                rigid_pose0: rigid
                    .as_ref()
                    .map_or((Vector::ZERO, Rotation::IDENTITY), |rs| (rs.com, rs.rot)),
                rhs,
                hard,
                warm,
                warm_impulses,
                warm_rigid,
                max_bias_velocity: if hard {
                    params.max_corrective_velocity()
                } else {
                    recovery.overlap_constraint_pace * material_pace(params)
                },
                report: (
                    mesh.collider(),
                    other_handle,
                    if slot_offset == u32::MAX {
                        u32::MAX
                    } else {
                        slot_offset + bi as u32
                    },
                ),
            });
        }
    }
    cells
}

/// How much of a pair's overlap correction runs this step (see `SoftOverlapState`): full while the
/// volume estimate improves, fading over the second half of the patience once it stalls, stood
/// down after it; re-arms only when the estimate drops five margins below its best. Writes `out`.
struct OverlapArming {
    /// Scale of the constraint's closing rate (`0.0`: stood down).
    scale: Real,
}

fn overlap_arming(
    params: &IntegrationParameters,
    mesh: &SoftCollisionMesh,
    out: &mut MeshContacts,
    other: ColliderHandle,
    error: Real,
) -> OverlapArming {
    let margin = params.soft_bodies.recovery.overlap_progress_margin;
    let patience = params.soft_bodies.recovery.overlap_patience.min(u16::MAX as u32) as u16;
    let previous = mesh.overlap_states.iter().find(|s| s.other == other).copied();
    let state = match previous {
        Some(prev) => {
            let needed = if prev.stalled { 5.0 * margin } else { margin };
            if error < prev.best_error * (1.0 - needed) {
                SoftOverlapState {
                    other,
                    best_error: error,
                    steps_stuck: 0,
                    stalled: false,
                }
            } else {
                let steps_stuck = prev.steps_stuck.saturating_add(1);
                SoftOverlapState {
                    steps_stuck,
                    stalled: prev.stalled || steps_stuck > patience,
                    ..prev
                }
            }
        }
        None => SoftOverlapState {
            other,
            best_error: error,
            steps_stuck: 0,
            stalled: false,
        },
    };
    out.overlap_states.push(state);
    let scale = if state.stalled || patience == 0 {
        0.0
    } else {
        let half = patience / 2;
        if state.steps_stuck <= half {
            1.0
        } else {
            1.0 - (state.steps_stuck - half) as Real / (patience - half).max(1) as Real
        }
    };
    OverlapArming {
        scale: scale.clamp(0.0, 1.0),
    }
}

/// The `(slot, inverse mass, gradient, position)` constraint entries of a body's patch entries,
/// with their `(side, particle)`; `None` slots (the body is not simulated this step) make
/// the entries fixed anchors.
fn patch_constraint_entries(
    body: &SoftBody,
    side: u8,
    slots: Option<&[u32]>,
    entries: &[(u32, Vector)],
    out: &mut Vec<(u32, Real, Vector, Vector)>,
    particles: &mut Vec<(u8, u32)>,
) {
    let Some(slots) = slots else {
        return;
    };
    for &(p, g) in entries {
        let slot = slots[p as usize];
        let part = &body.particles[p as usize];
        if slot != u32::MAX && part.inv_mass > 0.0 {
            out.push((slot, part.inv_mass, g, part.position));
            particles.push((side, p));
        }
    }
}

impl SoftConstraintsSet {
    /// The volume contacts of awake body `ai`'s closed `mesh` against the rigid colliders it
    /// has a narrow-phase pair with (see `SoftOverlapConstraint`), from the patches the narrow phase
    /// detected on the pairs: the rigid side takes the reaction.
    pub(super) fn assemble_rigid_overlap(
        &self,
        ai: usize,
        ctx: &AssemblyCtx,
        mesh: &SoftCollisionMesh,
        surface_handle: ColliderHandle,
        out: &mut BodyContacts,
    ) {
        let AssemblyCtx {
            island_id,
            params,
            narrow_phase,
            colliders,
            bodies,
            ..
        } = *ctx;
        let recovery = &params.soft_bodies.recovery;
        let awake = &self.awake[ai];
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        // Standing the patch's per-point constraints down (or bending them) leaves the volume
        // constraint as the support of those features: a hard constraint, not a paced recovery.
        let hard = recovery.overlap_skin_volume
            || recovery.overlap_patch_constraints != SoftPatchConstraints::Keep;
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        for pair in narrow_phase.contact_pairs_with(surface_handle) {
            let other_handle = if pair.collider1 == surface_handle {
                pair.collider2
            } else {
                pair.collider1
            };
            let Some(other_co) = colliders.get(other_handle) else {
                continue;
            };
            if other_co.deformable_mesh_ref.is_some() || other_co.is_sensor() {
                continue;
            }
            let Some(patch) = pair.rigid().and_then(|r| r.soft_patch.as_deref()) else {
            };
            let other_rb = other_co.parent().and_then(|h| bodies.get(h));
            let dynamic = other_rb.is_some_and(|rb| rb.is_dynamic());
            let slot = match other_rb {
                Some(rb)
                    if dynamic
                        && rb.ids.active_island_id == island_id as u32
                        && !rb.is_sleeping()
                        && rb.is_enabled() =>
                {
                    rb.ids.active_set_id
                }
                _ => u32::MAX,
            };
            let rigid = RigidSide {
                slot,
                com: patch.com,
                rot: patch.rot,
            };
            let cells = emit_volume_bins(
                params,
                out,
                mesh,
                other_handle,
                (sb, Some(slots)),
                None,
                Some(rigid),
                hard,
                &patch.bins,
                u32::MAX,
            );
            if recovery.overlap_patch_constraints != SoftPatchConstraints::Keep {
                out.rigid_patches
                    .push((other_handle, patch.depth.clone(), cells));
            }
        }
    }
}
