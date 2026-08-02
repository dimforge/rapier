//! Sweep machinery of the continuous-collision pass: fast-collider descriptions,
//! per-pair time-of-impact casts (proxy sweeps + nonlinear fallback),
//! and the per-body continuous solve.

use crate::alloc_prelude::*;
use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::{Collider, ColliderHandle, ColliderSet};
use crate::math::{Pose, Real, Vector};
use crate::parry::bounding_volume::Aabb;
use crate::parry::bounding_volume::BoundingVolume;
use crate::parry::partitioning::Bvh;
use crate::pipeline::{ActiveHooks, PairFilterContext, PhysicsHooks};
#[cfg(feature = "dim2")]
use parry::query::sweep_toi::CORE_FRACTION;
use parry::query::sweep_toi::{
    Sweep, SweepCompositeFastShape, SweepToiStatus, ToiProxy, sweep_time_of_impact,
    sweep_time_of_impact_composite,
};
use parry::query::{NonlinearRigidMotion, QueryDispatcher};
use parry::shape::{Shape, TypedShape};

/// Is `rb` a *fixed* CCD target? A parentless collider (`rb == None`) and a
/// `Fixed` parent body are both fixed. Kinematic and dynamic parents are not.
fn is_fixed_target(rb: Option<&RigidBody>) -> bool {
    rb.map(|b| b.is_fixed()).unwrap_or(true)
}

/// Is `rb` a bullet (a dynamic body with the full-CCD upgrade)? Bullets never sweep against
/// other bullets.
pub(super) fn is_bullet(rb: &RigidBody) -> bool {
    rb.is_dynamic() && rb.ccd.ccd_enabled
}

/// The target-selection rule for a fast body `rb1`: a non-bullet fast body only sweeps
/// against **fixed** targets; a bullet sweeps against every body type except other bullets.
fn tier_allows(rb1: &RigidBody, rb2: Option<&RigidBody>) -> bool {
    if is_bullet(rb1) {
        !rb2.map(is_bullet).unwrap_or(false)
    } else {
        is_fixed_target(rb2)
    }
}

/// Returns `true` if the user's `filter_contact_pair` hook rejected this pair. Mirrors the
/// narrow-phase filter in `NarrowPhase::compute_contacts` so CCD respects the same user-level
/// filtering (issue #754).
#[inline]
#[allow(clippy::too_many_arguments)]
fn pair_filtered_out_by_hooks(
    hooks: &dyn PhysicsHooks,
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    co1: &Collider,
    co2: &Collider,
    ch1: ColliderHandle,
    ch2: ColliderHandle,
    bh1: RigidBodyHandle,
    bh2: Option<RigidBodyHandle>,
) -> bool {
    let active_hooks = co1.flags.active_hooks | co2.flags.active_hooks;
    if !active_hooks.contains(ActiveHooks::FILTER_CONTACT_PAIRS) {
        return false;
    }
    let context = PairFilterContext {
        bodies,
        colliders,
        rigid_body1: Some(bh1),
        rigid_body2: bh2,
        collider1: ch1,
        collider2: ch2,
    };
    hooks.filter_contact_pair(&context).is_none()
}

/// Is this shape one of the composite shapes handled by
/// [`sweep_time_of_impact_composite`]?
fn is_composite_shape(shape: &dyn Shape) -> bool {
    matches!(
        shape.as_typed_shape(),
        TypedShape::TriMesh(_)
            | TypedShape::Polyline(_)
            | TypedShape::HeightField(_)
            | TypedShape::Compound(_)
    )
}

/// Is this a shape the continuous phase never sweeps as the *fast* (moving) shape?
/// Mesh-like bodies rely on speculative contacts only. Also kept out of `ccd_thickness`:
/// a mesh's zero thickness would flag the body fast every step for nothing.
pub(crate) fn shape_never_ccd_swept(shape: &dyn Shape) -> bool {
    matches!(
        shape.as_typed_shape(),
        TypedShape::TriMesh(_)
            | TypedShape::Polyline(_)
            | TypedShape::HeightField(_)
            | TypedShape::Voxels(_)
    )
}

/// The pose a target collider holds during the continuous pass. Targets are stationary at
/// their end-of-step pose (static targets trivially; dynamic/kinematic targets of bullets
/// read the — possibly already clamped — solved `next_position`).
fn target_collider_pose(co: &Collider, rb: Option<&RigidBody>) -> Pose {
    match (rb, co.parent.as_ref()) {
        (Some(rb), Some(parent)) => rb.pos.next_position * parent.pos_wrt_parent,
        _ => co.pos.0,
    }
}

/// Iterates the enabled colliders whose broad-phase (swept) AABB intersects `aabb`. Queries the
/// BVH directly (not via `QueryPipeline`) so the closure stays `Sync` for the parallel per-body
/// sweeps; the CCD sites use the default, pass-everything query filter anyway.
fn intersect_swept_aabb<'a>(
    bvh: &'a Bvh,
    colliders: &'a ColliderSet,
    aabb: Aabb,
) -> impl Iterator<Item = (ColliderHandle, &'a Collider)> + 'a {
    bvh.leaves(move |node| node.aabb().intersects(&aabb))
        .filter_map(move |leaf| {
            // NOTE: do **not** recompute and check the latest collider AABB: checking only
            //       against the one in the BVH is what makes the query conservative (the
            //       leaves hold the swept AABBs).
            let (co, ch) = colliders.get_unknown_gen(leaf)?;
            Some((ch, co))
        })
}

/// The candidate targets a fast body sweeps against. Non-bullets only ever hit **fixed** targets:
/// a flat AABB list skips walking the shared BVH past the dynamic neighbours (costly in dense
/// piles). Bullets and fixed-heavy worlds fall back to the full BVH.
#[derive(Copy, Clone)]
pub(super) enum CcdTargets<'a> {
    /// Flat list of the fixed colliders and their (slightly fattened) AABBs.
    FixedList(&'a [(ColliderHandle, Aabb)]),
    /// The full broad-phase BVH.
    FullBvh(&'a Bvh),
}

/// Maximum number of fixed colliders stored in the flat list before falling back to the
/// full broad-phase BVH.
const FIXED_TARGETS_LIST_MAX: usize = 512;

/// Collects the enabled fixed-target colliders (parentless + attached to `Fixed` bodies),
/// AABBs fattened by the prediction (speculative) distance. Returns `None` when there are
/// more than [`FIXED_TARGETS_LIST_MAX`] of them.
pub(super) fn collect_fixed_targets(
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    prediction_distance: Real,
) -> Option<Vec<(ColliderHandle, Aabb)>> {
    let mut fixed = Vec::new();
    for (ch, co) in colliders.iter_enabled() {
        let rb = co.parent.and_then(|p| bodies.get(p.handle));
        if is_fixed_target(rb) {
            if fixed.len() == FIXED_TARGETS_LIST_MAX {
                return None;
            }
            let aabb = co.shape.compute_aabb(&co.pos).loosened(prediction_distance);
            fixed.push((ch, aabb));
        }
    }
    Some(fixed)
}

/// A convex piece of the fast collider: its point-cloud proxy plus its own sweep (the
/// collider's sweep composed with the piece's local pose for compound children).
struct FastSubShape<'a> {
    proxy: ToiProxy<'a>,
    sweep: Sweep,
    /// The piece's centroid in its sweep's local frame, used for the initial-overlap core
    /// ball.
    local_centroid: Vector,
    /// The piece's smallest extent (inner radius).
    min_extent: Real,
}

/// How the continuous phase sweeps the fast collider's shape.
// Only ever a stack local, built once per fast collider per step: boxing the convex variant
// to even out the sizes would buy an allocation on that path and nothing else.
#[allow(clippy::large_enum_variant)]
enum FastShapeKind<'a> {
    /// A single convex point-cloud shape: the exact proxy sweep.
    Convex(FastSubShape<'a>),
    /// A compound: each point-cloud child sweeps independently, each running its own TOI.
    /// Children without a point-cloud proxy are skipped.
    Compound(Vec<FastSubShape<'a>>),
    /// Cylinders, cones, half-spaces, custom shapes: nonlinear shape-cast fallback.
    Nonlinear,
}

/// Description of the fast collider shared by all its pair casts.
struct FastColliderInfo<'a> {
    collider: &'a Collider,
    body: &'a RigidBody,
    kind: FastShapeKind<'a>,
}

impl<'a> FastColliderInfo<'a> {
    /// Builds the fast-collider description for a sweep from collider pose `start` to `end`, or
    /// `None` for never-swept shapes (meshes, heightfields, polylines, voxels — and compounds with
    /// no point-cloud child). `local_com` makes every sweep rotate about the true center of mass.
    fn new(
        collider: &'a Collider,
        body: &'a RigidBody,
        start: &Pose,
        end: &Pose,
        local_com: Vector,
    ) -> Option<Self> {
        let shape = collider.shape.as_ref();
        if shape_never_ccd_swept(shape) {
            return None;
        }

        let kind = if let Some(proxy) = ToiProxy::from_shape(shape) {
            FastShapeKind::Convex(FastSubShape {
                proxy,
                sweep: Sweep::from_poses(start, end, local_com),
                local_centroid: shape.mass_properties(1.0).local_com,
                min_extent: shape.ccd_thickness(),
            })
        } else if let TypedShape::Compound(compound) = shape.as_typed_shape() {
            let children: Vec<_> = compound
                .shapes()
                .iter()
                .filter_map(|(child_pose, child_shape)| {
                    let proxy = ToiProxy::from_shape(child_shape.as_ref())?;
                    Some(FastSubShape {
                        proxy,
                        sweep: Sweep::from_poses(
                            &(*start * *child_pose),
                            &(*end * *child_pose),
                            child_pose.inverse_transform_point(local_com),
                        ),
                        local_centroid: child_shape.mass_properties(1.0).local_com,
                        min_extent: child_shape.ccd_thickness(),
                    })
                })
                .collect();
            if children.is_empty() {
                return None;
            }
            FastShapeKind::Compound(children)
        } else {
            FastShapeKind::Nonlinear
        };

        Some(Self {
            collider,
            body,
            kind,
        })
    }
}

/// A target collider's shape, classified once per pair before the per-piece casts.
// Stack-local per pair, like [`FastShapeKind`] — indirection would only add an allocation.
#[allow(clippy::large_enum_variant)]
enum TargetKind<'a> {
    /// Point-cloud target: the exact proxy-vs-proxy sweep.
    Proxy(ToiProxy<'a>),
    /// Composite target handled by [`sweep_time_of_impact_composite`].
    Composite,
}

/// A hit against a sensor (or a pair with mismatched solver groups), recorded during the
/// sweep for later intersection-event emission. Never clamps positions.
pub(super) struct PseudoHit {
    pub(super) ch1: ColliderHandle,
    pub(super) ch2: ColliderHandle,
    pub(super) fraction: Real,
}

/// Result of the continuous solve of one fast body.
pub(super) struct BodyContinuousResult {
    pub(super) handle: RigidBodyHandle,
    /// The earliest solid impact fraction in `[0, 1]`; `1.0` if the body sweeps freely.
    pub(super) fraction: Real,
    pub(super) pseudo_hits: Vec<PseudoHit>,
}

/// Casts the fast collider against one target and returns the accepted impact fraction.
/// Solid pairs stop only at strictly >0 fractions (on initial overlap, 2D retries with a core
/// ball while 3D advances); pseudo pairs (sensors, mismatched groups) report any genuine impact.
#[allow(clippy::too_many_arguments)]
fn cast_collider_pair(
    dispatcher: &dyn QueryDispatcher,
    fast: &FastColliderInfo,
    co2: &Collider,
    rb2: Option<&RigidBody>,
    max_fraction: Real,
    dt: Real,
    linear_slop: Real,
    is_pseudo: bool,
) -> Option<Real> {
    let target_pose = target_collider_pose(co2, rb2);

    let sub_shapes: &[FastSubShape] = match &fast.kind {
        FastShapeKind::Convex(sub) => core::slice::from_ref(sub),
        FastShapeKind::Compound(children) => children,
        FastShapeKind::Nonlinear => {
            return fallback_nonlinear_fraction(
                dispatcher,
                fast,
                co2,
                &target_pose,
                max_fraction,
                dt,
                is_pseudo,
            );
        }
    };

    let shape2 = co2.shape.as_ref();
    let target = match ToiProxy::from_shape(shape2) {
        Some(proxy) => TargetKind::Proxy(proxy),
        None if is_composite_shape(shape2) => TargetKind::Composite,
        // Targets with neither a point-cloud proxy nor a supported composite sweep
        // (cylinders, cones, voxels, custom shapes): one whole-collider nonlinear cast.
        None => {
            return fallback_nonlinear_fraction(
                dispatcher,
                fast,
                co2,
                &target_pose,
                max_fraction,
                dt,
                is_pseudo,
            );
        }
    };

    // Each accepted piece fraction tightens `max_fraction`, so the returned value is the
    // earliest impact across all the pieces (both the solid and pseudo accept conditions
    // only pass at or below the current bound).
    let mut best = None;
    let mut max_fraction = max_fraction;
    for sub in sub_shapes {
        if let Some(fraction) = cast_sub_shape(
            sub,
            &target,
            shape2,
            &target_pose,
            max_fraction,
            linear_slop,
            is_pseudo,
        ) {
            best = Some(fraction);
            max_fraction = fraction;
        }
    }
    best
}

/// Casts one convex piece of the fast collider against the target collider's shape and
/// returns the accepted impact fraction (see [`cast_collider_pair`]), or `None`.
fn cast_sub_shape(
    sub: &FastSubShape,
    target: &TargetKind,
    shape2: &dyn Shape,
    target_pose: &Pose,
    max_fraction: Real,
    linear_slop: Real,
    is_pseudo: bool,
) -> Option<Real> {
    let output = match target {
        TargetKind::Proxy(target_proxy) => {
            let target_sweep = Sweep::constant(target_pose, Vector::ZERO);
            sweep_time_of_impact(
                target_proxy,
                &target_sweep,
                &sub.proxy,
                &sub.sweep,
                max_fraction,
                linear_slop,
            )
        }
        TargetKind::Composite => {
            // Heightfields are treated as one-sided in 3D; oriented polylines and
            // oriented meshes enable the one-sided early-outs from their own flags inside
            // the composite dispatch.
            #[cfg(feature = "dim2")]
            let one_sided = false;
            #[cfg(feature = "dim3")]
            let one_sided = matches!(shape2.as_typed_shape(), TypedShape::HeightField(_));

            let fast_desc = SweepCompositeFastShape {
                proxy: &sub.proxy,
                sweep: &sub.sweep,
                local_centroid: sub.local_centroid,
                min_extent: sub.min_extent,
            };
            sweep_time_of_impact_composite(
                shape2,
                target_pose,
                fast_desc,
                one_sided,
                is_pseudo,
                max_fraction,
                linear_slop,
            )?
        }
    };

    if is_pseudo {
        // Any genuine impact counts (initial overlaps are filtered later by the endpoint
        // intersection re-check); `Separated` does not.
        return match output.status {
            SweepToiStatus::Hit | SweepToiStatus::Failed | SweepToiStatus::Overlapped
                if output.fraction <= max_fraction =>
            {
                Some(output.fraction)
            }
            _ => None,
        };
    }

    if 0.0 < output.fraction && output.fraction < max_fraction {
        return Some(output.fraction);
    }

    #[cfg(feature = "dim2")]
    if output.fraction == 0.0 {
        // Fallback to the TOI of a small core circle around the fast shape centroid,
        // so a body already touching a surface isn't pinned at fraction 0.
        if let TargetKind::Proxy(target_proxy) = target {
            let core = ToiProxy::point(sub.local_centroid, CORE_FRACTION * sub.min_extent);
            let target_sweep = Sweep::constant(target_pose, Vector::ZERO);
            let output = sweep_time_of_impact(
                target_proxy,
                &target_sweep,
                &core,
                &sub.sweep,
                max_fraction,
                linear_slop,
            );
            if 0.0 < output.fraction && output.fraction < max_fraction {
                return Some(output.fraction);
            }
        }
    }

    None
}

/// Nonlinear shape-cast fallback for pairs the proxy sweep can't handle (cylinder, cone or
/// custom fast shapes; cylinder, cone, voxels or custom targets). Motion is derived from
/// the solved effective velocities, like rapier's historical CCD.
fn fallback_nonlinear_fraction(
    dispatcher: &dyn QueryDispatcher,
    fast: &FastColliderInfo,
    co2: &Collider,
    target_pose: &Pose,
    max_fraction: Real,
    dt: Real,
    is_pseudo: bool,
) -> Option<Real> {
    if dt == 0.0 {
        return None;
    }

    let rb1 = fast.body;
    let parent1 = fast.collider.parent.as_ref()?;
    let motion1 = NonlinearRigidMotion::new(
        rb1.pos.position,
        rb1.mprops.local_mprops.local_com,
        rb1.ccd_vels.linvel,
        rb1.ccd_vels.angvel,
    )
    .prepend(parent1.pos_wrt_parent);
    let motion2 = NonlinearRigidMotion::constant_position(*target_pose);

    let hit = dispatcher
        .cast_shapes_nonlinear(
            &motion1,
            fast.collider.shape.as_ref(),
            &motion2,
            co2.shape.as_ref(),
            0.0,
            dt,
            is_pseudo, // Sensors stop at the first penetration.
        )
        .ok()??;

    let fraction = hit.time_of_impact / dt;
    if is_pseudo {
        (fraction <= max_fraction).then_some(fraction)
    } else {
        (0.0 < fraction && fraction < max_fraction).then_some(fraction)
    }
}

/// How the per-body sweep treats *pseudo* pairs (a sensor collider, or mismatched solver
/// groups): the continuous pass records them for intersection-event emission; the substep
/// splitter ignores them since they never clamp motion.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(super) enum PseudoHitMode {
    Record,
    Ignore,
}

/// Runs the continuous sweep of a single fast body from its current pose to `end_body_pose`:
/// sweeps each of its colliders through `targets`, keeps the earliest solid impact fraction,
/// and records sensor crossings in [`PseudoHitMode::Record`] mode. Shared by the continuous
/// pass (end pose = solved `next_position`) and the substep splitter (end pose = the
/// forces/velocities integration, pseudo pairs ignored).
#[allow(clippy::too_many_arguments)]
pub(super) fn sweep_fast_body(
    handle: RigidBodyHandle,
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    end_body_pose: Pose,
    targets: CcdTargets,
    dispatcher: &dyn QueryDispatcher,
    hooks: &dyn PhysicsHooks,
    dt: Real,
    linear_slop: Real,
    pseudo_mode: PseudoHitMode,
) -> BodyContinuousResult {
    let rb1 = &bodies[handle];
    let mut fraction: Real = 1.0;
    let mut pseudo_hits = Vec::new();

    for ch1 in &rb1.colliders.0 {
        let co1 = &colliders[*ch1];
        let Some(parent1) = co1.parent.as_ref() else {
            continue;
        };
        if pseudo_mode == PseudoHitMode::Ignore && co1.is_sensor() {
            continue; // Sensors never clamp motion, so they can't affect the earliest impact.
        }

        let start = rb1.pos.position * parent1.pos_wrt_parent;
        let end = end_body_pose * parent1.pos_wrt_parent;
        let local_com = parent1
            .pos_wrt_parent
            .inverse_transform_point(rb1.mprops.local_mprops.local_com);
        let Some(fast) = FastColliderInfo::new(co1, rb1, &start, &end, local_com) else {
            // Never-swept shape (mesh, heightfield, polyline, voxels): no CCD.
            continue;
        };

        // Swept box: union of the collider's AABBs at the start and end poses.
        let start_aabb = co1.shape.compute_aabb(&start);
        let end_aabb = co1.shape.compute_aabb(&end);
        let swept_aabb = start_aabb.merged(&end_aabb);

        let mut handle_candidate = |ch2: ColliderHandle, co2: &Collider| {
            if ch2 == *ch1 {
                return;
            }
            let bh2 = co2.parent.map(|p| p.handle);
            if bh2 == Some(handle) {
                return; // Skip same body.
            }
            let rb2 = bh2.and_then(|h| bodies.get(h));
            if !tier_allows(rb1, rb2) {
                return;
            }
            if !co1.flags.collision_groups.test(co2.flags.collision_groups) {
                return;
            }

            let is_pseudo = co1.is_sensor()
                || co2.is_sensor()
                || !co1.flags.solver_groups.test(co2.flags.solver_groups);
            if is_pseudo && pseudo_mode == PseudoHitMode::Ignore {
                return;
            }
            if pair_filtered_out_by_hooks(
                hooks, bodies, colliders, co1, co2, *ch1, ch2, handle, bh2,
            ) {
                return;
            }

            if let Some(hit_fraction) = cast_collider_pair(
                dispatcher,
                &fast,
                co2,
                rb2,
                fraction,
                dt,
                linear_slop,
                is_pseudo,
            ) {
                if is_pseudo {
                    pseudo_hits.push(PseudoHit {
                        ch1: *ch1,
                        ch2,
                        fraction: hit_fraction,
                    });
                } else {
                    fraction = hit_fraction;
                }
            }
        };

        match targets {
            CcdTargets::FixedList(fixed) => {
                for (ch2, aabb2) in fixed {
                    if aabb2.intersects(&swept_aabb) {
                        handle_candidate(*ch2, &colliders[*ch2]);
                    }
                }
            }
            CcdTargets::FullBvh(bvh) => {
                for (ch2, co2) in intersect_swept_aabb(bvh, colliders, swept_aabb) {
                    handle_candidate(ch2, co2);
                }
            }
        }
    }

    BodyContinuousResult {
        handle,
        fraction,
        pseudo_hits,
    }
}

/// Fake hook that simply detects if hooks are enabled during CCD.
///
/// This lets us know if another pass is needed when the hooks are non-sync.
#[cfg(all(feature = "parallel", feature = "unsync-callbacks"))]
#[derive(Default)]
struct HookProbe(core::sync::atomic::AtomicBool);

#[cfg(all(feature = "parallel", feature = "unsync-callbacks"))]
impl PhysicsHooks for HookProbe {
    fn filter_contact_pair(
        &self,
        _: &crate::pipeline::PairFilterContext,
    ) -> Option<crate::geometry::SolverFlags> {
        self.0.store(true, core::sync::atomic::Ordering::Relaxed);
        // Irrelevant: reaching this discards the whole pass in favour of the serial redo.
        Some(crate::geometry::SolverFlags::COMPUTE_IMPULSES)
    }
}

/// Maps `f` (typically an expensive time-of-impact computation) over the given body handles,
/// in parallel when the `parallel` feature is enabled.
///
/// `f` takes the hooks rather than capturing them, so that a non-`Sync` build can hand the
/// workers a stand-in and keep the user's callback on this thread.
#[cfg(feature = "parallel")]
pub(super) fn map_bodies_parallel<T: Send>(
    handles: &[RigidBodyHandle],
    hooks: &dyn PhysicsHooks,
    f: impl Fn(RigidBodyHandle, &dyn PhysicsHooks) -> T + Sync + Send,
) -> Vec<T> {
    // Below a few dozen bodies the rayon fan-out (pool wake + join) costs more
    // than the sweeps themselves.
    let map_with = |hooks: &(dyn PhysicsHooks + Sync)| {
        if handles.len() >= 64 {
            use rayon::prelude::*;
            return handles.par_iter().map(|h| f(*h, hooks)).collect();
        }
        handles.iter().map(|h| f(*h, hooks)).collect()
    };

    #[cfg(not(feature = "unsync-callbacks"))]
    return map_with(hooks);

    #[cfg(feature = "unsync-callbacks")]
    {
        let probe = HookProbe::default();
        let swept = map_with(&probe);
        if !probe.0.load(core::sync::atomic::Ordering::Relaxed) {
            // No sweep met a hook-flagged pair, so the stand-in answered nothing.
            return swept;
        }
        // One did: redo on this thread with the real hooks. Sweeps only read the body and
        // collider sets and return their result — the caller applies it — so throwing the
        // first pass away is free of side effects.
        drop(swept);
        handles.iter().map(|h| f(*h, hooks)).collect()
    }
}

/// Serial fallback for non-`parallel` builds. Keeps the same shape as the parallel one so
/// both call sites read alike; nothing here needs the hooks to be `Sync`.
#[cfg(not(feature = "parallel"))]
pub(super) fn map_bodies_parallel<T>(
    handles: &[RigidBodyHandle],
    hooks: &dyn PhysicsHooks,
    f: impl Fn(RigidBodyHandle, &dyn PhysicsHooks) -> T,
) -> Vec<T> {
    handles.iter().map(|h| f(*h, hooks)).collect()
}
