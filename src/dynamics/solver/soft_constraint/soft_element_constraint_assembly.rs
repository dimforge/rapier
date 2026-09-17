//! Per-step assembly of the awake soft bodies' element constraints into the group-major,
//! color-by-color layout the solver expects: per-color counts and prefix sums (serial), then each
//! body fills its own slots of every color's range (per body, in parallel under `parallel`).

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;
use core::ops::Range;
use core::sync::atomic::AtomicBool;

use super::soft_constraints_set::{
    AwakeSoftBody, SoftColorRange, SoftConstraintsSet, SoftGroupLayout, SoftShapeConstraint,
    SoftVolumeConstraint,
};
use super::soft_element_constraint::{
    CorotationalConstraint, MAX_CONSTRAINT_PARTICLES, NeoHookeanConstraint, STRAIN_ROWS,
    SoftElasticModel, SoftElasticConstraint, SoftScalarConstraint, SoftScalarConstraintKind,
    SoftScalarConstraintWriteback, StrainMatrix, StrainVector,
};
use crate::dynamics::soft_body::SOFT_BODY_OVERFLOW_COLOR;
use crate::dynamics::solver::reset_buffer_reusing;
use crate::dynamics::{
    RigidBodyHandle, RigidBodySet, SoftBody, SoftBodyCellModel, SoftBodyEdgeKind, SoftBodyHandle,
    SoftBodySet, SpringCoefficients,
};
use crate::math::{AngVector, DIM, Matrix, Real, Vector};
use crate::utils::RotationOps;
use na::SimdRealField;
use super::soft_constraints_set::AwakeCluster;

/// Number of element colors: the parallel ones, then the overflow color.
const NUM_COLORS: usize = SOFT_BODY_OVERFLOW_COLOR as usize + 1;

/// Workspace of the element-constraint assembly kept across steps.
#[derive(Default)]
pub(crate) struct ElementConstraintAssemblyWorkspace {
    /// Substep length of every group.
    group_dts: Vec<Real>,
    /// Per (awake body, color): the body's scalar / elastic constraint count, turned into its first
    /// slot of the color's range by the layout pass.
    offsets: Vec<usize>,
    elastic_offsets: Vec<usize>,
    /// Per awake body: its shape constraint count, then its first shape constraint.
    shape_offsets: Vec<usize>,
}

/// The constraint buffers being filled, as raw pointers shared by the per-body fill jobs.
///
/// Safety: the layout pass gives every body its own slots (the offsets partition the buffers),
/// so a job only writes slots no other job touches.
#[derive(Copy, Clone)]
struct ConstraintSlots {
    scalar_constraints: *mut SoftScalarConstraint,
    elastic_constraints: *mut SoftElasticConstraint,
    shape_constraints: *mut SoftShapeConstraint,
}

// SAFETY: see the struct-level contract.
unsafe impl Send for ConstraintSlots {}
unsafe impl Sync for ConstraintSlots {}

/// The per-body cursors of one fill job.
struct BodyCursors {
    slots: ConstraintSlots,
    scalar_constraints: [usize; NUM_COLORS],
    elastic_constraints: [usize; NUM_COLORS],
    shape_constraints: usize,
}

impl BodyCursors {
    #[inline]
    fn push_constraint(&mut self, color: u8, constraint: SoftScalarConstraint) {
        let slot = &mut self.scalar_constraints[color as usize];
        // SAFETY: the body's own slot (see `ConstraintSlots`).
        unsafe { self.slots.scalar_constraints.add(*slot).write(constraint) };
        *slot += 1;
    }

    #[inline]
    fn push_elastic_constraint(&mut self, color: u8, constraint: SoftElasticConstraint) {
        let slot = &mut self.elastic_constraints[color as usize];
        // SAFETY: the body's own slot (see `ConstraintSlots`).
        unsafe { self.slots.elastic_constraints.add(*slot).write(constraint) };
        *slot += 1;
    }

    #[inline]
    fn push_shape_constraint(&mut self, constraint: SoftShapeConstraint) {
        // SAFETY: the body's own slot (see `ConstraintSlots`).
        unsafe { self.slots.shape_constraints.add(self.shape_constraints).write(constraint) };
        self.shape_constraints += 1;
    }
}

impl SoftConstraintsSet {
    /// Assembles every awake soft body's constraints and lays out their particles' solver slots
    /// (`slot_base` and up, group-major, in awake order); `group_of_slot(slot)` maps a rigid slot
    /// to its substep group, `group_dt(group)` its substep length and `step_dt` the whole step's.
    #[allow(clippy::too_many_arguments)]
    pub fn assemble(
        &mut self,
        island_id: usize,
        slot_base: usize,
        num_groups: usize,
        group_of_slot: impl Fn(u32) -> usize,
        group_dt: impl Fn(usize) -> Real,
        step_dt: Real,
        max_corrective_velocity: Real,
        bodies: &RigidBodySet,
        soft_bodies: &mut SoftBodySet,
    ) {
        self.awake.clear();
        self.slots.clear();
        self.scalar_constraints.clear();
        self.elastic_constraints.clear();
        self.color_ranges.clear();
        self.shape_constraints.clear();
        self.volume_constraints.clear();
        self.volume_grads.clear();
        self.overlap_constraints.clear();
        self.overlap_grads.clear();
        self.overlap_warm_impulses.clear();
        self.overlap_particles.clear();
        self.contacts.clear();
        self.attachments.clear();
        self.groups.clear();

        if soft_bodies.is_empty() {
            return;
        }
        self.collect_awake_bodies(island_id, group_of_slot, step_dt, bodies, soft_bodies);
        if self.awake.is_empty() {
            return;
        }
        // Group-major order (stable, so the handle order is kept within a group), then the
        // particles' solver slots: contiguous per body, bodies in awake order.
        self.awake.sort_by_key(|a| a.group);
        for awake in &mut self.awake {
            awake.substep_dt = group_dt(awake.group as usize);
            awake.slot_start = self.slots.len();
            let first = slot_base + self.slots.len();
            self.slots
                .extend((0..awake.num_particles).map(|i| (first + i) as u32));
        }
        self.groups.resize(num_groups, SoftGroupLayout::default());

        let mut workspace = core::mem::take(&mut self.element_constraint_workspace);
        workspace.group_dts.clear();
        workspace.group_dts.extend((0..num_groups).map(group_dt));
        let num_awake = self.awake.len();
        workspace.offsets.clear();
        workspace.offsets.resize(num_awake * NUM_COLORS, 0);
        workspace.elastic_offsets.clear();
        workspace.elastic_offsets.resize(num_awake * NUM_COLORS, 0);
        workspace.shape_offsets.clear();
        workspace.shape_offsets.resize(num_awake, 0);

        // Pass 1: every body's constraint counts per color (serial: a few percent of the fill, less
        // than a fork-join).
        for ai in 0..num_awake {
            let (counts, elastic_counts) = (
                &mut workspace.offsets[ai * NUM_COLORS..(ai + 1) * NUM_COLORS],
                &mut workspace.elastic_offsets[ai * NUM_COLORS..(ai + 1) * NUM_COLORS],
            );
            // SAFETY: read-only access during assembly.
            let range = 0..element_count(unsafe { &*self.awake[ai].ptr });
            self.count_body_constraints(ai, counts, elastic_counts, range);
            workspace.shape_offsets[ai] = self.count_body_shape_constraints(ai);
        }

        // Pass 2 (serial): the layout. Per group: the awake range, the shape and volume constraints
        // (awake order), then per color the constraint range and each body's first slot in it
        // (awake order, so a group's constraints of a color are contiguous and deterministic).
        let mut awake_cursor = 0;
        let mut cursor = 0;
        let mut elastic_cursor = 0;
        let mut shape_cursor = 0;
        for gi in 0..num_groups {
            let awake_start = awake_cursor;
            while awake_cursor < self.awake.len() && self.awake[awake_cursor].group as usize == gi {
                awake_cursor += 1;
            }
            let dt = workspace.group_dts[gi];
            let shape_start = shape_cursor;
            let volume_start = self.volume_constraints.len();
            let mut damping = false;
            for ai in awake_start..awake_cursor {
                let awake = &mut self.awake[ai];
                let shape_count = workspace.shape_offsets[ai];
                workspace.shape_offsets[ai] = shape_cursor;
                awake.shape_constraints = shape_cursor..shape_cursor + shape_count;
                shape_cursor += shape_count;
                if awake.frozen {
                    continue;
                }
                // SAFETY: read-only access during assembly.
                let sb = unsafe { &*awake.ptr };
                let material = &sb.material;
                if material.deformation_damping > 0.0 {
                    awake.damping_factor = 1.0 - (-material.deformation_damping * dt).exp();
                    damping = true;
                }
                if sb.volume_preservation && !sb.volume_pieces.is_empty() {
                    let grads_start = self.volume_grads.len();
                    self.volume_grads
                        .resize(grads_start + sb.particles.len(), Vector::ZERO);
                    let first = self.volume_constraints.len();
                    for (pi, piece) in sb.volume_pieces.iter().enumerate() {
                        self.volume_constraints.push(SoftVolumeConstraint {
                            soft_body: ai as u32,
                            piece: pi as u32,
                            target: piece.rest_volume * sb.volume_factor,
                            grads: grads_start..grads_start + sb.particles.len(),
                            erp_inv_dt: material.volume_softness.erp_inv_dt(dt),
                            cfm_coeff: material.volume_softness.cfm_coeff(dt),
                            max_bias_velocity: max_corrective_velocity,
                            inv_lhs: 0.0,
                            cfm_gain: 0.0,
                            rhs: 0.0,
                            impulse: piece.impulse,
                            fem: None,
                        });
                    }
                    awake.volume_constraints = first..self.volume_constraints.len();
                }
            }
            let colors_start = self.color_ranges.len();
            for color in 0..NUM_COLORS {
                let start = cursor;
                let elastic_start = elastic_cursor;
                for ai in awake_start..awake_cursor {
                    let count = &mut workspace.offsets[ai * NUM_COLORS + color];
                    let elastic_count = &mut workspace.elastic_offsets[ai * NUM_COLORS + color];
                    (cursor, *count) = (cursor + *count, cursor);
                    (elastic_cursor, *elastic_count) =
                        (elastic_cursor + *elastic_count, elastic_cursor);
                }
                let range = SoftColorRange {
                    scalar_constraints: start..cursor,
                    elastic_constraints: elastic_start..elastic_cursor,
                };
                if color == SOFT_BODY_OVERFLOW_COLOR as usize {
                    self.groups[gi].serial = range;
                } else if range.len() > 0 {
                    self.color_ranges.push(range);
                }
            }
            let g = &mut self.groups[gi];
            g.awake = awake_start..awake_cursor;
            g.slots = match (self.awake.get(awake_start), awake_cursor.checked_sub(1)) {
                (Some(first), Some(last)) if awake_start < awake_cursor => {
                    let last = &self.awake[last];
                    slot_base + first.slot_start..slot_base + last.slot_start + last.num_particles
                }
                _ => slot_base..slot_base,
            };
            g.colors = colors_start..self.color_ranges.len();
            g.shape_constraints = shape_start..shape_cursor;
            // Serial as well when a FEM body has shape constraints: each writes all of its particles.
            g.shape_serial = self.awake[awake_start..awake_cursor].iter().any(|a| {
                // SAFETY: read-only access during assembly.
                !a.awake_clusters.is_empty() || unsafe { &*a.ptr }.uses_fem()
            });
            g.volume_constraints = volume_start..self.volume_constraints.len();
            g.damping = damping;
        }

        // Pass 3: every body builds its constraints straight into its slots.
        // SAFETY: plain-old-data constraints; every slot below the new lengths is written by the fill
        // before being read (the counts of pass 1 are exact).
        unsafe {
            reset_buffer_reusing(&mut self.scalar_constraints, cursor);
            reset_buffer_reusing(&mut self.elastic_constraints, elastic_cursor);
            self.strained.clear();
            self.strained.resize(cursor + elastic_cursor, false);
            reset_buffer_reusing(&mut self.shape_constraints, shape_cursor);
        }
        {
            let slots = ConstraintSlots {
                scalar_constraints: self.scalar_constraints.as_mut_ptr(),
                elastic_constraints: self.elastic_constraints.as_mut_ptr(),
                shape_constraints: self.shape_constraints.as_mut_ptr(),
            };
            let this = &*self;
            let workspace = &workspace;
            let fill = |ai: usize| {
                let mut cursors = BodyCursors {
                    slots,
                    scalar_constraints: [0; NUM_COLORS],
                    elastic_constraints: [0; NUM_COLORS],
                    shape_constraints: workspace.shape_offsets[ai],
                };
                cursors
                    .scalar_constraints
                    .copy_from_slice(&workspace.offsets[ai * NUM_COLORS..(ai + 1) * NUM_COLORS]);
                cursors.elastic_constraints.copy_from_slice(
                    &workspace.elastic_offsets[ai * NUM_COLORS..(ai + 1) * NUM_COLORS],
                );
                let dt = workspace.group_dts[this.awake[ai].group as usize];
                // SAFETY: read-only access during assembly.
                let num_elements = element_count(unsafe { &*this.awake[ai].ptr });
                #[cfg(feature = "parallel")]
                if num_elements >= 2 * FILL_CHUNK && rayon::current_num_threads() > 1 {
                    // A large body fills its elements in chunks across threads: each chunk counts
                    // its constraints per color, takes cursors from the prefix sums (slot order
                    // stays the element order, as serially), then fills them.
                    use rayon::prelude::*;
                    let num_chunks = num_elements.div_ceil(FILL_CHUNK);
                    let chunk_range =
                        |c: usize| c * FILL_CHUNK..((c + 1) * FILL_CHUNK).min(num_elements);
                    let mut chunk_counts: Vec<([usize; NUM_COLORS], [usize; NUM_COLORS])> =
                        (0..num_chunks)
                            .into_par_iter()
                            .map(|c| {
                                let mut counts = ([0; NUM_COLORS], [0; NUM_COLORS]);
                                this.count_body_constraints(
                                    ai,
                                    &mut counts.0,
                                    &mut counts.1,
                                    chunk_range(c),
                                );
                                counts
                            })
                            .collect();
                    for (scalar, elastic) in chunk_counts.iter_mut() {
                        for color in 0..NUM_COLORS {
                            let (count, elastic_count) = (scalar[color], elastic[color]);
                            scalar[color] = cursors.scalar_constraints[color];
                            elastic[color] = cursors.elastic_constraints[color];
                            cursors.scalar_constraints[color] += count;
                            cursors.elastic_constraints[color] += elastic_count;
                        }
                    }
                    chunk_counts
                        .into_par_iter()
                        .enumerate()
                        .for_each(|(c, (scalar, elastic))| {
                            let mut cursors = BodyCursors {
                                slots,
                                scalar_constraints: scalar,
                                elastic_constraints: elastic,
                                shape_constraints: usize::MAX,
                            };
                            this.fill_body_constraints(
                                ai,
                                dt,
                                max_corrective_velocity,
                                &mut cursors,
                                chunk_range(c),
                            );
                        });
                    this.fill_body_shape_constraints(ai, dt, &mut cursors);
                    debug_assert_eq!(
                        cursors.shape_constraints,
                        this.awake[ai].shape_constraints.end
                    );
                    return;
                }
                this.fill_body_constraints(
                    ai,
                    dt,
                    max_corrective_velocity,
                    &mut cursors,
                    0..num_elements,
                );
                this.fill_body_shape_constraints(ai, dt, &mut cursors);
                debug_assert_eq!(cursors.shape_constraints, this.awake[ai].shape_constraints.end);
            };
            #[cfg(feature = "parallel")]
            {
                use rayon::prelude::*;
                (0..num_awake).into_par_iter().for_each(fill);
            }
            #[cfg(not(feature = "parallel"))]
            (0..num_awake).for_each(fill);
        }
        self.element_constraint_workspace = workspace;

        // Every parallel color must be particle-disjoint: two constraints of a color sharing a solver
        // body would race on its velocity.
        #[cfg(debug_assertions)]
        for range in &self.color_ranges {
            let mut seen: parry::utils::hashmap::HashMap<u32, ()> = Default::default();
            let ids = self.scalar_constraints[range.scalar_constraints.clone()]
                .iter()
                .flat_map(|c| c.solver_ids[..c.num_particles as usize].iter().copied())
                .chain(
                    self.elastic_constraints[range.elastic_constraints.clone()]
                        .iter()
                        .flat_map(|constraint| constraint.solver_ids.iter().copied()),
                );
            for id in ids {
                if id != u32::MAX {
                    debug_assert!(
                        seen.insert(id, ()).is_none(),
                        "soft-body solver slot {id} shared by two constraints of one color"
                    );
                }
            }
        }
    }

    /// Fills `awake` (in handle order) with the soft bodies whose root body is awake in island
    /// `island_id`. A fully pinned body is frozen: no constraint of its own, but its particles
    /// still move kinematically and its surface holds awake bodies; the caller lays out slots.
    fn collect_awake_bodies(
        &mut self,
        island_id: usize,
        group_of_slot: impl Fn(u32) -> usize,
        step_dt: Real,
        bodies: &RigidBodySet,
        soft_bodies: &mut SoftBodySet,
    ) {
        let inv_dt = crate::utils::inv(step_dt);

        // The root body's slot in the island, if awake in it.
        let root_slot = |h: RigidBodyHandle| -> Option<u32> {
            let rb = bodies.get(h)?;
            (rb.ids.active_island_id == island_id as u32 && !rb.is_sleeping() && rb.is_enabled())
                .then_some(rb.ids.active_set_id)
        };
        let mut selected: Vec<(SoftBodyHandle, u16, bool)> = Vec::new();
        for (handle, sb) in soft_bodies.iter() {
            let Some(slot) = root_slot(sb.root_body()) else {
                // Asleep or not simulated.
                continue;
            };
            let frozen = !sb.particles.iter().any(|p| p.inv_mass > 0.0);
            selected.push((handle, group_of_slot(slot) as u16, frozen));
        }
        for (handle, group, frozen) in selected {
            let Some(sb) = soft_bodies.get_mut(handle) else {
                continue;
            };
            // A body deserialized from a snapshot predating the volume pieces has none yet.
            if sb.volume_preservation && sb.volume_pieces.is_empty() {
                sb.rebuild_volume_pieces(&[]);
            }

            let awake_clusters =
            sb
                .clusters
                .iter()
                .enumerate()
                .filter(|(_, c)| c.is_live() && c.shape_matching)
                .map(|(ci, c)| {
                    let mut target_linvel = Default::default();
                    let mut target_angvel = Default::default();

                    if let (Some(prev_target_pose), Some(target_pose)) = (&c.prev_shape_matching_target, &c.shape_matching_target) {
                        let dpos = target_pose.translation - prev_target_pose.translation;
                        let drot = target_pose.rotation * prev_target_pose.rotation.inverse();
                        target_linvel = dpos * inv_dt;
                        #[cfg(feature = "dim2")]
                        { target_angvel = drot.angle() * inv_dt; }
                        #[cfg(feature = "dim3")]
                        { target_angvel = drot.to_scaled_axis() * inv_dt; }
                    }

                    AwakeCluster {
                        cluster: ci as u32,
                        rotation: c.rotation,
                        target_linvel,
                        target_angvel,
                        com: Vector::ZERO,
                        rest_com: Vector::ZERO,
                        dyn_com: Vector::ZERO,
                        inv_inertia: zero_inertia(),
                    }
                })
                .collect();

            self.awake.push(AwakeSoftBody {
                ptr: sb as *mut SoftBody,
                handle,
                group,
                substep_dt: step_dt,
                frozen,
                slot_start: 0,
                num_particles: sb.particles.len(),
                shape_com: Vector::ZERO,
                shape_constraints: 0..0,
                volume_constraints: 0..0,
                damping_factor: 0.0,
                plastic_flow: AtomicBool::new(false),
                torn: AtomicBool::new(false),
                contact_approach_speed: None,
                awake_clusters,
                #[cfg(feature = "fem")]
                fem: None,
            });
        }
    }

    /// Counts the constraints of awake body `ai` per color (scalar constraints, elastic block
    /// constraints) and its shape-matching constraints: the exact numbers
    /// `fill_body_constraints` emits.
    fn count_body_constraints(
        &self,
        ai: usize,
        counts: &mut [usize],
        elastic_counts: &mut [usize],
        range: Range<usize>,
    ) {
        let awake = &self.awake[ai];
        if awake.frozen {
            // No solver DOF: no element, shape or volume constraint, nothing to prepare.
            return;
        }
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let (edges, dihedrals, cells) = element_ranges(sb, range);
        #[cfg(feature = "dim2")]
        let _ = dihedrals;
        // The FEM solver integrates the distance and bending elements itself; they get no constraint.
        if !sb.uses_fem() {
            for e in &sb.edges[edges] {
                counts[e.color as usize] += 1;
            }
            #[cfg(feature = "dim3")]
            for d in &sb.dihedrals[dihedrals] {
                counts[d.color as usize] += 1;
            }
        }
        let (mu, _) = sb.material.lame_parameters();
        for c in &sb.cells[cells] {
            match sb.cell_model {
                // The FEM solver integrates the volume cells itself; they get no constraint.
                SoftBodyCellModel::Volume if sb.uses_fem() => {}
                SoftBodyCellModel::Volume => counts[c.color as usize] += 1,
                SoftBodyCellModel::Corotational | SoftBodyCellModel::NeoHookean => {
                    // The FEM solver integrates the elastic cells itself; they get no constraint.
                    if !sb.uses_fem()
                        && mu * c.stiffness_scale > 0.0
                        && elastic_cell_terms(sb, slots, c).2 > 0.0
                    {
                        elastic_counts[c.color as usize] += 1;
                    }
                }
            }
        }
    }

    /// The number of shape-matching constraints of awake body `ai`: one per free, slotted
    /// particle of each shape-matched cluster (a particle in several such clusters gets one
    /// constraint per cluster).
    fn count_body_shape_constraints(&self, ai: usize) -> usize {
        let awake = &self.awake[ai];
        if awake.frozen {
            return 0;
        }
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let mut count = 0;
        for cluster in &sb.clusters {
            if cluster.is_live() && cluster.shape_matching {
                count += cluster
                    .particles()
                    .iter()
                    .filter(|&&v| {
                        slots[v as usize] != u32::MAX && sb.particles[v as usize].inv_mass != 0.0
                    })
                    .count();
            }
        }
        count
    }

    /// Builds awake body `ai`'s element constraints (substep length `dt`) into its slots: distance,
    /// dihedral and cell constraints in element order, restricted to `range` of its element index
    /// space (see [`element_ranges`]). Reads only the body's own soft body and the shared `slots`.
    fn fill_body_constraints(
        &self,
        ai: usize,
        dt: Real,
        max_corrective_velocity: Real,
        out: &mut BodyCursors,
        range: Range<usize>,
    ) {
        let awake = &self.awake[ai];
        if awake.frozen {
            return;
        }
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let (edges, dihedrals, cells) = element_ranges(sb, range);
        #[cfg(feature = "dim2")]
        let _ = dihedrals;
        let material = &sb.material;
        let coeffs_of = |c: &SpringCoefficients<Real>| (c.erp_inv_dt(dt), c.cfm_coeff(dt));
        let (edge_erp, edge_cfm) = coeffs_of(&material.edge_softness);
        let (bend_erp, bend_cfm) = coeffs_of(&material.bend_softness);
        let (vol_erp, vol_cfm) = coeffs_of(&material.volume_softness);
        let (mu, lambda) = material.lame_parameters();
        let elastic_zeta = material.elastic_damping_ratio;
        let im_of = |i: u32| -> Real {
            let s = slots[i as usize];
            if s == u32::MAX {
                0.0
            } else {
                sb.particles[i as usize].inv_mass
            }
        };
        let pos_of = |i: u32| sb.particles[i as usize].position;
        let base_constraint = |kind: SoftScalarConstraintKind,
                        writeback: SoftScalarConstraintWriteback,
                        element: usize,
                        vertices: &[u32],
                        rest: Real,
                        erp: Real,
                        cfm: Real,
                        impulse: Real|
         -> SoftScalarConstraint {
            let mut solver_ids = [u32::MAX; MAX_CONSTRAINT_PARTICLES];
            let mut pos = [Vector::ZERO; MAX_CONSTRAINT_PARTICLES];
            let mut im = [0.0; MAX_CONSTRAINT_PARTICLES];
            for (k, &v) in vertices.iter().enumerate() {
                solver_ids[k] = slots[v as usize];
                pos[k] = pos_of(v);
                im[k] = im_of(v);
            }
            SoftScalarConstraint {
                kind,
                num_particles: vertices.len() as u8,
                writeback,
                soft_body: ai as u32,
                element: element as u32,
                solver_ids,
                pos,
                im,
                grad: [Vector::ZERO; MAX_CONSTRAINT_PARTICLES],
                rest,
                erp_inv_dt: erp,
                cfm_coeff: cfm,
                inv_lhs: 0.0,
                cfm_gain: 0.0,
                rhs: 0.0,
                impulse,
                peak_impulse: 0.0,
                impulse_bounds: [-Real::MAX, Real::MAX],
            }
        };

        let element_constraints = !sb.uses_fem();
        for (ei, e) in sb
            .edges
            .iter()
            .enumerate()
            .take(edges.end)
            .skip(edges.start)
            .filter(|_| element_constraints)
        {
            let (erp, cfm) = match (e.softness, e.kind) {
                (Some(softness), _) => coeffs_of(&softness),
                (None, SoftBodyEdgeKind::Structural) => (edge_erp, edge_cfm),
                (None, SoftBodyEdgeKind::Bend) => (bend_erp, bend_cfm),
            };
            let mut constraint = base_constraint(
                SoftScalarConstraintKind::Distance,
                SoftScalarConstraintWriteback::Edge,
                ei,
                &e.vertices,
                e.rest_length,
                erp,
                cfm,
                e.impulse,
            );
            if e.tension_only {
                constraint.impulse_bounds = [0.0, Real::MAX];
                constraint.impulse = constraint.impulse.max(0.0);
            }
            out.push_constraint(e.color, constraint);
        }

        #[cfg(feature = "dim3")]
        for (di, d) in sb
            .dihedrals
            .iter()
            .enumerate()
            .take(dihedrals.end)
            .skip(dihedrals.start)
            .filter(|_| element_constraints)
        {
            let constraint = base_constraint(
                SoftScalarConstraintKind::Dihedral,
                SoftScalarConstraintWriteback::Dihedral,
                di,
                &d.vertices,
                d.rest_angle,
                bend_erp,
                bend_cfm,
                d.impulse,
            );
            out.push_constraint(d.color, constraint);
        }

        for (ci, c) in sb.cells.iter().enumerate().take(cells.end).skip(cells.start) {
            match sb.cell_model {
                // Integrated implicitly by the FEM solver instead (see `soft_fem`).
                SoftBodyCellModel::Volume if sb.uses_fem() => continue,
                SoftBodyCellModel::Volume => {
                    let constraint = base_constraint(
                        SoftScalarConstraintKind::CellVolume,
                        SoftScalarConstraintWriteback::CellVolume,
                        ci,
                        &c.vertices,
                        c.rest_volume,
                        vol_erp,
                        vol_cfm,
                        c.impulses[0],
                    );
                    out.push_constraint(c.color, constraint);
                }
                SoftBodyCellModel::Corotational | SoftBodyCellModel::NeoHookean => {
                    if sb.uses_fem() {
                        // Integrated implicitly by the FEM solver instead (see `soft_fem`).
                        continue;
                    }
                    let neo_hookean = sb.cell_model == SoftBodyCellModel::NeoHookean;
                    // Constraint-space stiffnesses 2μV₀ / 4μV₀ (diagonal / shear rows) and λV₀
                    // (volumetric, at rest) map to natural frequencies via the effective masses,
                    // ω² = k · Σ w |∇C|²; Neo-Hookean damping/snap-back caps use (2μ + λ)V₀ / 4μV₀.
                    let vol = c.rest_volume.abs();
                    let (coeffs, im, w_vol) = elastic_cell_terms(sb, slots, c);
                    // Per-cell stiffness scale (regional materials through the clusters).
                    let (mu, lambda) = (mu * c.stiffness_scale, lambda * c.stiffness_scale);
                    if w_vol <= 0.0 || mu <= 0.0 {
                        // Every particle pinned, a degenerate cell, or no material.
                        continue;
                    }
                    let mut solver_ids = [u32::MAX; MAX_CONSTRAINT_PARTICLES];
                    let mut pos = [Vector::ZERO; MAX_CONSTRAINT_PARTICLES];
                    for (k, &v) in c.vertices.iter().enumerate() {
                        solver_ids[k] = slots[v as usize];
                        pos[k] = pos_of(v);
                    }
                    let block = SoftElasticConstraint::strain_block(&coeffs, &im);
                    let two_pi = Real::simd_two_pi();
                    // Snap-back caps: a strain error moves the particles at about
                    // `erp * strain * cell size` per second.
                    let cell_size = vol.powf(1.0 / DIM as Real).max(1.0e-6);
                    let strain_cap = |erp: Real| {
                        if erp > 0.0 {
                            max_corrective_velocity / (erp * cell_size)
                        } else {
                            Real::MAX
                        }
                    };
                    let mut erp_strain = StrainVector::zeros();
                    let mut cfm_strain = StrainVector::zeros();
                    let mut softened = block;
                    let mut inert = [false; STRAIN_ROWS];
                    let mut damping = StrainVector::zeros();
                    let mut strain_per_impulse = StrainVector::zeros();
                    for r in 0..STRAIN_ROWS {
                        let w = block[(r, r)];
                        // The row's deviatoric stiffness: the stress-based strain of both models
                        // reads through it (an isotropic material's deviatoric response is `2μ`).
                        let k_dev = if r < DIM { 2.0 * mu * vol } else { 4.0 * mu * vol };
                        let k = if neo_hookean {
                            NeoHookeanConstraint::rest_stiffness(mu, lambda, r) * vol
                        } else {
                            k_dev
                        };
                        let omega = (k * w).sqrt();
                        if omega > 0.0 {
                            strain_per_impulse[r] = 1.0 / (k_dev * dt);
                            let (erp, cfm) =
                                coeffs_of(&SpringCoefficients::new(omega / two_pi, elastic_zeta));
                            erp_strain[r] = erp;
                            cfm_strain[r] = w * cfm;
                            softened[(r, r)] += cfm_strain[r];
                            // Damping c = 2ζω/w (the constraint as a spring of mass 1/w).
                            damping[r] = 2.0 * elastic_zeta * omega / w;
                        } else {
                            inert[r] = true;
                        }
                    }
                    let strain_caps = StrainVector::from_fn(|r, _| strain_cap(erp_strain[r]));
                    let strain_impulse = StrainVector::from_fn(|r, _| c.impulses[r]);
                    let (inv_a, model, vol_impulse) = if neo_hookean {
                        let model = SoftElasticModel::NeoHookean(NeoHookeanConstraint {
                            dt,
                            mu: mu * vol,
                            lambda: (lambda + mu) * vol,
                            damping,
                            stiffness_strain: StrainVector::repeat(Real::MAX),
                            m: StrainMatrix::zeros(),
                            dt_grad: StrainVector::zeros(),
                        });
                        (StrainMatrix::zeros(), model, 0.0)
                    } else {
                        let omega_vol = (lambda * vol * w_vol).sqrt();
                        // A zero Poisson ratio disables the volumetric row (zero gain
                        // and rhs: its impulse stays zero).
                        let (erp_vol, cfm_coeff_vol) = if omega_vol > 0.0 {
                            coeffs_of(&SpringCoefficients::new(omega_vol / two_pi, elastic_zeta))
                        } else {
                            (0.0, 0.0)
                        };
                        let inv_a = SoftElasticConstraint::invert_block(softened, inert);
                        let model = SoftElasticModel::Corotational(CorotationalConstraint {
                            erp_strain,
                            cfm_strain,
                            erp_vol,
                            cfm_coeff_vol,
                            volumetric: omega_vol > 0.0,
                            vol_cap: strain_cap(erp_vol),
                            grad_vol: [Vector::ZERO; MAX_CONSTRAINT_PARTICLES],
                            cfm_gain_vol: 0.0,
                            rhs_vol: 0.0,
                            b: StrainVector::zeros(),
                            z: StrainVector::zeros(),
                            inv_s: 0.0,
                        });
                        (inv_a, model, c.impulses[STRAIN_ROWS])
                    };
                    let constraint = SoftElasticConstraint {
                        soft_body: ai as u32,
                        element: ci as u32,
                        solver_ids,
                        pos,
                        im,
                        coeffs,
                        inv_rest_matrix: c.inv_rest_matrix,
                        rotation: c.rotation,
                        rot_mat: c.rotation.to_mat(),
                        inv_a,
                        strain_cap: strain_caps,
                        strain: StrainVector::zeros(),
                        inverted: false,
                        strain_impulse,
                        strain_per_impulse,
                        vol_impulse,
                        model,
                    };
                    out.push_elastic_constraint(c.color, constraint);
                }
            }
        }

    }

    /// Builds the shape-matching constraints of awake body `ai` into its slots (see
    /// [`Self::count_body_shape_constraints`]).
    fn fill_body_shape_constraints(&self, ai: usize, dt: Real, out: &mut BodyCursors) {
        let awake = &self.awake[ai];
        if awake.frozen {
            return;
        }
        // SAFETY: read-only access during assembly.
        let sb = unsafe { &*awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let coeffs_of = |c: &SpringCoefficients<Real>| (c.erp_inv_dt(dt), c.cfm_coeff(dt));
        let (shape_erp, shape_cfm) = coeffs_of(&sb.material.shape_matching_softness);
        // Per-cluster shape-matching constraints, in cluster order (warm-started from the cluster's
        // own impulse store).
        for (ci, cluster) in sb.clusters.iter().enumerate() {
            if !cluster.is_live() || !cluster.shape_matching {
                continue;
            }
            for (k, &v) in cluster.particles().iter().enumerate() {
                let pi = v as usize;
                let p = &sb.particles[pi];
                let s = slots[pi];
                if s == u32::MAX || p.inv_mass == 0.0 {
                    continue;
                }
                out.push_shape_constraint(SoftShapeConstraint {
                    particle: v,
                    cluster: ci as u32,
                    solver_id: s,
                    im: Vector::splat(p.inv_mass),
                    goal: p.position,
                    goal_vel: Vector::ZERO,
                    erp_inv_dt: shape_erp,
                    cfm_coeff: shape_cfm,
                    inv_lhs: Vector::ZERO,
                    cfm_gain: Vector::ZERO,
                    rhs: Vector::ZERO,
                    impulse: cluster.shape_impulses.get(k).copied().unwrap_or(Vector::ZERO),
                    fem: None,
                    inv_lhs_block: Matrix::ZERO,
                    cfm_gain_block: Matrix::ZERO,
                });
            }
        }
    }
}

/// Elements per chunk of a large body's parallel fill.
#[cfg(feature = "parallel")]
const FILL_CHUNK: usize = 2048;

/// The size of a body's element index space: its edges, then its dihedrals (3D), then its cells.
fn element_count(sb: &SoftBody) -> usize {
    #[cfg(feature = "dim3")]
    let dihedrals = sb.dihedrals.len();
    #[cfg(feature = "dim2")]
    let dihedrals = 0;
    sb.edges.len() + dihedrals + sb.cells.len()
}

/// The edge, dihedral and cell index ranges covered by `range` of a body's element index space.
fn element_ranges(
    sb: &SoftBody,
    range: Range<usize>,
) -> (Range<usize>, Range<usize>, Range<usize>) {
    let sub = |start: usize, len: usize| -> Range<usize> {
        range.start.saturating_sub(start).min(len)..range.end.saturating_sub(start).min(len)
    };
    let edges = sb.edges.len();
    #[cfg(feature = "dim3")]
    let dihedrals = sb.dihedrals.len();
    #[cfg(feature = "dim2")]
    let dihedrals = 0;
    (
        sub(0, edges),
        sub(edges, dihedrals),
        sub(edges + dihedrals, sb.cells.len()),
    )
}

/// An elastic cell's strain coefficients, its particles' inverse masses (zero without a solver
/// slot) and its rest volumetric weight `Σ_p w_p |∇_p C_V|²` (at rest `∇C_V` of particle `p` is
/// `Σ_j coeffs[p][j] e_j`); a zero weight (all pinned, degenerate cell) means no constraint.
#[inline]
fn elastic_cell_terms(
    sb: &SoftBody,
    slots: &[u32],
    c: &crate::dynamics::SoftBodyCell,
) -> ([Vector; MAX_CONSTRAINT_PARTICLES], [Real; MAX_CONSTRAINT_PARTICLES], Real) {
    let coeffs = SoftElasticConstraint::coefficients(&c.inv_rest_matrix);
    let mut im = [0.0; MAX_CONSTRAINT_PARTICLES];
    for (k, &v) in c.vertices.iter().enumerate() {
        if slots[v as usize] != u32::MAX {
            im[k] = sb.particles[v as usize].inv_mass;
        }
    }
    let mut w_vol = 0.0;
    for (w, c) in im.iter().zip(coeffs.iter()) {
        w_vol += w * c.length_squared();
    }
    (coeffs, im, w_vol)
}

/// A zero angular inertia (the cluster fit's initial value).
fn zero_inertia() -> crate::math::AngularInertia {
    #[cfg(feature = "dim2")]
    {
        0.0
    }
    #[cfg(feature = "dim3")]
    {
        parry::utils::SdpMatrix3::zero()
    }
}
