//! The `Conversion` driver. Walks the [`Model`] tree once, building bodies,
//! joints, equality constraints, sensors, and actuators into the contained
//! [`MjcfRobot`]. Sub-areas (geom shape building, mass derivation, serial
//! joint construction) live in sibling modules and add `impl Conversion`
//! blocks of their own.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use mjcf_rs::Pose as MPose;
use mjcf_rs::body as mb;
use mjcf_rs::equality::{Equality as MjcfEquality, EqualityConnect, EqualityWeld};
use mjcf_rs::equality::EqualityJoint as MjcfEqualityJointDef;
use mjcf_rs::extras::Sensor as MjcfSensor;
use mjcf_rs::glam::DVec3;
use mjcf_rs::model::{BodyEntry, BodyId, Model};

use rapier3d::dynamics::{
    GenericJointBuilder, JointAxesMask, MassProperties, RigidBodyBuilder, RigidBodyType,
};
use rapier3d::geometry::Collider;
use rapier3d::math::{Pose, Real, Vector};

use super::mass::scale_mass_properties;
use super::options::MjcfLoaderOptions;
use super::types::{
    MjcfActuatorBinding, MjcfBody, MjcfEqualityJoint, MjcfJoint, MjcfJointCoupling, MjcfRobot,
    MjcfSensorBinding, MjcfVisualMesh, SensorObjectRef,
};

pub(super) struct Conversion<'a> {
    pub(super) model: &'a Model,
    pub(super) options: &'a MjcfLoaderOptions,
    #[allow(dead_code)]
    pub(super) base_dir: &'a Path,
    pub(super) robot: MjcfRobot,
    /// Maps MJCF body id → rapier body index of the leaf entry (i.e. the
    /// final body, not an intermediate).
    pub(super) leaf_of_mjcf_body: Vec<Option<usize>>,
    /// World pose of each rapier body, used to compute joint frames.
    pub(super) world_pose_of_rapier_body: Vec<MPose>,
    /// MJCF body id of every rapier body.
    pub(super) rapier_to_mjcf_body: Vec<BodyId>,
    /// `true` when the rapier body should be marked as fixed at the end
    /// because the MJCF root body was fixed (no `<freejoint>`).
    pub(super) is_root_body: Vec<bool>,
    /// MJCF id of the bodies that ended up rooted to the world without any
    /// joint (so they should be fixed unconditionally).
    pub(super) welded_to_world: HashSet<BodyId>,
    /// Workspace for collider build; cleared every body.
    pub(super) staged_colliders: Vec<Collider>,
    /// Names of the staged colliders (parallel to `staged_colliders`).
    pub(super) staged_collider_names: Vec<Option<String>>,
    /// Workspace for visual-mesh build (visual-only geoms when
    /// `create_colliders_from_visual_shapes` is `false`); cleared every body.
    pub(super) staged_visual_meshes: Vec<MjcfVisualMesh>,
    /// Per rapier body, the collider names in insertion order. Used to
    /// build the `geom_name_to_collider` map after the fact.
    pub(super) body_collider_names: Vec<Vec<Option<String>>>,
    /// Per rapier body, the mass we explicitly assigned via
    /// `additional_mass_properties` (0 for bodies without one). Used by
    /// `settotalmass` post-processing.
    pub(super) body_masses: Vec<Real>,
    /// Per rapier body, the original derived mass properties (before
    /// settotalmass rescale). Sparse — only present for bodies with
    /// non-zero mass.
    pub(super) body_mass_props: HashMap<usize, MassProperties>,
}

impl<'a> Conversion<'a> {
    pub(super) fn new(
        model: &'a Model,
        options: &'a MjcfLoaderOptions,
        base_dir: &'a Path,
    ) -> Self {
        let mut robot = MjcfRobot {
            name: model.name.clone(),
            ..Default::default()
        };
        // Reserve slot 0 for the world.
        robot.bodies.push(MjcfBody {
            name: None,
            is_intermediate: false,
            mjcf_body_id: 0,
            body: RigidBodyBuilder::fixed().build(),
            colliders: Vec::new(),
            gravcomp: 0.0,
            mass: 0.0,
            visual_meshes: Vec::new(),
        });
        robot.gravity = Vector::new(
            model.option.gravity[0] as Real,
            model.option.gravity[1] as Real,
            model.option.gravity[2] as Real,
        );

        let n = model.bodies.len();
        Self {
            model,
            options,
            base_dir,
            robot,
            leaf_of_mjcf_body: vec![None; n],
            world_pose_of_rapier_body: vec![MPose::IDENTITY],
            rapier_to_mjcf_body: vec![0],
            is_root_body: vec![false],
            welded_to_world: HashSet::new(),
            staged_colliders: Vec::new(),
            staged_collider_names: Vec::new(),
            staged_visual_meshes: Vec::new(),
            body_collider_names: vec![Vec::new()],
            body_masses: vec![0.0],
            body_mass_props: HashMap::new(),
        }
    }

    /// Consumes the driver, returning the built [`MjcfRobot`].
    pub(super) fn into_robot(self) -> MjcfRobot {
        self.robot
    }

    pub(super) fn run(&mut self) {
        // Build bodies + joints in MJCF order so parents precede children.
        // Index 0 is the world; skip it.
        // First pass: also compute world positions for each MJCF body using
        // their `pose` (relative to the parent's rapier body).
        self.leaf_of_mjcf_body[0] = Some(0);

        for (mjcf_id, entry) in self.model.bodies.iter().enumerate().skip(1) {
            self.materialize_body(mjcf_id, entry);
        }

        // After all bodies + joints are created, decide which rapier bodies
        // should be fixed.
        self.finalize_root_fixity();

        // Inject any geoms/sites attached directly to the worldbody as
        // colliders on the world rigid-body (they will all be inserted as
        // children of a fixed world body).
        self.materialize_world_geoms();

        self.lookup_tables();

        // Build the equality joints — must come after we know the rapier
        // body indices.
        self.materialize_equality();

        // Bindings for actuators / sensors.
        self.bind_extras();

        // Keyframes pass-through.
        self.robot.keyframes = self.model.keyframes.clone();

        // Contact data pass-through (resolved into hooks at insert time).
        self.robot.contact_excludes = self.model.contact.excludes.clone();
        self.robot.contact_pairs = self.model.contact.pairs.clone();

        // settotalmass post-processing: rescale every body's "additional"
        // mass properties so the total matches the target.
        if self.model.compiler.set_total_mass > 0.0 {
            self.apply_set_total_mass();
        }
    }

    fn apply_set_total_mass(&mut self) {
        let target = self.model.compiler.set_total_mass as Real;
        // Sum the masses we intentionally assigned. We tracked these in
        // `body_masses` while materializing each body.
        let total: Real = self.body_masses.iter().sum();
        if total <= 0.0 {
            return;
        }
        let factor = target / total;
        for (idx, m) in self.body_masses.iter().enumerate() {
            if *m <= 0.0 {
                continue;
            }
            let mjcf_body = &mut self.robot.bodies[idx];
            // Replace the existing additional-mass props with the rescaled
            // version. We rebuild from the original derived MassProperties
            // we cached during materialization.
            if let Some(orig) = self.body_mass_props.get(&idx).cloned() {
                let scaled = scale_mass_properties(orig, factor);
                mjcf_body.body.set_additional_mass_properties(scaled, false);
                mjcf_body.mass = scaled.mass();
            }
        }
    }

    fn materialize_body(&mut self, mjcf_id: BodyId, entry: &BodyEntry) {
        let parent_mjcf = entry.parent.unwrap_or(0);
        let parent_rapier = self.leaf_of_mjcf_body[parent_mjcf].unwrap_or(0);
        let parent_world_pose = self.world_pose_of_rapier_body[parent_rapier];
        let scaled_pose = MPose::from_parts(
            entry.body.pose.translation * self.options.scale as f64,
            entry.body.pose.rotation,
        );
        let body_world_pose = parent_world_pose * scaled_pose;

        let joints = &entry.body.joints;
        let has_free = joints.iter().any(|j| j.type_ == mb::JointType::Free);

        // Decide on intermediate-body chain. The final body is always the
        // MJCF-declared body; we may add 0..N intermediate bodies between
        // the parent and the final body.
        if has_free {
            if joints.len() != 1 {
                log::warn!(
                    "MJCF body `{}` has a free joint mixed with other joints; only the free joint is honored",
                    entry.body.name.as_deref().unwrap_or("<unnamed>")
                );
            }
            // Free body — no rapier joint, body is dynamic and not constrained.
            let _ = self.push_body_for_mjcf(mjcf_id, entry, body_world_pose, false);
            return;
        }

        // Non-free joints — build the chain.
        let serial = joints
            .iter()
            .filter(|j| j.type_ != mb::JointType::Free)
            .count();
        if serial == 0 {
            // No joint declared. If parent is world, we'll mark the body as
            // welded (fixed). Otherwise emit a fixed joint to the parent.
            let rb_idx = self.push_body_for_mjcf(mjcf_id, entry, body_world_pose, false);
            if parent_mjcf == 0 {
                self.welded_to_world.insert(mjcf_id);
            } else {
                let parent_to_child = parent_world_pose.inverse() * body_world_pose;
                let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
                    .local_frame1(parent_to_child.into())
                    .local_frame2(Pose::IDENTITY)
                    .contacts_enabled(self.options.enable_joint_collisions)
                    .build();
                self.robot.joints.push(MjcfJoint {
                    name: None,
                    link1: parent_rapier,
                    link2: rb_idx,
                    joint,
                    damping_per_dof: 0.0,
                    armature_per_dof: 0.0,
                    spring_stiffness_per_dof: 0.0,
                    spring_ref: 0.0,
                    springdamper: None,
                });
            }
            return;
        }

        // serial >= 1: each joint adds one DoF in series. We synthesize
        // (serial - 1) intermediate bodies, then the final body. The very
        // last joint connects the (serial - 1)th intermediate (or the
        // parent if serial == 1) to the final body.
        let mut prev_rapier = parent_rapier;
        let mut prev_world_pose = parent_world_pose;
        for (idx, j) in joints
            .iter()
            .filter(|j| j.type_ != mb::JointType::Free)
            .enumerate()
        {
            let is_last = idx + 1 == serial;
            let cur_rapier = if is_last {
                self.push_body_for_mjcf(mjcf_id, entry, body_world_pose, false)
            } else {
                self.push_intermediate_body(mjcf_id, entry, body_world_pose)
            };
            // For multi-joint chains, the intermediate bodies sit at the
            // final body's world pose — they're mass-less spacers, and the
            // joint's pos/axis encodes the geometric DoF.
            let cur_world_pose = body_world_pose;
            let joint = self.build_serial_joint(j, prev_world_pose, cur_world_pose);

            // MuJoCo's `<joint springdamper="timeconst dampratio">` requests a
            // spring with that time constant and damping ratio; the actual
            // stiffness/damping depend on the assembled joint-space inertia, so
            // they're resolved post-assembly (see `add_springdamper_to_multibody`).
            // MuJoCo requires both values strictly positive (it errors
            // otherwise); we warn-and-ignore an out-of-spec pair. A valid
            // springdamper overrides the explicit stiffness/damping.
            let springdamper = match j.springdamper {
                Some(sd) if sd[0] > 0.0 && sd[1] > 0.0 => Some((sd[0] as Real, sd[1] as Real)),
                Some(_) => {
                    log::warn!(
                        "<joint name={:?}>: springdamper needs two positive values \
                         (timeconst dampratio); ignoring it (MuJoCo rejects this)",
                        j.name
                    );
                    None
                }
                None => None,
            };

            // Per-DoF damping: applied uniformly across the joint's free DoFs.
            // The `disable_joint_motors` option intentionally does NOT affect
            // this — per-DoF damping is not a motor, it's a friction-like term
            // on the dynamics equations. A valid `springdamper` supplies its own
            // damping post-assembly, so it overrides `<joint damping>` here.
            let damping_per_dof = if springdamper.is_some() {
                0.0
            } else {
                j.damping as Real
            };
            // Armature (reflected rotor inertia) is routed into the multibody's
            // generalized mass matrix at insertion time rather than baked into
            // the link's spatial inertia — see `MjcfJoint::armature_per_dof`.
            let armature_per_dof = j.armature.max(0.0) as Real;
            // Passive spring carried so the multibody path can integrate it
            // implicitly (a valid `springdamper` overrides `<joint stiffness>`,
            // leaving this 0 and supplying the stiffness post-assembly). A
            // spring is *not* a motor, so it is deliberately independent of
            // `disable_joint_motors` / `SKIP_JOINT_MOTORS`; use
            // `SKIP_JOINT_SPRINGS` to drop it.
            let spring_ref = (j.springref - j.ref_) as Real;
            let spring_stiffness_per_dof = if springdamper.is_none() && j.stiffness > 0.0 {
                j.stiffness as Real
            } else {
                0.0
            };
            self.robot.joints.push(MjcfJoint {
                name: j.name.clone(),
                link1: prev_rapier,
                link2: cur_rapier,
                joint,
                damping_per_dof,
                armature_per_dof,
                spring_stiffness_per_dof,
                spring_ref,
                springdamper,
            });
            prev_rapier = cur_rapier;
            prev_world_pose = cur_world_pose;
        }
    }

    /// Push the rapier body that materializes the MJCF body itself.
    fn push_body_for_mjcf(
        &mut self,
        mjcf_id: BodyId,
        entry: &BodyEntry,
        world_pose: MPose,
        force_fixed: bool,
    ) -> usize {
        let mut builder = self.build_rigid_body(entry, world_pose, force_fixed);
        let derived_mp = if self.options.apply_imported_mass_props {
            self.derive_mass_properties(entry)
        } else {
            None
        };
        if let Some(mp_) = derived_mp {
            builder = builder.additional_mass_properties(mp_);
        }
        let mut body = builder.build();
        self.attach_colliders_to_body(mjcf_id, entry, world_pose, &mut body);
        let idx = self.robot.bodies.len();
        let body_mass = derived_mp.as_ref().map(|mp| mp.mass()).unwrap_or(0.0);
        self.robot.bodies.push(MjcfBody {
            name: entry.body.name.clone(),
            is_intermediate: false,
            mjcf_body_id: mjcf_id,
            body,
            colliders: std::mem::take(&mut self.staged_colliders),
            gravcomp: entry.body.gravcomp,
            mass: body_mass,
            visual_meshes: std::mem::take(&mut self.staged_visual_meshes),
        });
        self.body_collider_names
            .push(std::mem::take(&mut self.staged_collider_names));
        self.world_pose_of_rapier_body.push(world_pose);
        self.rapier_to_mjcf_body.push(mjcf_id);
        self.is_root_body.push(entry.parent == Some(0));
        self.leaf_of_mjcf_body[mjcf_id] = Some(idx);
        if let Some(mp_) = derived_mp {
            self.body_masses.push(mp_.mass());
            self.body_mass_props.insert(idx, mp_);
        } else {
            self.body_masses.push(0.0);
        }
        idx
    }

    /// Push a rapier body to act as a massless spacer in a multi-joint chain.
    fn push_intermediate_body(
        &mut self,
        mjcf_id: BodyId,
        entry: &BodyEntry,
        world_pose: MPose,
    ) -> usize {
        let mut builder = if entry.body.mocap {
            RigidBodyBuilder::kinematic_position_based()
        } else {
            RigidBodyBuilder::dynamic()
        };
        builder = builder.pose(world_pose.into()).can_sleep(entry.body.sleep);
        let body = builder.build();
        let idx = self.robot.bodies.len();
        self.robot.bodies.push(MjcfBody {
            name: entry
                .body
                .name
                .as_ref()
                .map(|n| format!("{n}__intermediate_{idx}")),
            is_intermediate: true,
            mjcf_body_id: mjcf_id,
            body,
            colliders: Vec::new(),
            gravcomp: 0.0,
            mass: 0.0,
            visual_meshes: Vec::new(),
        });
        self.body_collider_names.push(Vec::new());
        self.body_masses.push(0.0);
        self.world_pose_of_rapier_body.push(world_pose);
        self.rapier_to_mjcf_body.push(mjcf_id);
        self.is_root_body.push(false);
        idx
    }

    /// Builds a [`RigidBodyBuilder`] for an MJCF body.
    fn build_rigid_body(
        &self,
        entry: &BodyEntry,
        world_pose: MPose,
        force_fixed: bool,
    ) -> RigidBodyBuilder {
        let with_shift = self.options.shift * Pose::from(world_pose);
        let mut builder = self.options.rigid_body_blueprint.clone();
        if force_fixed {
            builder.body_type = RigidBodyType::Fixed;
        } else if entry.body.mocap {
            builder.body_type = RigidBodyType::KinematicPositionBased;
        } else {
            builder.body_type = RigidBodyType::Dynamic;
        }
        builder = builder.pose(with_shift);
        if !entry.body.sleep {
            builder = builder.can_sleep(false);
        }
        // gravcomp ∈ {0, 1} maps cleanly to gravity_scale; intermediate
        // values fall back to gravity_scale = 1.0 - gravcomp (the user can
        // refine later via apply_gravity_compensation if they care about
        // exact tracking when gravcomp is fractional and the body's
        // configuration changes between steps).
        if entry.body.gravcomp != 0.0 {
            let scale = 1.0 - entry.body.gravcomp as Real;
            builder = builder.gravity_scale(scale);
        }
        builder
    }

    fn finalize_root_fixity(&mut self) {
        let make_roots_fixed = self.options.make_roots_fixed;
        for i in 1..self.robot.bodies.len() {
            let mjcf_id = self.rapier_to_mjcf_body[i];
            let force = self.welded_to_world.contains(&mjcf_id);
            let entry = &self.model.bodies[mjcf_id].body;
            // mocap bodies should remain `KinematicPositionBased` even
            // when they would otherwise be welded to the world.
            if entry.mocap {
                continue;
            }
            let has_free = entry.joints.iter().any(|j| j.type_ == mb::JointType::Free);
            let body = &mut self.robot.bodies[i].body;
            if force {
                // Body welded to the world (no joint declared with a
                // world-parent) — already Fixed by construction.
                body.set_body_type(RigidBodyType::Fixed, false);
            } else if make_roots_fixed && has_free {
                // `make_roots_fixed` analogue of URDF's "fix free roots":
                // bodies declared with `<freejoint>` are the MJCF concept
                // of a free-floating root, so this is the only category
                // we promote to Fixed. Bodies that connect to the world
                // through a regular joint are *not* roots (the world is
                // their parent through a constraint) — promoting them to
                // Fixed would break the multibody insertion path, which
                // requires non-root links to be Dynamic.
                body.set_body_type(RigidBodyType::Fixed, false);
            }
        }
    }

    fn materialize_world_geoms(&mut self) {
        // The world body sits at index 0 (already pushed).
        let entry_geoms = self.model.bodies[0].body.geoms.clone();
        self.staged_colliders.clear();
        self.staged_collider_names.clear();
        self.staged_visual_meshes.clear();
        for g in &entry_geoms {
            if let Some(c) = self.build_collider(g) {
                self.staged_colliders.push(c);
                self.staged_collider_names.push(g.name.clone());
            } else if let Some(vm) = self.build_visual_mesh(g) {
                self.staged_visual_meshes.push(vm);
            }
        }
        // Same "synthesize visuals for collision-active mesh geoms"
        // pass that body bodies get — the world body has its own
        // geoms in some scenes (e.g. floor / props) and they deserve
        // the same treatment.
        self.synthesize_visuals_for_collider_meshes(&entry_geoms);
        self.robot.bodies[0].visual_meshes = std::mem::take(&mut self.staged_visual_meshes);
        self.robot.bodies[0].colliders = std::mem::take(&mut self.staged_colliders);
        self.body_collider_names[0] = std::mem::take(&mut self.staged_collider_names);
    }

    fn lookup_tables(&mut self) {
        for (i, b) in self.robot.bodies.iter().enumerate() {
            if !b.is_intermediate
                && let Some(name) = &b.name
            {
                self.robot.body_name_to_idx.insert(name.clone(), i);
            }
        }
        // Geoms keyed against the actual rapier collider index in each body.
        for (i, names) in self.body_collider_names.iter().enumerate() {
            for (ci, name) in names.iter().enumerate() {
                if let Some(name) = name {
                    self.robot
                        .geom_name_to_collider
                        .entry(name.clone())
                        .or_insert((i, ci));
                }
            }
        }
        for (i, j) in self.robot.joints.iter().enumerate() {
            if let Some(name) = &j.name {
                self.robot.joint_name_to_idx.insert(name.clone(), i);
            }
        }
    }

    fn materialize_equality(&mut self) {
        for eq in &self.model.equality {
            match eq {
                MjcfEquality::Connect(c) => {
                    if let Some((l1, l2, frame1, frame2)) = self.resolve_connect_frames(c) {
                        let joint = GenericJointBuilder::new(JointAxesMask::LIN_AXES)
                            .local_frame1(frame1)
                            .local_frame2(frame2)
                            .contacts_enabled(self.options.enable_joint_collisions)
                            .build();
                        self.robot.equality_joints.push(MjcfEqualityJoint {
                            name: c.common.name.clone(),
                            link1: l1,
                            link2: l2,
                            active: c.common.active,
                            joint,
                        });
                    }
                }
                MjcfEquality::Weld(w) => {
                    if let Some((l1, l2, frame1, frame2)) = self.resolve_weld_frames(w) {
                        let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
                            .local_frame1(frame1)
                            .local_frame2(frame2)
                            .contacts_enabled(self.options.enable_joint_collisions)
                            .build();
                        self.robot.equality_joints.push(MjcfEqualityJoint {
                            name: w.common.name.clone(),
                            link1: l1,
                            link2: l2,
                            active: w.common.active,
                            joint,
                        });
                    }
                }
                MjcfEquality::Joint(j) => self.materialize_joint_equality(j),
            }
        }
    }

    /// `<equality><joint>`: resolve the linear (first-order) coupling
    /// `q2 = polycoef[1]·q1 + polycoef[0]` between two joints and record it for
    /// the multibody insertion pass. The MJCF polynomial is in `(q − ref)`, but
    /// rapier joint coordinates already subtract `ref`, so the reference terms
    /// cancel and the relation is exactly `q2 = polycoef[1]·q1 + polycoef[0]`.
    fn materialize_joint_equality(&mut self, j: &MjcfEqualityJointDef) {
        if j.polycoef[2] != 0.0 || j.polycoef[3] != 0.0 || j.polycoef[4] != 0.0 {
            log::warn!(
                "<equality><joint name={:?}>: only a linear polycoef is supported; \
                 higher-order terms are ignored",
                j.common.name
            );
        }
        let Some(joint2_name) = j.joint2.as_deref() else {
            log::warn!(
                "<equality><joint name={:?}>: the single-joint form (joint2 omitted) is \
                 unsupported; skipping",
                j.common.name
            );
            return;
        };
        let (Some(joint1), Some(joint2)) = (
            self.robot.joint_name_to_idx.get(&j.joint1).copied(),
            self.robot.joint_name_to_idx.get(joint2_name).copied(),
        ) else {
            log::warn!(
                "<equality><joint name={:?}>: unknown joint1 {:?} or joint2 {:?}; skipping",
                j.common.name,
                j.joint1,
                joint2_name,
            );
            return;
        };
        self.robot.joint_couplings.push(MjcfJointCoupling {
            joint1,
            joint2,
            coeff: j.polycoef[1] as Real,
            offset: j.polycoef[0] as Real,
            active: j.common.active,
        });
    }

    fn resolve_connect_frames(&self, c: &EqualityConnect) -> Option<(usize, usize, Pose, Pose)> {
        let Some(body1) = self.robot.body_name_to_idx.get(&c.body1).copied() else {
            log::warn!(
                "<equality><connect name={:?}>: unknown body1 \"{}\"; skipping",
                c.common.name,
                c.body1,
            );
            return None;
        };
        let body2 = if let Some(name) = c.body2.as_deref() {
            match self.robot.body_name_to_idx.get(name).copied() {
                Some(idx) => idx,
                None => {
                    log::warn!(
                        "<equality><connect name={:?}>: unknown body2 \"{name}\"; skipping",
                        c.common.name,
                    );
                    return None;
                }
            }
        } else {
            0
        };
        // anchor is in body1's frame. Body world poses are already in
        // scaled world coords (see `materialize_body` → `scale_pose`),
        // so the MJCF anchor must be scaled too — otherwise composing
        // `world1 * anchor1` puts the loop-closure point ~(scale-1)·|anchor|
        // away from where the model intended, and the impulse joint
        // starts violated by that amount.
        let s = self.options.scale as f64;
        let anchor1 = MPose::from_translation(DVec3::new(
            c.anchor[0] * s,
            c.anchor[1] * s,
            c.anchor[2] * s,
        ));
        // Compute the equivalent anchor in body2's frame (so the joint
        // resolves at zero).
        let world1 = self.world_pose_of_rapier_body[body1];
        let world2 = self.world_pose_of_rapier_body[body2];
        let anchor_world = world1 * anchor1;
        let anchor2 = world2.inverse() * anchor_world;
        Some((body1, body2, Pose::from(anchor1), Pose::from(anchor2)))
    }

    fn resolve_weld_frames(&self, w: &EqualityWeld) -> Option<(usize, usize, Pose, Pose)> {
        let Some(body1) = self.robot.body_name_to_idx.get(&w.body1).copied() else {
            log::warn!(
                "<equality><weld name={:?}>: unknown body1 \"{}\"; skipping",
                w.common.name,
                w.body1,
            );
            return None;
        };
        let body2 = if let Some(name) = w.body2.as_deref() {
            match self.robot.body_name_to_idx.get(name).copied() {
                Some(idx) => idx,
                None => {
                    log::warn!(
                        "<equality><weld name={:?}>: unknown body2 \"{name}\"; skipping",
                        w.common.name,
                    );
                    return None;
                }
            }
        } else {
            0
        };
        let world1 = self.world_pose_of_rapier_body[body1];
        let world2 = self.world_pose_of_rapier_body[body2];
        // Frame1 in body1's local: identity (anchor at body's origin) unless
        // an explicit anchor or relpose is given. The anchor and relpose
        // translations come from the raw MJCF and must be scaled to match
        // the (already-scaled) body world poses — see the matching note in
        // `resolve_connect_frames`.
        let s = self.options.scale as f64;
        let frame1_local = w
            .anchor
            .map(|a| MPose::from_translation(DVec3::new(a[0] * s, a[1] * s, a[2] * s)))
            .unwrap_or(MPose::IDENTITY);
        let frame1_world = world1 * frame1_local;
        let frame2_world = if let Some(rel) = w.relpose {
            let scaled_rel = MPose::from_parts(rel.translation * s, rel.rotation);
            frame1_world * scaled_rel
        } else {
            frame1_world
        };
        let frame2_local = world2.inverse() * frame2_world;
        Some((
            body1,
            body2,
            Pose::from(frame1_local),
            Pose::from(frame2_local),
        ))
    }

    fn bind_extras(&mut self) {
        for a in &self.model.actuators {
            let joint_index = match a.joint.as_deref() {
                Some(joint_name) => match self.robot.joint_name_to_idx.get(joint_name).copied() {
                    Some(idx) => Some(idx),
                    None => {
                        log::warn!(
                            "<actuator name={:?} joint=\"{joint_name}\">: unknown joint name; \
                             actuator will be recorded but apply_controls() will skip it",
                            a.name,
                        );
                        None
                    }
                },
                None => {
                    if a.tendon.is_some() {
                        // Tendons are out of scope; not actionable. No warning.
                    } else if a.body.is_none() && a.site.is_none() {
                        log::warn!(
                            "<actuator name={:?}>: no `joint=`, `tendon=`, `body=`, or `site=` \
                             reference; actuator will be recorded but apply_controls() will skip it",
                            a.name,
                        );
                    }
                    None
                }
            };
            self.robot.actuators.push(MjcfActuatorBinding {
                actuator: a.clone(),
                joint_index,
            });
        }
        for s in &self.model.sensors {
            let object = self.resolve_sensor_object(s);
            self.robot.sensors.push(MjcfSensorBinding {
                sensor: s.clone(),
                object,
            });
        }
    }

    fn resolve_sensor_object(&self, s: &MjcfSensor) -> SensorObjectRef {
        let (Some(ty), Some(name)) = (s.objtype.as_deref(), s.objname.as_deref()) else {
            return SensorObjectRef::None;
        };
        match ty {
            "body" => match self.robot.body_name_to_idx.get(name).copied() {
                Some(idx) => SensorObjectRef::Body(idx),
                None => {
                    log::warn!(
                        "<sensor name={:?} kind={:?} objtype=\"body\" objname=\"{name}\">: unknown body; sensor will return None",
                        s.name,
                        s.kind,
                    );
                    SensorObjectRef::None
                }
            },
            "joint" => match self.robot.joint_name_to_idx.get(name).copied() {
                Some(idx) => SensorObjectRef::Joint(idx),
                None => {
                    log::warn!(
                        "<sensor name={:?} kind={:?} objtype=\"joint\" objname=\"{name}\">: unknown joint; sensor will return None",
                        s.name,
                        s.kind,
                    );
                    SensorObjectRef::None
                }
            },
            "site" => match self.model.site_by_name(name) {
                Some((mjcf_body, site_idx)) => match self.leaf_of_mjcf_body[mjcf_body] {
                    Some(rb) => SensorObjectRef::Site {
                        body: rb,
                        site: site_idx,
                    },
                    None => {
                        log::warn!(
                            "<sensor name={:?} kind={:?} objtype=\"site\" objname=\"{name}\">: site's body wasn't materialized; sensor will return None",
                            s.name,
                            s.kind,
                        );
                        SensorObjectRef::None
                    }
                },
                None => {
                    log::warn!(
                        "<sensor name={:?} kind={:?} objtype=\"site\" objname=\"{name}\">: unknown site; sensor will return None",
                        s.name,
                        s.kind,
                    );
                    SensorObjectRef::None
                }
            },
            other => {
                log::debug!(
                    "<sensor name={:?} kind={:?} objtype=\"{other}\" objname=\"{name}\">: unrecognised objtype; sensor will return None",
                    s.name,
                    s.kind,
                );
                SensorObjectRef::None
            }
        }
    }
}
