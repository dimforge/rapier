//! Mass-property derivation, scaling, clamping, and the
//! multibody-damping-routing helper for post-insertion handling.
//!
//! The `<inertial>` element, `inertiafromgeom="auto"`, `bound_*` clamps and
//! `balance_inertia` all funnel through here. `move_motor_damping_to_multibody`
//! is the inverse-direction helper used at insertion time to push MJCF
//! `<joint damping>` values into the multibody's per-DoF damping vector.

use mjcf_rs::body as mb;
use mjcf_rs::compiler::InertiaFromGeom;
use mjcf_rs::model::BodyEntry;

use rapier3d::dynamics::{
    MassProperties, Multibody, MultibodyDofCoupling, MultibodyJointHandle, MultibodyJointSet,
    RigidBodyHandle, RigidBodySet,
};
use rapier3d::math::{Matrix, Pose, Real, Vector};

use super::conversion::Conversion;

impl<'a> Conversion<'a> {
    /// Compute mass properties from `<inertial>` (or derive from geoms when
    /// the compiler asks for it).
    ///
    /// Note: `<joint armature>` is intentionally **not** folded into the
    /// returned inertia tensor. MuJoCo's armature is a reflected rotor inertia
    /// that belongs on the diagonal of the joint-space mass matrix, not on the
    /// link's spatial inertia — baking it into the tensor produces extreme
    /// anisotropy (huge along the joint axis, ~0 across it) and an
    /// ill-conditioned multibody mass matrix. It is routed through
    /// [`add_armature_to_multibody`] at insertion time instead.
    pub(super) fn derive_mass_properties(&self, entry: &BodyEntry) -> Option<MassProperties> {
        let s = self.options.scale;
        let from_inertial = entry
            .body
            .inertial
            .as_ref()
            .map(|i| inertial_to_mass_props(i, s));
        let force_geom = self.model.compiler.inertia_from_geom == InertiaFromGeom::True;
        let auto_geom = self.model.compiler.inertia_from_geom == InertiaFromGeom::Auto
            && entry.body.inertial.is_none();
        let mp = if force_geom || auto_geom {
            self.geoms_to_mass_props(entry).or(from_inertial)
        } else {
            from_inertial
        };
        mp.map(|mp_| clamp_mass_properties(mp_, &self.model.compiler))
    }

    pub(super) fn geoms_to_mass_props(&self, entry: &BodyEntry) -> Option<MassProperties> {
        let density_default = self.model.compiler.inertia_density;
        let mut acc: Option<MassProperties> = None;
        for g in &entry.body.geoms {
            let (shape, body_frame_pose) = match self.build_geom_shape(g) {
                Some(s) => s,
                None => continue,
            };
            let density = if let Some(m) = g.mass {
                let v = shape.mass_properties(1.0).mass();
                if v > 1e-30 { (m as Real) / v } else { 0.0 }
            } else {
                g.density.unwrap_or(density_default) as Real
            };
            if density <= 0.0 {
                continue;
            }
            let mp_geom = shape.mass_properties(density);
            // `body_frame_pose` already includes the geom's `pos`/`quat`
            // attribute and the shape's intrinsic offset.
            let mp_local = mp_geom.transform_by(&body_frame_pose);
            acc = Some(match acc {
                None => mp_local,
                Some(a) => a + mp_local,
            });
        }
        acc
    }
}

pub(super) fn inertial_to_mass_props(i: &mb::Inertial, scale: Real) -> MassProperties {
    // The inertia tensor has units of kg·m² — when we scale lengths by
    // `scale`, the inertia tensor must scale by `scale²` so the body's
    // rotational dynamics stay consistent with the (also-scaled) com
    // offset and joint anchor positions. Mass itself is not scaled.
    let com = Vector::new(
        i.pose.translation.x as Real * scale,
        i.pose.translation.y as Real * scale,
        i.pose.translation.z as Real * scale,
    );
    let s2 = scale * scale;
    match i.inertia {
        mb::InertiaSpec::Diagonal(d) => {
            let principal = Vector::new(d[0] as Real * s2, d[1] as Real * s2, d[2] as Real * s2);
            let r = Pose::from(i.pose).rotation;
            MassProperties::with_principal_inertia_frame(com, i.mass as Real, principal, r)
        }
        mb::InertiaSpec::Full(f) => {
            let ixx = f[0] as Real * s2;
            let iyy = f[1] as Real * s2;
            let izz = f[2] as Real * s2;
            let ixy = f[3] as Real * s2;
            let ixz = f[4] as Real * s2;
            let iyz = f[5] as Real * s2;
            // Mat3 is column-major; the inertia matrix is symmetric so column
            // and row layouts coincide.
            let m = Matrix::from_cols_array(&[ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]);
            MassProperties::with_inertia_matrix(com, i.mass as Real, m)
        }
    }
}

pub(super) fn scale_mass_properties(mp: MassProperties, factor: Real) -> MassProperties {
    MassProperties::with_principal_inertia_frame(
        mp.local_com,
        mp.mass() * factor,
        mp.principal_inertia() * factor,
        mp.principal_inertia_local_frame,
    )
}

pub(super) fn clamp_mass_properties(
    mp: MassProperties,
    compiler: &mjcf_rs::compiler::Compiler,
) -> MassProperties {
    let mass = mp.mass();
    let bm = compiler.bound_mass as Real;
    let new_mass = if bm > 0.0 && mass < bm { bm } else { mass };
    let principal = mp.principal_inertia();
    let bi = compiler.bound_inertia as Real;
    let mut clamped = principal;
    if bi > 0.0 {
        for i in 0..3 {
            if clamped[i] < bi {
                clamped[i] = bi;
            }
        }
    }
    if compiler.balance_inertia {
        // I_xx >= |I_yy - I_zz|, etc.
        let a = (clamped[1] - clamped[2]).abs();
        if clamped[0] < a {
            clamped[0] = a;
        }
        let b = (clamped[0] - clamped[2]).abs();
        if clamped[1] < b {
            clamped[1] = b;
        }
        let c = (clamped[0] - clamped[1]).abs();
        if clamped[2] < c {
            clamped[2] = c;
        }
    }
    MassProperties::with_principal_inertia_frame(
        mp.local_com,
        new_mass,
        clamped,
        mp.principal_inertia_local_frame,
    )
}

/// After a joint has been inserted into a multibody, route the MJCF
/// `<joint damping>` value through the multibody's per-DoF damping vector
/// (more stable than motor damping for stiff systems) and zero the
/// motor's damping component on each free axis so we don't double-damp.
///
/// The damping is applied uniformly to every free DoF of the joint — so a
/// ball joint with MJCF damping=d gets `d` on each of its three angular
/// DoFs (isotropic, matching MJCF semantics).
pub(super) fn move_motor_damping_to_multibody(
    multibody_joints: &mut MultibodyJointSet,
    handle: MultibodyJointHandle,
    damping: Real,
) {
    use rapier3d::math::SPATIAL_DIM;
    let Some((multibody, link_id)) = multibody_joints.get_mut(handle) else {
        return;
    };
    // Walk the link list once to compute this link's DoF offset in the
    // multibody's flat damping vector (rapier doesn't expose
    // `MultibodyLink::assembly_id` publicly, so we reconstruct it).
    let mut offset = 0;
    for (i, link) in multibody.links().enumerate() {
        if i == link_id {
            break;
        }
        offset += link.joint().ndofs();
    }
    let Some(link) = multibody.links_mut().nth(link_id) else {
        return;
    };
    let locked_bits = link.joint.data.locked_axes.bits();
    // Zero motor damping for every motorised axis so we don't double-damp
    // (motor still applies its stiffness / target-velocity contribution).
    let motor_bits = link.joint.data.motor_axes.bits();
    for i in 0..SPATIAL_DIM {
        if (motor_bits & (1 << i)) != 0 {
            link.joint.data.motors[i].damping = 0.0;
        }
    }
    // Drop the &mut MultibodyLink borrow before reborrowing the multibody.
    let _ = link;
    let damping_vec = multibody.damping_mut();
    let mut local_dof = 0;
    for i in 0..SPATIAL_DIM {
        if (locked_bits & (1 << i)) == 0 {
            let idx = offset + local_dof;
            if idx < damping_vec.len() {
                damping_vec[idx] = damping;
            }
            local_dof += 1;
        }
    }
}

/// After a joint has been inserted into a multibody, add the MJCF
/// `<joint armature>` (reflected rotor inertia) to the multibody's per-DoF
/// armature vector. Each entry lands on the diagonal of the generalized mass
/// matrix — the joint-space placement MuJoCo uses — rather than in the link's
/// spatial inertia tensor.
///
/// The armature is applied uniformly to every free DoF of the joint, so a ball
/// joint with MJCF armature=a gets `a` on each of its three angular DoFs,
/// matching MJCF semantics.
pub(super) fn add_armature_to_multibody(
    multibody_joints: &mut MultibodyJointSet,
    handle: MultibodyJointHandle,
    armature: Real,
) {
    use rapier3d::math::SPATIAL_DIM;
    let Some((multibody, link_id)) = multibody_joints.get_mut(handle) else {
        return;
    };
    // Reconstruct this link's DoF offset in the multibody's flat armature
    // vector (assembly_id isn't public), same as the damping helper above.
    let mut offset = 0;
    for (i, link) in multibody.links().enumerate() {
        if i == link_id {
            break;
        }
        offset += link.joint().ndofs();
    }
    let Some(link) = multibody.links().nth(link_id) else {
        return;
    };
    let locked_bits = link.joint.data.locked_axes.bits();
    let armature_vec = multibody.armature_mut();
    let mut local_dof = 0;
    for i in 0..SPATIAL_DIM {
        if (locked_bits & (1 << i)) == 0 {
            let idx = offset + local_dof;
            if idx < armature_vec.len() {
                armature_vec[idx] = armature;
            }
            local_dof += 1;
        }
    }
}

/// After a joint has been inserted into a multibody, install a passive
/// `<joint stiffness springref>` spring as an *implicit* spring on the
/// multibody link (force `-k·(q − rest)`, integrated implicitly in the
/// generalized dynamics) and remove the explicit position-motor spring that
/// the serial-joint builder created for the impulse path. The spring is
/// applied to every free DoF of the joint, matching MuJoCo's per-joint
/// `stiffness`.
///
/// Implicit integration keeps stiff springs stable on low-inertia links,
/// where the explicit motor injects energy (e.g. robotiq's `spring_link`
/// driving the near-massless follower, or flybody's leg springs).
pub(super) fn add_spring_to_multibody(
    multibody_joints: &mut MultibodyJointSet,
    handle: MultibodyJointHandle,
    stiffness: Real,
    rest: Real,
) {
    use rapier3d::dynamics::JointAxesMask;
    use rapier3d::math::SPATIAL_DIM;
    let Some((multibody, link_id)) = multibody_joints.get_mut(handle) else {
        return;
    };
    let Some(link) = multibody.links_mut().nth(link_id) else {
        return;
    };
    let locked_bits = link.joint.data.locked_axes.bits();
    for axis in 0..SPATIAL_DIM {
        if (locked_bits & (1 << axis)) == 0 {
            link.joint.set_spring(axis, stiffness, rest);
            // Drop the explicit motor spring on this axis so the implicit
            // spring isn't double-applied. Actuators (applied later via
            // `apply_controls`) re-enable the motor on the axes they drive.
            if let Some(flag) = JointAxesMask::from_bits(1u8 << axis) {
                link.joint.data.motor_axes.remove(flag);
            }
        }
    }
}

/// Install a MJCF `<joint springdamper="timeconst dampratio">` spring,
/// resolved against the *assembled* multibody so it realizes the requested
/// time constant and damping ratio — MuJoCo's `AutoSpringDamper`. Must run
/// after the whole multibody is built and its bodies' mass properties are up
/// to date (`dof_inverse_inertia` reads `diag(M⁻¹)`).
///
/// With effective inertia `I = ndof / Σ dof_invweight0` over the joint's DoFs:
/// `stiffness = I / (T²·ζ²)` and `damping = 2·I / T`. The stiffness becomes an
/// implicit spring (like [`add_spring_to_multibody`]) and the damping goes to
/// the per-DoF damping vector; the explicit motor on those axes is cleared.
pub(super) fn add_springdamper_to_multibody(
    multibody_joints: &mut MultibodyJointSet,
    bodies: &RigidBodySet,
    handle: MultibodyJointHandle,
    timeconst: Real,
    dampratio: Real,
    rest: Real,
) {
    use rapier3d::dynamics::JointAxesMask;
    use rapier3d::math::SPATIAL_DIM;

    let Some((multibody, link_id)) = multibody_joints.get_mut(handle) else {
        return;
    };

    // Per-DoF inverse joint-space inertia (also finalizes the root type).
    let invweight = multibody.dof_inverse_inertia(bodies);

    // This link's DoF offset and count in the generalized vectors.
    let mut offset = 0;
    let mut ndofs = 0;
    for (i, link) in multibody.links().enumerate() {
        let link_ndofs = link.joint().ndofs();
        if i == link_id {
            ndofs = link_ndofs;
            break;
        }
        offset += link_ndofs;
    }
    if ndofs == 0 {
        return;
    }

    // MuJoCo `AutoSpringDamper`: inertia = ndof / Σ invweight0, then
    // stiffness = inertia / (T²·ζ²), damping = 2·inertia / T.
    //
    // NOTE: the effective inertia is evaluated once here, at the load-time
    // configuration (MuJoCo does the same at `qpos0`), and `stiffness`/`damping`
    // are then *constant* — that's what makes this a physical spring. The
    // requested time constant / damping ratio hold exactly only at this pose;
    // as the articulated inertia changes with configuration the effective ω/ζ
    // drift, which is correct spring behavior. Re-deriving the coefficients from
    // the current inertia every frame would instead hold ω/ζ constant by making
    // the *stiffness* configuration-dependent — a normalized impedance, not a
    // spring, and a divergence from MuJoCo. (The integration itself already uses
    // the current per-frame mass matrix; only this calibration is fixed.)
    let eps = 1.0e-9;
    let sum_inv: Real = (0..ndofs)
        .map(|d| invweight.get(offset + d).copied().unwrap_or(0.0))
        .sum();
    let inertia = if sum_inv > eps {
        ndofs as Real / sum_inv
    } else {
        0.0
    };
    let stiffness = inertia / (timeconst * timeconst * dampratio * dampratio).max(eps);
    let damping = 2.0 * inertia / timeconst.max(eps);

    // Stiffness → implicit spring; clear the explicit motor on those axes.
    if let Some(link) = multibody.links_mut().nth(link_id) {
        let locked_bits = link.joint.data.locked_axes.bits();
        for axis in 0..SPATIAL_DIM {
            if (locked_bits & (1 << axis)) == 0 {
                link.joint.set_spring(axis, stiffness, rest);
                if let Some(flag) = JointAxesMask::from_bits(1u8 << axis) {
                    link.joint.data.motor_axes.remove(flag);
                }
            }
        }
    }

    // Damping → per-DoF damping vector (the joint's DoFs are offset..+ndofs).
    let damping_vec = multibody.damping_mut();
    for d in 0..ndofs {
        let idx = offset + d;
        if idx < damping_vec.len() {
            damping_vec[idx] = damping;
        }
    }
}

/// The local DoF index and spatial axis of a single-DoF multibody joint's free
/// axis (the first non-locked axis). Returns `None` if the link or a free axis
/// can't be found.
fn single_free_axis(multibody: &Multibody, link_id: usize) -> Option<(usize, usize)> {
    use rapier3d::math::SPATIAL_DIM;
    let link = multibody.links().nth(link_id)?;
    let locked = link.joint.data.locked_axes.bits();
    for axis in 0..SPATIAL_DIM {
        if (locked & (1 << axis)) == 0 {
            // First free axis ⇒ local DoF index 0 within the link's slice.
            return Some((0, axis));
        }
    }
    None
}

/// Install a `<equality><joint>` coupling `q2 = coeff·q1 + offset` as a DoF
/// coupling on the multibody shared by the two joints (`child1`/`child2` are
/// the child bodies of joint1/joint2; `handle1` is joint1's multibody handle).
/// Both joints must live in the same multibody and be single-DoF (hinge/slide).
pub(super) fn add_joint_coupling_to_multibody(
    multibody_joints: &mut MultibodyJointSet,
    handle1: MultibodyJointHandle,
    child1: RigidBodyHandle,
    child2: RigidBodyHandle,
    coeff: Real,
    offset: Real,
) {
    let (Some(l1), Some(l2)) = (
        multibody_joints.rigid_body_link(child1).copied(),
        multibody_joints.rigid_body_link(child2).copied(),
    ) else {
        return;
    };
    if l1.multibody != l2.multibody {
        log::warn!(
            "<equality><joint>: joint1 and joint2 are in different multibodies; \
             cross-multibody coupling is unsupported, skipping"
        );
        return;
    }
    let link2_id = l2.id;

    let Some((multibody, link1_id)) = multibody_joints.get_mut(handle1) else {
        return;
    };
    let (Some((dof1, axis1)), Some((dof2, axis2))) = (
        single_free_axis(multibody, link1_id),
        single_free_axis(multibody, link2_id),
    ) else {
        log::warn!("<equality><joint>: a coupled joint has no free DoF; skipping");
        return;
    };
    multibody.add_dof_coupling(MultibodyDofCoupling {
        link1: link1_id,
        dof1,
        axis1,
        link2: link2_id,
        dof2,
        axis2,
        coeff,
        offset,
    });
}
