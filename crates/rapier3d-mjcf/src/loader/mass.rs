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

use rapier3d::dynamics::{MassProperties, MultibodyJointHandle, MultibodyJointSet};
use rapier3d::math::{Matrix, Real, Vector};

use super::conversion::Conversion;
use super::pose_utils::mpose_to_rapier;

impl<'a> Conversion<'a> {
    /// Compute mass properties from `<inertial>` (or derive from geoms when
    /// the compiler asks for it). Adds `<joint armature>` contributions.
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
        let mut mp = if force_geom || auto_geom {
            self.geoms_to_mass_props(entry).or(from_inertial)
        } else {
            from_inertial
        };
        // Add armature contributions along each joint axis. For an axis
        // `a`, the rotor inertia is `armature * outer(a, a)`, applied to
        // the link's inertia tensor in the body frame.
        for j in &entry.body.joints {
            if j.armature <= 0.0 {
                continue;
            }
            let axis = match j.type_ {
                mb::JointType::Hinge | mb::JointType::Slide => Some(Vector::new(
                    j.axis[0] as Real,
                    j.axis[1] as Real,
                    j.axis[2] as Real,
                )),
                _ => None,
            };
            let Some(axis) = axis else { continue };
            let n = axis.length();
            if n < 1e-30 {
                continue;
            }
            let a = axis / n;
            let arm = j.armature as Real;
            let extra = Matrix::from_cols_array(&[
                arm * a.x * a.x,
                arm * a.x * a.y,
                arm * a.x * a.z,
                arm * a.x * a.y,
                arm * a.y * a.y,
                arm * a.y * a.z,
                arm * a.x * a.z,
                arm * a.y * a.z,
                arm * a.z * a.z,
            ]);
            let cur = mp.unwrap_or_default();
            let cur_inertia = cur.reconstruct_inertia_matrix();
            let new_inertia = cur_inertia + extra;
            mp = Some(MassProperties::with_inertia_matrix(
                cur.local_com,
                cur.mass(),
                new_inertia,
            ));
        }
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
        i.pose.pos[0] as Real * scale,
        i.pose.pos[1] as Real * scale,
        i.pose.pos[2] as Real * scale,
    );
    let s2 = scale * scale;
    match i.inertia {
        mb::InertiaSpec::Diagonal(d) => {
            let principal = Vector::new(d[0] as Real * s2, d[1] as Real * s2, d[2] as Real * s2);
            let r = mpose_to_rapier(i.pose).rotation;
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
