//! Per-element prototype structs and the merge helpers that overlay one
//! prototype on top of another. A prototype mirrors the AST element but
//! every field is `Option<...>` so the loader can tell whether a given
//! default *set* the field or not. Class-default inheritance walks from
//! "main" outward, merging at each step before the instance's own
//! attributes are applied on top.

use crate::Pose;
use crate::assets::MeshInertia;
use crate::body::{GeomType, JointType, SiteType};
use crate::types::Tristate;

#[derive(Default, Clone)]
pub(super) struct DefaultsClass {
    #[allow(dead_code)]
    pub(super) parent: Option<String>,
    pub(super) joint: Option<JointPrototype>,
    pub(super) geom: Option<GeomPrototype>,
    pub(super) site: Option<SitePrototype>,
    pub(super) mesh: Option<MeshPrototype>,
    pub(super) pair: Option<PairPrototype>,
    pub(super) equality: Option<EqualityPrototype>,
    pub(super) general: Option<ActuatorPrototype>,
    pub(super) motor: Option<ActuatorPrototype>,
    pub(super) position: Option<ActuatorPrototype>,
    pub(super) velocity: Option<ActuatorPrototype>,
    pub(super) intvelocity: Option<ActuatorPrototype>,
    pub(super) damper: Option<ActuatorPrototype>,
    pub(super) material: Option<MaterialPrototype>,
}

/// Per-class default `<material>` attributes. Mirrors the numeric fields of
/// [`crate::assets::Material`] as `Option`s so a class default only fills the
/// attributes the material itself didn't set.
#[derive(Default, Clone)]
pub(super) struct MaterialPrototype {
    pub(super) texture: Option<String>,
    pub(super) rgba: Option<[f64; 4]>,
    pub(super) emission: Option<f64>,
    pub(super) specular: Option<f64>,
    pub(super) shininess: Option<f64>,
    pub(super) roughness: Option<f64>,
    pub(super) metallic: Option<f64>,
}

#[derive(Default, Clone)]
pub(super) struct JointPrototype {
    pub(super) type_: Option<JointType>,
    pub(super) pos: Option<[f64; 3]>,
    pub(super) axis: Option<[f64; 3]>,
    pub(super) limited: Option<Tristate>,
    pub(super) range: Option<[f64; 2]>,
    pub(super) stiffness: Option<f64>,
    pub(super) damping: Option<f64>,
    pub(super) springref: Option<f64>,
    pub(super) springdamper: Option<[f64; 2]>,
    pub(super) armature: Option<f64>,
    pub(super) frictionloss: Option<f64>,
    pub(super) ref_: Option<f64>,
    pub(super) margin: Option<f64>,
}

#[derive(Default, Clone)]
pub(super) struct GeomPrototype {
    pub(super) type_: Option<GeomType>,
    pub(super) size: Option<[f64; 3]>,
    pub(super) pose: Option<Pose>,
    pub(super) fromto: Option<[f64; 6]>,
    pub(super) friction: Option<[f64; 3]>,
    pub(super) mass: Option<f64>,
    pub(super) density: Option<f64>,
    pub(super) margin: Option<f64>,
    pub(super) contype: Option<u32>,
    pub(super) conaffinity: Option<u32>,
    pub(super) condim: Option<u32>,
    pub(super) group: Option<i32>,
    pub(super) priority: Option<i32>,
    pub(super) rgba: Option<[f64; 4]>,
    pub(super) material: Option<String>,
    pub(super) mesh: Option<String>,
    pub(super) hfield: Option<String>,
}

#[derive(Default, Clone)]
pub(super) struct SitePrototype {
    pub(super) pose: Option<Pose>,
    pub(super) size: Option<[f64; 3]>,
    pub(super) rgba: Option<[f64; 4]>,
    pub(super) material: Option<String>,
    pub(super) type_: Option<SiteType>,
}

#[derive(Default, Clone)]
pub(super) struct MeshPrototype {
    pub(super) scale: Option<[f64; 3]>,
    pub(super) inertia: Option<MeshInertia>,
    pub(super) refpose: Option<Pose>,
    pub(super) max_hull_vert: Option<u32>,
    pub(super) smoothnormal: Option<f64>,
}

#[derive(Default, Clone)]
pub(super) struct PairPrototype {
    pub(super) condim: Option<u32>,
    pub(super) friction: Option<[f64; 3]>,
    pub(super) margin: Option<f64>,
    pub(super) gap: Option<f64>,
}

#[derive(Default, Clone)]
pub(super) struct EqualityPrototype {
    pub(super) active: Option<bool>,
}

#[derive(Default, Clone)]
pub(super) struct ActuatorPrototype {
    pub(super) gear: Option<[f64; 6]>,
    pub(super) ctrl_range: Option<[f64; 2]>,
    pub(super) force_range: Option<[f64; 2]>,
    pub(super) ctrl_limited: Option<Tristate>,
    pub(super) force_limited: Option<Tristate>,
    pub(super) gainprm: Option<Vec<f64>>,
    pub(super) biasprm: Option<Vec<f64>>,
    pub(super) gain_type: Option<String>,
    pub(super) bias_type: Option<String>,
    pub(super) dyn_type: Option<String>,
    pub(super) dynprm: Option<Vec<f64>>,
    pub(super) kp: Option<f64>,
    pub(super) kv: Option<f64>,
}

pub(super) fn merge_joint_proto(
    base: Option<JointPrototype>,
    top: JointPrototype,
) -> JointPrototype {
    let mut acc = base.unwrap_or_default();
    if top.type_.is_some() {
        acc.type_ = top.type_;
    }
    if top.pos.is_some() {
        acc.pos = top.pos;
    }
    if top.axis.is_some() {
        acc.axis = top.axis;
    }
    if top.limited.is_some() {
        acc.limited = top.limited;
    }
    if top.range.is_some() {
        acc.range = top.range;
    }
    if top.stiffness.is_some() {
        acc.stiffness = top.stiffness;
    }
    if top.damping.is_some() {
        acc.damping = top.damping;
    }
    if top.springref.is_some() {
        acc.springref = top.springref;
    }
    if top.springdamper.is_some() {
        acc.springdamper = top.springdamper;
    }
    if top.armature.is_some() {
        acc.armature = top.armature;
    }
    if top.frictionloss.is_some() {
        acc.frictionloss = top.frictionloss;
    }
    if top.ref_.is_some() {
        acc.ref_ = top.ref_;
    }
    if top.margin.is_some() {
        acc.margin = top.margin;
    }
    acc
}

pub(super) fn merge_geom_proto(base: Option<GeomPrototype>, top: GeomPrototype) -> GeomPrototype {
    let mut acc = base.unwrap_or_default();
    macro_rules! over {
        ($f:ident) => {
            if top.$f.is_some() {
                acc.$f = top.$f;
            }
        };
    }
    over!(type_);
    over!(size);
    over!(pose);
    over!(fromto);
    over!(friction);
    over!(mass);
    over!(density);
    over!(margin);
    over!(contype);
    over!(conaffinity);
    over!(condim);
    over!(group);
    over!(priority);
    over!(rgba);
    if top.material.is_some() {
        acc.material = top.material;
    }
    if top.mesh.is_some() {
        acc.mesh = top.mesh;
    }
    if top.hfield.is_some() {
        acc.hfield = top.hfield;
    }
    acc
}

pub(super) fn merge_material_proto(
    base: Option<MaterialPrototype>,
    top: MaterialPrototype,
) -> MaterialPrototype {
    let mut acc = base.unwrap_or_default();
    macro_rules! over {
        ($f:ident) => {
            if top.$f.is_some() {
                acc.$f = top.$f;
            }
        };
    }
    over!(texture);
    over!(rgba);
    over!(emission);
    over!(specular);
    over!(shininess);
    over!(roughness);
    over!(metallic);
    acc
}

pub(super) fn merge_site_proto(base: Option<SitePrototype>, top: SitePrototype) -> SitePrototype {
    let mut acc = base.unwrap_or_default();
    if top.pose.is_some() {
        acc.pose = top.pose;
    }
    if top.size.is_some() {
        acc.size = top.size;
    }
    if top.rgba.is_some() {
        acc.rgba = top.rgba;
    }
    if top.material.is_some() {
        acc.material = top.material;
    }
    if top.type_.is_some() {
        acc.type_ = top.type_;
    }
    acc
}

pub(super) fn merge_mesh_proto(base: Option<MeshPrototype>, top: MeshPrototype) -> MeshPrototype {
    let mut acc = base.unwrap_or_default();
    if top.scale.is_some() {
        acc.scale = top.scale;
    }
    if top.inertia.is_some() {
        acc.inertia = top.inertia;
    }
    if top.refpose.is_some() {
        acc.refpose = top.refpose;
    }
    if top.max_hull_vert.is_some() {
        acc.max_hull_vert = top.max_hull_vert;
    }
    if top.smoothnormal.is_some() {
        acc.smoothnormal = top.smoothnormal;
    }
    acc
}

pub(super) fn merge_pair_proto(base: Option<PairPrototype>, top: PairPrototype) -> PairPrototype {
    let mut acc = base.unwrap_or_default();
    if top.condim.is_some() {
        acc.condim = top.condim;
    }
    if top.friction.is_some() {
        acc.friction = top.friction;
    }
    if top.margin.is_some() {
        acc.margin = top.margin;
    }
    if top.gap.is_some() {
        acc.gap = top.gap;
    }
    acc
}

pub(super) fn merge_equality_proto(
    base: Option<EqualityPrototype>,
    top: EqualityPrototype,
) -> EqualityPrototype {
    let mut acc = base.unwrap_or_default();
    if top.active.is_some() {
        acc.active = top.active;
    }
    acc
}

pub(super) fn merge_actuator_proto(
    base: Option<ActuatorPrototype>,
    top: ActuatorPrototype,
) -> ActuatorPrototype {
    let mut acc = base.unwrap_or_default();
    if top.gear.is_some() {
        acc.gear = top.gear;
    }
    if top.ctrl_range.is_some() {
        acc.ctrl_range = top.ctrl_range;
    }
    if top.force_range.is_some() {
        acc.force_range = top.force_range;
    }
    if top.ctrl_limited.is_some() {
        acc.ctrl_limited = top.ctrl_limited;
    }
    if top.force_limited.is_some() {
        acc.force_limited = top.force_limited;
    }
    if top.gainprm.is_some() {
        acc.gainprm = top.gainprm;
    }
    if top.biasprm.is_some() {
        acc.biasprm = top.biasprm;
    }
    if top.gain_type.is_some() {
        acc.gain_type = top.gain_type;
    }
    if top.bias_type.is_some() {
        acc.bias_type = top.bias_type;
    }
    if top.dyn_type.is_some() {
        acc.dyn_type = top.dyn_type;
    }
    if top.dynprm.is_some() {
        acc.dynprm = top.dynprm;
    }
    if top.kp.is_some() {
        acc.kp = top.kp;
    }
    if top.kv.is_some() {
        acc.kv = top.kv;
    }
    acc
}
