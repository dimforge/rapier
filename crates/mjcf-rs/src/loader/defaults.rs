//! `<default>` class parsing + class-chain lookup + per-element merged
//! prototype assembly. The output of this module is what
//! `bodies.rs` / `blocks.rs` consume to resolve concrete elements
//! against their inherited defaults.

use roxmltree::Node;

use crate::Pose;
use crate::assets::MeshInertia;
use crate::body::SiteType;
use crate::error::ParseError;
use crate::extras::ActuatorKind;
use glamx::glam::{DQuat, DVec3};

use super::parse_utils::{
    parse_bool, parse_f64, parse_f64_list, parse_friction, parse_gear, parse_geom_type, parse_i32,
    parse_joint_type, parse_quat, parse_rotation_attr, parse_tristate, parse_u32, parse_vec2,
    parse_vec3, parse_vec3_lax, parse_vec4, parse_vec6,
};
use super::prototypes::{
    ActuatorPrototype, DefaultsClass, EqualityPrototype, GeomPrototype, JointPrototype,
    MeshPrototype, PairPrototype, SitePrototype, merge_actuator_proto, merge_equality_proto,
    merge_geom_proto, merge_joint_proto, merge_mesh_proto, merge_pair_proto, merge_site_proto,
};
use super::state::ParseState;

impl ParseState {
    pub(super) fn parse_default(
        &mut self,
        node: Node,
        parent_class: Option<&str>,
    ) -> Result<(), ParseError> {
        let class_name = node
            .attribute("class")
            .map(|s| s.to_string())
            .unwrap_or_else(|| {
                parent_class
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| "main".to_string())
            });
        // Insert / take existing.
        let mut class = self.defaults.remove(&class_name).unwrap_or_default();
        class.parent = parent_class.map(|s| s.to_string());
        self.defaults_parent_of
            .insert(class_name.clone(), parent_class.map(|s| s.to_string()));

        for child in node.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "joint" => {
                    let p = self.parse_joint_prototype(child)?;
                    class.joint = Some(merge_joint_proto(class.joint.clone(), p));
                }
                "geom" => {
                    let p = self.parse_geom_prototype(child)?;
                    class.geom = Some(merge_geom_proto(class.geom.clone(), p));
                }
                "site" => {
                    let p = self.parse_site_prototype(child)?;
                    class.site = Some(merge_site_proto(class.site.clone(), p));
                }
                "mesh" => {
                    let p = self.parse_mesh_prototype(child)?;
                    class.mesh = Some(merge_mesh_proto(class.mesh.clone(), p));
                }
                "pair" => {
                    let p = self.parse_pair_prototype(child)?;
                    class.pair = Some(merge_pair_proto(class.pair.clone(), p));
                }
                "equality" => {
                    let p = self.parse_equality_prototype(child)?;
                    class.equality = Some(merge_equality_proto(class.equality.clone(), p));
                }
                "general" | "motor" | "position" | "velocity" | "intvelocity" | "damper" => {
                    let p = self.parse_actuator_prototype(child)?;
                    let slot = match child.tag_name().name() {
                        "motor" => &mut class.motor,
                        "position" => &mut class.position,
                        "velocity" => &mut class.velocity,
                        "intvelocity" => &mut class.intvelocity,
                        "damper" => &mut class.damper,
                        _ => &mut class.general,
                    };
                    *slot = Some(merge_actuator_proto(slot.clone(), p));
                }
                "default" => {
                    // Nested default — store the outer first, recurse, then re-insert.
                    self.defaults.insert(class_name.clone(), class.clone());
                    self.parse_default(child, Some(&class_name))?;
                    class = self.defaults.remove(&class_name).unwrap();
                }
                _ => {
                    // <camera>/<light>/<tendon>/<pair>/<exclude>/<numeric>/etc — ignored for now.
                }
            }
        }
        self.defaults.insert(class_name, class);
        Ok(())
    }

    pub(super) fn parse_joint_prototype(&self, node: Node) -> Result<JointPrototype, ParseError> {
        let mut p = JointPrototype::default();
        for attr in node.attributes() {
            match attr.name() {
                "type" => p.type_ = Some(parse_joint_type(attr.value())?),
                "pos" => p.pos = Some(parse_vec3(attr.value(), "joint", "pos")?),
                "axis" => p.axis = Some(parse_vec3(attr.value(), "joint", "axis")?),
                "limited" => p.limited = Some(parse_tristate(attr.value())?),
                "range" => {
                    let v = parse_vec2(attr.value(), "joint", "range")?;
                    p.range = Some(v);
                }
                "stiffness" => p.stiffness = Some(parse_f64(attr.value())?),
                "damping" => p.damping = Some(parse_f64(attr.value())?),
                "springref" => p.springref = Some(parse_f64(attr.value())?),
                "springdamper" => {
                    p.springdamper = Some(parse_vec2(attr.value(), "joint", "springdamper")?)
                }
                "armature" => p.armature = Some(parse_f64(attr.value())?),
                "frictionloss" => p.frictionloss = Some(parse_f64(attr.value())?),
                "ref" => p.ref_ = Some(parse_f64(attr.value())?),
                "margin" => p.margin = Some(parse_f64(attr.value())?),
                _ => {}
            }
        }
        // angle-units: range/springref/ref are in degrees if compiler.angle_is_degree.
        // We don't know the joint type at default-prototype time, so we apply
        // unit conversion lazily during instance resolution. Defer.
        Ok(p)
    }

    pub(super) fn parse_geom_prototype(&self, node: Node) -> Result<GeomPrototype, ParseError> {
        let mut p = GeomPrototype::default();
        let mut pose_pos = None;
        let mut pose_rot = None;
        for attr in node.attributes() {
            match attr.name() {
                "type" => p.type_ = Some(parse_geom_type(attr.value())?),
                "size" => p.size = Some(parse_vec3_lax(attr.value(), "geom", "size")?),
                "pos" => pose_pos = Some(parse_vec3(attr.value(), "geom", "pos")?),
                "quat" => pose_rot = Some(parse_quat(attr.value(), "geom", "quat")?),
                "axisangle" | "euler" | "xyaxes" | "zaxis" => {
                    pose_rot = Some(parse_rotation_attr(
                        attr.name(),
                        attr.value(),
                        "geom",
                        &self.model.compiler,
                    )?);
                }
                "fromto" => p.fromto = Some(parse_vec6(attr.value(), "geom", "fromto")?),
                "friction" => p.friction = Some(parse_friction(attr.value())?),
                "mass" => p.mass = Some(parse_f64(attr.value())?),
                "density" => p.density = Some(parse_f64(attr.value())?),
                "margin" => p.margin = Some(parse_f64(attr.value())?),
                "contype" => p.contype = Some(parse_u32(attr.value())?),
                "conaffinity" => p.conaffinity = Some(parse_u32(attr.value())?),
                "condim" => p.condim = Some(parse_u32(attr.value())?),
                "group" => p.group = Some(parse_i32(attr.value())?),
                "priority" => p.priority = Some(parse_i32(attr.value())?),
                "rgba" => p.rgba = Some(parse_vec4(attr.value(), "geom", "rgba")?),
                "material" => p.material = Some(attr.value().to_string()),
                "mesh" => p.mesh = Some(attr.value().to_string()),
                "hfield" => p.hfield = Some(attr.value().to_string()),
                _ => {}
            }
        }
        if pose_pos.is_some() || pose_rot.is_some() {
            p.pose = Some(Pose::from_parts(
                DVec3::from_array(pose_pos.unwrap_or([0.0; 3])),
                pose_rot.unwrap_or(DQuat::IDENTITY),
            ));
        }
        Ok(p)
    }

    pub(super) fn parse_site_prototype(&self, node: Node) -> Result<SitePrototype, ParseError> {
        let mut p = SitePrototype::default();
        let mut pose_pos = None;
        let mut pose_rot = None;
        for attr in node.attributes() {
            match attr.name() {
                "type" => {
                    p.type_ = Some(match attr.value() {
                        "sphere" => SiteType::Sphere,
                        "capsule" => SiteType::Capsule,
                        "ellipsoid" => SiteType::Ellipsoid,
                        "cylinder" => SiteType::Cylinder,
                        "box" => SiteType::Box,
                        v => {
                            return Err(ParseError::BadAttributeValue {
                                tag: "site".to_string(),
                                attr: "type".to_string(),
                                value: v.to_string(),
                                message: "unsupported site type".to_string(),
                            });
                        }
                    })
                }
                "size" => p.size = Some(parse_vec3_lax(attr.value(), "site", "size")?),
                "pos" => pose_pos = Some(parse_vec3(attr.value(), "site", "pos")?),
                "quat" => pose_rot = Some(parse_quat(attr.value(), "site", "quat")?),
                "axisangle" | "euler" | "xyaxes" | "zaxis" => {
                    pose_rot = Some(parse_rotation_attr(
                        attr.name(),
                        attr.value(),
                        "site",
                        &self.model.compiler,
                    )?);
                }
                "rgba" => p.rgba = Some(parse_vec4(attr.value(), "site", "rgba")?),
                "material" => p.material = Some(attr.value().to_string()),
                _ => {}
            }
        }
        if pose_pos.is_some() || pose_rot.is_some() {
            p.pose = Some(Pose::from_parts(
                DVec3::from_array(pose_pos.unwrap_or([0.0; 3])),
                pose_rot.unwrap_or(DQuat::IDENTITY),
            ));
        }
        Ok(p)
    }

    pub(super) fn parse_mesh_prototype(&self, node: Node) -> Result<MeshPrototype, ParseError> {
        let mut p = MeshPrototype::default();
        let mut pose_pos = None;
        let mut pose_rot = None;
        for attr in node.attributes() {
            match attr.name() {
                "scale" => p.scale = Some(parse_vec3(attr.value(), "mesh", "scale")?),
                "inertia" => {
                    p.inertia = Some(match attr.value() {
                        "shell" => MeshInertia::Shell,
                        "convex" => MeshInertia::Convex,
                        "exact" => MeshInertia::Exact,
                        "legacy" => MeshInertia::Legacy,
                        v => {
                            return Err(ParseError::BadAttributeValue {
                                tag: "mesh".to_string(),
                                attr: "inertia".to_string(),
                                value: v.to_string(),
                                message: "expected shell/convex/exact/legacy".to_string(),
                            });
                        }
                    })
                }
                "refpos" => pose_pos = Some(parse_vec3(attr.value(), "mesh", "refpos")?),
                "refquat" => pose_rot = Some(parse_quat(attr.value(), "mesh", "refquat")?),
                "maxhullvert" => p.max_hull_vert = Some(parse_u32(attr.value())?),
                "smoothnormal" => p.smoothnormal = Some(parse_f64(attr.value())?),
                _ => {}
            }
        }
        if pose_pos.is_some() || pose_rot.is_some() {
            p.refpose = Some(Pose::from_parts(
                DVec3::from_array(pose_pos.unwrap_or([0.0; 3])),
                pose_rot.unwrap_or(DQuat::IDENTITY),
            ));
        }
        Ok(p)
    }

    pub(super) fn parse_pair_prototype(&self, node: Node) -> Result<PairPrototype, ParseError> {
        let mut p = PairPrototype::default();
        for attr in node.attributes() {
            match attr.name() {
                "condim" => p.condim = Some(parse_u32(attr.value())?),
                "friction" => p.friction = Some(parse_friction(attr.value())?),
                "margin" => p.margin = Some(parse_f64(attr.value())?),
                "gap" => p.gap = Some(parse_f64(attr.value())?),
                _ => {}
            }
        }
        Ok(p)
    }

    pub(super) fn parse_equality_prototype(
        &self,
        node: Node,
    ) -> Result<EqualityPrototype, ParseError> {
        let mut p = EqualityPrototype::default();
        for attr in node.attributes() {
            if attr.name() == "active" {
                p.active = Some(parse_bool(attr.value())?);
            }
        }
        Ok(p)
    }

    pub(super) fn parse_actuator_prototype(
        &self,
        node: Node,
    ) -> Result<ActuatorPrototype, ParseError> {
        let mut p = ActuatorPrototype::default();
        for attr in node.attributes() {
            match attr.name() {
                "gear" => p.gear = Some(parse_gear(attr.value())?),
                "ctrlrange" => {
                    p.ctrl_range = Some(parse_vec2(attr.value(), "actuator", "ctrlrange")?)
                }
                "forcerange" => {
                    p.force_range = Some(parse_vec2(attr.value(), "actuator", "forcerange")?)
                }
                "ctrllimited" => p.ctrl_limited = Some(parse_tristate(attr.value())?),
                "forcelimited" => p.force_limited = Some(parse_tristate(attr.value())?),
                "gainprm" => p.gainprm = Some(parse_f64_list(attr.value())?),
                "biasprm" => p.biasprm = Some(parse_f64_list(attr.value())?),
                "gaintype" => p.gain_type = Some(attr.value().to_string()),
                "biastype" => p.bias_type = Some(attr.value().to_string()),
                "dyntype" => p.dyn_type = Some(attr.value().to_string()),
                "dynprm" => p.dynprm = Some(parse_f64_list(attr.value())?),
                "kp" => p.kp = Some(parse_f64(attr.value())?),
                "kv" => p.kv = Some(parse_f64(attr.value())?),
                _ => {}
            }
        }
        Ok(p)
    }

    /// Returns the chain of class names from the deepest to "main".
    pub(super) fn class_chain(&self, name: Option<&str>) -> Vec<String> {
        let mut out = Vec::new();
        let mut cur = name.map(|s| s.to_string());
        if cur.is_none() {
            cur = Some("main".to_string());
        }
        let mut visited = std::collections::HashSet::new();
        while let Some(c) = cur {
            if !visited.insert(c.clone()) {
                break;
            }
            cur = self.defaults_parent_of.get(&c).cloned().flatten();
            out.push(c);
        }
        // Always end with "main".
        if !out.iter().any(|s| s == "main") {
            out.push("main".to_string());
        }
        out
    }

    pub(super) fn merged_joint_proto(&self, class: Option<&str>) -> JointPrototype {
        let mut acc = JointPrototype::default();
        // Walk from the most-general (main) to the most-specific.
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.joint
            {
                acc = merge_joint_proto(Some(acc), p.clone());
            }
        }
        acc
    }

    pub(super) fn merged_geom_proto(&self, class: Option<&str>) -> GeomPrototype {
        let mut acc = GeomPrototype::default();
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.geom
            {
                acc = merge_geom_proto(Some(acc), p.clone());
            }
        }
        acc
    }

    pub(super) fn merged_site_proto(&self, class: Option<&str>) -> SitePrototype {
        let mut acc = SitePrototype::default();
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.site
            {
                acc = merge_site_proto(Some(acc), p.clone());
            }
        }
        acc
    }

    pub(super) fn merged_mesh_proto(&self, class: Option<&str>) -> MeshPrototype {
        let mut acc = MeshPrototype::default();
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.mesh
            {
                acc = merge_mesh_proto(Some(acc), p.clone());
            }
        }
        acc
    }

    pub(super) fn merged_actuator_proto(
        &self,
        kind: ActuatorKind,
        class: Option<&str>,
    ) -> ActuatorPrototype {
        let mut acc = ActuatorPrototype::default();
        let pick = |d: &DefaultsClass| match kind {
            ActuatorKind::Motor => d.motor.clone(),
            ActuatorKind::Position => d.position.clone(),
            ActuatorKind::Velocity => d.velocity.clone(),
            ActuatorKind::IntVelocity => d.intvelocity.clone(),
            ActuatorKind::Damper => d.damper.clone(),
            _ => d.general.clone(),
        };
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c) {
                if let Some(p) = pick(d) {
                    acc = merge_actuator_proto(Some(acc), p);
                }
                if let Some(p) = &d.general {
                    acc = merge_actuator_proto(Some(acc), p.clone());
                }
            }
        }
        acc
    }

    pub(super) fn merged_pair_proto(&self, class: Option<&str>) -> PairPrototype {
        let mut acc = PairPrototype::default();
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.pair
            {
                acc = merge_pair_proto(Some(acc), p.clone());
            }
        }
        acc
    }

    pub(super) fn merged_equality_proto(&self, class: Option<&str>) -> EqualityPrototype {
        let mut acc = EqualityPrototype::default();
        for c in self.class_chain(class).into_iter().rev() {
            if let Some(d) = self.defaults.get(&c)
                && let Some(p) = &d.equality
            {
                acc = merge_equality_proto(Some(acc), p.clone());
            }
        }
        acc
    }
}
