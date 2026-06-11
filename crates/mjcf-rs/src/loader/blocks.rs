//! Top-level block parsers: `<contact>`, `<equality>`, `<actuator>`,
//! `<sensor>`, `<keyframe>`. Each runs in pass 2 of `process_root_children`,
//! after the `<default>` / `<asset>` tables are fully populated.

use roxmltree::Node;

use crate::Pose;
use crate::contact::{ContactExclude, ContactPair};
use crate::equality::{Equality, EqualityCommon, EqualityConnect, EqualityWeld};
use crate::error::ParseError;
use crate::extras::{Actuator, ActuatorKind, Keyframe, Sensor};
use crate::types::Tristate;
use glamx::glam::{DQuat, DVec3};

use super::parse_utils::{
    parse_bool, parse_f64, parse_f64_list, parse_friction, parse_gear, parse_tristate, parse_u32,
    parse_vec2, parse_vec3,
};
use super::state::ParseState;

impl ParseState {
    pub(super) fn parse_contact(&mut self, node: Node) -> Result<(), ParseError> {
        for child in node.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "pair" => {
                    let class = child.attribute("class").map(|s| s.to_string());
                    let proto = self.merged_pair_proto(class.as_deref());
                    let mut p = ContactPair {
                        class,
                        condim: proto.condim,
                        friction: proto.friction,
                        margin: proto.margin,
                        gap: proto.gap,
                        ..Default::default()
                    };
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => p.name = Some(attr.value().to_string()),
                            "geom1" => p.geom1 = attr.value().to_string(),
                            "geom2" => p.geom2 = attr.value().to_string(),
                            "condim" => p.condim = Some(parse_u32(attr.value())?),
                            "friction" => p.friction = Some(parse_friction(attr.value())?),
                            "margin" => p.margin = Some(parse_f64(attr.value())?),
                            "gap" => p.gap = Some(parse_f64(attr.value())?),
                            _ => {}
                        }
                    }
                    self.model.contact.pairs.push(p);
                }
                "exclude" => {
                    let mut e = ContactExclude::default();
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => e.name = Some(attr.value().to_string()),
                            "body1" => e.body1 = attr.value().to_string(),
                            "body2" => e.body2 = attr.value().to_string(),
                            _ => {}
                        }
                    }
                    self.model.contact.excludes.push(e);
                }
                "include" => self.parse_include(child, true)?,
                _ => {}
            }
        }
        Ok(())
    }

    pub(super) fn parse_equality(&mut self, node: Node) -> Result<(), ParseError> {
        for child in node.children().filter(|n| n.is_element()) {
            let tag = child.tag_name().name();
            let class = child.attribute("class").map(|s| s.to_string());
            let proto = self.merged_equality_proto(class.as_deref());
            let mut common = EqualityCommon {
                class,
                active: proto.active.unwrap_or(true),
                ..Default::default()
            };
            for attr in child.attributes() {
                match attr.name() {
                    "name" => common.name = Some(attr.value().to_string()),
                    "active" => common.active = parse_bool(attr.value())?,
                    _ => {}
                }
            }
            match tag {
                "connect" => {
                    let mut c = EqualityConnect {
                        common,
                        ..Default::default()
                    };
                    for attr in child.attributes() {
                        match attr.name() {
                            "body1" => c.body1 = attr.value().to_string(),
                            "body2" => c.body2 = Some(attr.value().to_string()),
                            "anchor" => {
                                c.anchor = parse_vec3(attr.value(), "connect", "anchor")?;
                            }
                            _ => {}
                        }
                    }
                    self.model.equality.push(Equality::Connect(c));
                }
                "weld" => {
                    let mut w = EqualityWeld {
                        common,
                        torque_scale: 1.0,
                        ..Default::default()
                    };
                    for attr in child.attributes() {
                        match attr.name() {
                            "body1" => w.body1 = attr.value().to_string(),
                            "body2" => w.body2 = Some(attr.value().to_string()),
                            "anchor" => {
                                w.anchor = Some(parse_vec3(attr.value(), "weld", "anchor")?)
                            }
                            "relpose" => {
                                let v = parse_f64_list(attr.value())?;
                                if v.len() == 7 {
                                    // `relpose` is `pos(3) quat(w, x, y, z)(4)`.
                                    let pose = Pose::from_parts(
                                        DVec3::new(v[0], v[1], v[2]),
                                        DQuat::from_xyzw(v[4], v[5], v[6], v[3]),
                                    );
                                    w.relpose = Some(pose);
                                }
                            }
                            "torquescale" => w.torque_scale = parse_f64(attr.value())?,
                            _ => {}
                        }
                    }
                    self.model.equality.push(Equality::Weld(w));
                }
                "joint" | "tendon" | "flex" | "flexvert" | "flexstrain" => {
                    log::debug!("equality `{tag}` is out of scope; skipped");
                }
                "include" => self.parse_include(child, true)?,
                _ => {}
            }
        }
        Ok(())
    }

    pub(super) fn parse_actuator_block(&mut self, node: Node) -> Result<(), ParseError> {
        for child in node.children().filter(|n| n.is_element()) {
            let tag = child.tag_name().name();
            let kind = match tag {
                "motor" => ActuatorKind::Motor,
                "position" => ActuatorKind::Position,
                "velocity" => ActuatorKind::Velocity,
                "intvelocity" => ActuatorKind::IntVelocity,
                "damper" => ActuatorKind::Damper,
                "general" => ActuatorKind::General,
                "include" => {
                    self.parse_include(child, true)?;
                    continue;
                }
                _ => ActuatorKind::Other,
            };
            let class = child.attribute("class").map(|s| s.to_string());
            let proto = self.merged_actuator_proto(kind, class.as_deref());
            let mut a = Actuator {
                kind,
                class,
                gear: proto.gear.unwrap_or([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ctrl_range: proto.ctrl_range,
                force_range: proto.force_range,
                ctrl_limited: proto.ctrl_limited.unwrap_or(Tristate::Auto),
                force_limited: proto.force_limited.unwrap_or(Tristate::Auto),
                gainprm: proto.gainprm.unwrap_or_default(),
                biasprm: proto.biasprm.unwrap_or_default(),
                gain_type: proto.gain_type.clone(),
                bias_type: proto.bias_type.clone(),
                dyn_type: proto.dyn_type.clone(),
                dynprm: proto.dynprm.unwrap_or_default(),
                kp: proto.kp,
                kv: proto.kv,
                ..Default::default()
            };
            for attr in child.attributes() {
                match attr.name() {
                    "name" => a.name = Some(attr.value().to_string()),
                    "joint" => a.joint = Some(attr.value().to_string()),
                    "tendon" => a.tendon = Some(attr.value().to_string()),
                    "body" => a.body = Some(attr.value().to_string()),
                    "site" => a.site = Some(attr.value().to_string()),
                    "gear" => a.gear = parse_gear(attr.value())?,
                    "ctrlrange" => a.ctrl_range = Some(parse_vec2(attr.value(), tag, "ctrlrange")?),
                    "forcerange" => {
                        a.force_range = Some(parse_vec2(attr.value(), tag, "forcerange")?)
                    }
                    "ctrllimited" => a.ctrl_limited = parse_tristate(attr.value())?,
                    "forcelimited" => a.force_limited = parse_tristate(attr.value())?,
                    "gainprm" => a.gainprm = parse_f64_list(attr.value())?,
                    "biasprm" => a.biasprm = parse_f64_list(attr.value())?,
                    "gaintype" => a.gain_type = Some(attr.value().to_string()),
                    "biastype" => a.bias_type = Some(attr.value().to_string()),
                    "dyntype" => a.dyn_type = Some(attr.value().to_string()),
                    "dynprm" => a.dynprm = parse_f64_list(attr.value())?,
                    "kp" => a.kp = Some(parse_f64(attr.value())?),
                    "kv" => a.kv = Some(parse_f64(attr.value())?),
                    _ => {}
                }
            }
            self.model.actuators.push(a);
        }
        Ok(())
    }

    pub(super) fn parse_sensor_block(&mut self, node: Node) -> Result<(), ParseError> {
        for child in node.children().filter(|n| n.is_element()) {
            let tag = child.tag_name().name();
            if tag == "include" {
                self.parse_include(child, true)?;
                continue;
            }
            let mut s = Sensor {
                kind: tag.to_string(),
                ..Default::default()
            };
            for attr in child.attributes() {
                match attr.name() {
                    "name" => s.name = Some(attr.value().to_string()),
                    "class" => s.class = Some(attr.value().to_string()),
                    "objtype" => s.objtype = Some(attr.value().to_string()),
                    "objname" => s.objname = Some(attr.value().to_string()),
                    "reftype" => s.reftype = Some(attr.value().to_string()),
                    "refname" => s.refname = Some(attr.value().to_string()),
                    "site" => {
                        // some sensors take a `site=` shortcut for objtype/objname
                        s.objtype = Some("site".to_string());
                        s.objname = Some(attr.value().to_string());
                    }
                    "body" => {
                        s.objtype = Some("body".to_string());
                        s.objname = Some(attr.value().to_string());
                    }
                    "joint" => {
                        s.objtype = Some("joint".to_string());
                        s.objname = Some(attr.value().to_string());
                    }
                    "geom" => {
                        s.objtype = Some("geom".to_string());
                        s.objname = Some(attr.value().to_string());
                    }
                    "cutoff" => s.cutoff = Some(parse_f64(attr.value())?),
                    "noise" => s.noise = Some(parse_f64(attr.value())?),
                    _ => {}
                }
            }
            self.model.sensors.push(s);
        }
        Ok(())
    }

    pub(super) fn parse_keyframe_block(&mut self, node: Node) -> Result<(), ParseError> {
        for child in node.children().filter(|n| n.is_element()) {
            if child.tag_name().name() == "include" {
                self.parse_include(child, true)?;
                continue;
            }
            if child.tag_name().name() != "key" {
                continue;
            }
            let mut k = Keyframe::default();
            for attr in child.attributes() {
                match attr.name() {
                    "name" => k.name = Some(attr.value().to_string()),
                    "time" => k.time = parse_f64(attr.value())?,
                    "qpos" => k.qpos = parse_f64_list(attr.value())?,
                    "qvel" => k.qvel = parse_f64_list(attr.value())?,
                    "act" => k.act = parse_f64_list(attr.value())?,
                    "ctrl" => k.ctrl = parse_f64_list(attr.value())?,
                    "mpos" => k.mpos = parse_f64_list(attr.value())?,
                    "mquat" => k.mquat = parse_f64_list(attr.value())?,
                    _ => {}
                }
            }
            self.model.keyframes.push(k);
        }
        Ok(())
    }
}
