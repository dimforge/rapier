//! Body-tree parsing: `<worldbody>`, `<body>`, `<frame>`, `<inertial>`,
//! and per-element resolvers (`<joint>`, `<geom>`, `<site>`) that
//! materialize the merged class defaults into concrete AST nodes.

use roxmltree::Node;

use crate::body::{Body, Geom, GeomType, InertiaSpec, Inertial, Joint, JointType, Site, SiteType};
use crate::error::ParseError;
use crate::model::{BodyEntry, BodyId};
use crate::pose::{Pose, pose_mul};
use crate::types::Tristate;

use super::parse_utils::{
    deg_to_rad, parse_bool, parse_f64, parse_f64_list, parse_quat, parse_rotation_attr, parse_vec3,
    parse_vec6,
};
use super::prototypes::{merge_geom_proto, merge_joint_proto, merge_site_proto};
use super::state::ParseState;

impl ParseState {
    pub(super) fn parse_worldbody(&mut self, node: Node) -> Result<(), ParseError> {
        // Worldbody can have <geom>, <site>, <body>, <frame>, <include> children.
        let world_id: BodyId = 0;
        // Geoms / sites attached to the world body.
        for child in node.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "geom" => {
                    let g = self.resolve_geom(child, /*childclass=*/ None)?;
                    self.model.bodies[world_id].body.geoms.push(g);
                }
                "site" => {
                    let s = self.resolve_site(child, /*childclass=*/ None)?;
                    self.model.bodies[world_id].body.sites.push(s);
                }
                "body" => {
                    self.parse_body(child, world_id, None, Pose::IDENTITY)?;
                }
                "frame" => {
                    self.parse_frame(child, world_id, None, Pose::IDENTITY)?;
                }
                "include" => self.parse_include(child, true)?,
                "attach" => {
                    self.parse_attach(child, world_id)?;
                }
                _ => {}
            }
        }
        Ok(())
    }

    pub(super) fn parse_body(
        &mut self,
        node: Node,
        parent_id: BodyId,
        ambient_childclass: Option<&str>,
        ambient_frame: Pose,
    ) -> Result<BodyId, ParseError> {
        let mut body = Body::default();
        let mut pose_pos = [0.0; 3];
        let mut pose_rot = [1.0, 0.0, 0.0, 0.0];
        let mut has_rot = false;
        for attr in node.attributes() {
            match attr.name() {
                "name" => body.name = Some(attr.value().to_string()),
                "class" => body.class = Some(attr.value().to_string()),
                "childclass" => body.childclass = Some(attr.value().to_string()),
                "mocap" => body.mocap = parse_bool(attr.value())?,
                "gravcomp" => body.gravcomp = parse_f64(attr.value())?,
                "sleep" => body.sleep = parse_bool(attr.value())?,
                "pos" => pose_pos = parse_vec3(attr.value(), "body", "pos")?,
                "quat" => {
                    pose_rot = parse_quat(attr.value(), "body", "quat")?;
                    has_rot = true;
                }
                "axisangle" | "euler" | "xyaxes" | "zaxis" => {
                    pose_rot = parse_rotation_attr(
                        attr.name(),
                        attr.value(),
                        "body",
                        &self.model.compiler,
                    )?;
                    has_rot = true;
                }
                "user" => body.user = parse_f64_list(attr.value())?,
                _ => {}
            }
        }
        let _ = has_rot;
        body.pose = pose_mul(
            ambient_frame,
            Pose {
                pos: pose_pos,
                quat: pose_rot,
            },
        );

        let childclass_for_children = body
            .childclass
            .clone()
            .or_else(|| ambient_childclass.map(|s| s.to_string()));

        // First pass: emit the body shell so child bodies can reference us as parent.
        let id = self.model.bodies.len();
        self.model.bodies.push(BodyEntry {
            parent: Some(parent_id),
            body,
        });

        // Now parse children using their effective class. Children of a body
        // (joints, geoms, sites, sub-bodies) inherit the body's `childclass`,
        // which itself falls back to the ambient childclass when unset.
        for child in node.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "inertial" => {
                    let i = self.parse_inertial(child)?;
                    self.model.bodies[id].body.inertial = Some(i);
                }
                "joint" => {
                    let j = self.resolve_joint(child, childclass_for_children.as_deref())?;
                    self.model.bodies[id].body.joints.push(j);
                }
                "freejoint" => {
                    let mut j = Joint::default();
                    j.type_ = JointType::Free;
                    j.name = child.attribute("name").map(|s| s.to_string());
                    self.model.bodies[id].body.joints.push(j);
                }
                "geom" => {
                    let g = self.resolve_geom(child, childclass_for_children.as_deref())?;
                    self.model.bodies[id].body.geoms.push(g);
                }
                "site" => {
                    let s = self.resolve_site(child, childclass_for_children.as_deref())?;
                    self.model.bodies[id].body.sites.push(s);
                }
                "body" => {
                    self.parse_body(
                        child,
                        id,
                        childclass_for_children.as_deref(),
                        Pose::IDENTITY,
                    )?;
                }
                "frame" => {
                    self.parse_frame(
                        child,
                        id,
                        childclass_for_children.as_deref(),
                        Pose::IDENTITY,
                    )?;
                }
                "include" => self.parse_include(child, true)?,
                "attach" => {
                    self.parse_attach(child, id)?;
                }
                "camera" | "light" | "composite" | "flexcomp" | "plugin" => {
                    // recorded as 📦 / out-of-scope, ignored for the loader.
                }
                _ => {}
            }
        }
        Ok(id)
    }

    pub(super) fn parse_frame(
        &mut self,
        node: Node,
        parent_id: BodyId,
        ambient_childclass: Option<&str>,
        ambient_frame: Pose,
    ) -> Result<(), ParseError> {
        let mut pos = [0.0; 3];
        let mut rot = [1.0, 0.0, 0.0, 0.0];
        let mut frame_class = None;
        for attr in node.attributes() {
            match attr.name() {
                "pos" => pos = parse_vec3(attr.value(), "frame", "pos")?,
                "quat" => rot = parse_quat(attr.value(), "frame", "quat")?,
                "axisangle" | "euler" | "xyaxes" | "zaxis" => {
                    rot = parse_rotation_attr(
                        attr.name(),
                        attr.value(),
                        "frame",
                        &self.model.compiler,
                    )?;
                }
                "childclass" => frame_class = Some(attr.value().to_string()),
                _ => {}
            }
        }
        let combined = pose_mul(ambient_frame, Pose { pos, quat: rot });
        let cclass = frame_class
            .as_deref()
            .or(ambient_childclass)
            .map(|s| s.to_string());
        for child in node.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "geom" => {
                    let mut g = self.resolve_geom(child, cclass.as_deref())?;
                    g.pose = pose_mul(combined, g.pose);
                    self.model.bodies[parent_id].body.geoms.push(g);
                }
                "site" => {
                    let mut s = self.resolve_site(child, cclass.as_deref())?;
                    s.pose = pose_mul(combined, s.pose);
                    self.model.bodies[parent_id].body.sites.push(s);
                }
                "body" => {
                    self.parse_body(child, parent_id, cclass.as_deref(), combined)?;
                }
                "frame" => {
                    self.parse_frame(child, parent_id, cclass.as_deref(), combined)?;
                }
                "include" => self.parse_include(child, true)?,
                _ => {}
            }
        }
        Ok(())
    }

    pub(super) fn parse_inertial(&self, node: Node) -> Result<Inertial, ParseError> {
        let mut i = Inertial::default();
        let mut pose_pos = [0.0; 3];
        let mut pose_rot = [1.0, 0.0, 0.0, 0.0];
        let mut diag: Option<[f64; 3]> = None;
        let mut full: Option<[f64; 6]> = None;
        for attr in node.attributes() {
            match attr.name() {
                "pos" => pose_pos = parse_vec3(attr.value(), "inertial", "pos")?,
                "quat" => pose_rot = parse_quat(attr.value(), "inertial", "quat")?,
                "axisangle" | "euler" | "xyaxes" | "zaxis" => {
                    pose_rot = parse_rotation_attr(
                        attr.name(),
                        attr.value(),
                        "inertial",
                        &self.model.compiler,
                    )?;
                }
                "mass" => i.mass = parse_f64(attr.value())?,
                "diaginertia" => diag = Some(parse_vec3(attr.value(), "inertial", "diaginertia")?),
                "fullinertia" => full = Some(parse_vec6(attr.value(), "inertial", "fullinertia")?),
                _ => {}
            }
        }
        i.pose = Pose {
            pos: pose_pos,
            quat: pose_rot,
        };
        i.inertia = if let Some(f) = full {
            InertiaSpec::Full(f)
        } else if let Some(d) = diag {
            InertiaSpec::Diagonal(d)
        } else {
            InertiaSpec::Diagonal([0.0; 3])
        };
        Ok(i)
    }

    /// Resolve a `<joint>` element into a fully-populated `Joint`, applying
    /// defaults from the class chain.
    pub(super) fn resolve_joint(
        &self,
        node: Node,
        ambient_class: Option<&str>,
    ) -> Result<Joint, ParseError> {
        let class = node
            .attribute("class")
            .or(ambient_class)
            .map(|s| s.to_string());
        let proto = self.merged_joint_proto(class.as_deref());
        let mut instance = self.parse_joint_prototype(node)?;
        instance = merge_joint_proto(Some(proto), instance);

        let mut j = Joint::default();
        j.name = node.attribute("name").map(|s| s.to_string());
        j.class = class;
        j.type_ = instance.type_.unwrap_or(JointType::Hinge);
        j.pos = instance.pos.unwrap_or([0.0; 3]);
        j.axis = instance.axis.unwrap_or([0.0, 0.0, 1.0]);
        j.limited = instance.limited.unwrap_or(Tristate::Auto);
        j.range = instance.range;
        j.stiffness = instance.stiffness.unwrap_or(0.0);
        j.damping = instance.damping.unwrap_or(0.0);
        j.springref = instance.springref.unwrap_or(0.0);
        j.springdamper = instance.springdamper;
        j.armature = instance.armature.unwrap_or(0.0);
        j.frictionloss = instance.frictionloss.unwrap_or(0.0);
        j.ref_ = instance.ref_.unwrap_or(0.0);
        j.margin = instance.margin.unwrap_or(0.0);

        // Convert hinge angle units from degrees if needed.
        if matches!(j.type_, JointType::Hinge) && self.model.compiler.angle_is_degree {
            if let Some(r) = j.range.as_mut() {
                r[0] = deg_to_rad(r[0]);
                r[1] = deg_to_rad(r[1]);
            }
            j.springref = deg_to_rad(j.springref);
            j.ref_ = deg_to_rad(j.ref_);
        }
        Ok(j)
    }

    pub(super) fn resolve_geom(
        &self,
        node: Node,
        ambient_class: Option<&str>,
    ) -> Result<Geom, ParseError> {
        let class = node
            .attribute("class")
            .or(ambient_class)
            .map(|s| s.to_string());
        let proto = self.merged_geom_proto(class.as_deref());
        let mut instance = self.parse_geom_prototype(node)?;
        instance = merge_geom_proto(Some(proto), instance);

        let mut g = Geom::default();
        g.name = node.attribute("name").map(|s| s.to_string());
        g.class = class;
        g.type_ = instance.type_.unwrap_or(GeomType::Sphere);
        g.size = instance.size.unwrap_or([0.0, 0.0, 0.0]);
        g.pose = instance.pose.unwrap_or_default();
        g.fromto = instance.fromto;
        g.friction = instance.friction.unwrap_or([1.0, 0.005, 0.0001]);
        g.mass = instance.mass;
        g.density = instance.density;
        g.margin = instance.margin.unwrap_or(0.0);
        g.contype = instance.contype.unwrap_or(1);
        g.conaffinity = instance.conaffinity.unwrap_or(1);
        g.condim = instance.condim.unwrap_or(3);
        g.group = instance.group.unwrap_or(0);
        g.priority = instance.priority.unwrap_or(0);
        g.rgba = instance.rgba;
        g.material = instance.material.clone();
        // `instance.mesh` / `instance.hfield` already reflect the
        // class-default chain (the prototype merge stored anything the
        // `<default class="X"><geom mesh="…"/></default>` chain
        // provided). The geom's own `mesh=` / `hfield=` attributes were
        // merged in by `parse_geom_prototype` and override the
        // inherited values.
        g.mesh = instance.mesh.clone();
        g.hfield = instance.hfield.clone();

        for attr in node.attributes() {
            if attr.name() == "user" {
                g.user = parse_f64_list(attr.value())?;
            }
        }
        Ok(g)
    }

    pub(super) fn resolve_site(
        &self,
        node: Node,
        ambient_class: Option<&str>,
    ) -> Result<Site, ParseError> {
        let class = node
            .attribute("class")
            .or(ambient_class)
            .map(|s| s.to_string());
        let proto = self.merged_site_proto(class.as_deref());
        let mut instance = self.parse_site_prototype(node)?;
        instance = merge_site_proto(Some(proto), instance);

        let mut s = Site::default();
        s.name = node.attribute("name").map(|s| s.to_string());
        s.class = class;
        s.pose = instance.pose.unwrap_or_default();
        s.size = instance.size.unwrap_or([0.005, 0.005, 0.005]);
        s.rgba = instance.rgba;
        s.material = instance.material.clone();
        s.type_ = instance.type_.unwrap_or(SiteType::Sphere);
        for attr in node.attributes() {
            if attr.name() == "user" {
                s.user = parse_f64_list(attr.value())?;
            }
        }
        Ok(s)
    }
}
