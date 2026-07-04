//! `<asset>` block parsing: mesh / hfield / texture / material definitions,
//! plus `<asset><model>` sub-model registration. The sub-model registry
//! feeds `<attach>` (see `attach.rs`).

use std::path::{Path, PathBuf};

use roxmltree::Node;

use crate::Pose;
use crate::assets::{Hfield, Material, Mesh, MeshInertia, Texture};
use crate::error::ParseError;
use crate::model::Model;
use glamx::glam::{DQuat, DVec3};

use super::parse_utils::{
    absolutize_model_assets, asset_default_name, parse_f64, parse_f64_list, parse_quat, parse_u32,
    parse_vec3, parse_vec4,
};
use super::prototypes::merge_material_proto;
use super::state::ParseState;

impl ParseState {
    pub(super) fn parse_asset(&mut self, root: Node) -> Result<(), ParseError> {
        for child in root.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "mesh" => {
                    let class = child.attribute("class").map(|s| s.to_string());
                    let proto = self.merged_mesh_proto(class.as_deref());
                    let mut m = Mesh {
                        class,
                        scale: proto.scale.unwrap_or([1.0, 1.0, 1.0]),
                        inertia: proto.inertia.unwrap_or_default(),
                        refpose: proto.refpose.unwrap_or_default(),
                        max_hull_vert: proto.max_hull_vert,
                        smoothnormal: proto.smoothnormal.unwrap_or(0.0),
                        ..Default::default()
                    };
                    let mut pose_pos = None;
                    let mut pose_rot = None;
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => m.name = Some(attr.value().to_string()),
                            "file" => m.file = Some(attr.value().to_string()),
                            "scale" => m.scale = parse_vec3(attr.value(), "mesh", "scale")?,
                            "refpos" => {
                                pose_pos = Some(parse_vec3(attr.value(), "mesh", "refpos")?)
                            }
                            "refquat" => {
                                pose_rot = Some(parse_quat(attr.value(), "mesh", "refquat")?)
                            }
                            "vertex" => m.inline_vertices = Some(parse_f64_list(attr.value())?),
                            "normal" => m.inline_normals = Some(parse_f64_list(attr.value())?),
                            "face" => {
                                let v = parse_f64_list(attr.value())?;
                                m.inline_faces = Some(v.iter().map(|x| *x as u32).collect());
                            }
                            "maxhullvert" => m.max_hull_vert = Some(parse_u32(attr.value())?),
                            "smoothnormal" => m.smoothnormal = parse_f64(attr.value())?,
                            "inertia" => {
                                m.inertia = match attr.value() {
                                    "shell" => MeshInertia::Shell,
                                    "convex" => MeshInertia::Convex,
                                    "exact" => MeshInertia::Exact,
                                    "legacy" => MeshInertia::Legacy,
                                    v => {
                                        return Err(ParseError::BadAttributeValue {
                                            tag: "mesh".to_string(),
                                            attr: "inertia".to_string(),
                                            value: v.to_string(),
                                            message: "expected shell/convex/exact/legacy"
                                                .to_string(),
                                        });
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                    if let (Some(p), q) = (pose_pos, pose_rot) {
                        m.refpose =
                            Pose::from_parts(DVec3::from_array(p), q.unwrap_or(DQuat::IDENTITY));
                    } else if let Some(q) = pose_rot {
                        m.refpose.rotation = q;
                    }
                    if m.name.is_none() {
                        m.name = asset_default_name(m.file.as_deref());
                    }
                    self.model.assets.meshes.push(m);
                }
                "hfield" => {
                    let mut h = Hfield::default();
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => h.name = Some(attr.value().to_string()),
                            "class" => h.class = Some(attr.value().to_string()),
                            "nrow" => h.nrow = parse_u32(attr.value())?,
                            "ncol" => h.ncol = parse_u32(attr.value())?,
                            "size" => h.size = parse_vec4(attr.value(), "hfield", "size")?,
                            "file" => h.file = Some(attr.value().to_string()),
                            "elevation" => h.elevation = Some(parse_f64_list(attr.value())?),
                            _ => {}
                        }
                    }
                    if h.name.is_none() {
                        h.name = asset_default_name(h.file.as_deref());
                    }
                    self.model.assets.hfields.push(h);
                }
                "texture" => {
                    let mut t = Texture::default();
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => t.name = Some(attr.value().to_string()),
                            "class" => t.class = Some(attr.value().to_string()),
                            "type" => t.type_ = Some(attr.value().to_string()),
                            "file" => t.file = Some(attr.value().to_string()),
                            "builtin" => t.builtin = Some(attr.value().to_string()),
                            "rgb1" => t.rgb1 = Some(parse_vec3(attr.value(), "texture", "rgb1")?),
                            "rgb2" => t.rgb2 = Some(parse_vec3(attr.value(), "texture", "rgb2")?),
                            _ => {}
                        }
                    }
                    if t.name.is_none() {
                        t.name = asset_default_name(t.file.as_deref());
                    }
                    self.model.assets.textures.push(t);
                }
                "material" => {
                    // Resolve the material against its `<default class=…>`
                    // chain: the material's own attributes win, then the class
                    // defaults fill the rest, then MuJoCo's built-in defaults
                    // (specular/shininess 0.5; metallic/roughness -1, a sentinel
                    // meaning "unset — fall back to the legacy Phong shininess").
                    let inst = self.parse_material_prototype(child)?;
                    let mut name = None;
                    let mut class = None;
                    for attr in child.attributes() {
                        match attr.name() {
                            "name" => name = Some(attr.value().to_string()),
                            "class" => class = Some(attr.value().to_string()),
                            _ => {}
                        }
                    }
                    let p = merge_material_proto(
                        Some(self.merged_material_proto(class.as_deref())),
                        inst,
                    );
                    let m = Material {
                        name,
                        class,
                        texture: p.texture,
                        rgba: p.rgba,
                        emission: p.emission.unwrap_or(0.0),
                        specular: p.specular.unwrap_or(0.5),
                        shininess: p.shininess.unwrap_or(0.5),
                        roughness: p.roughness.unwrap_or(-1.0),
                        metallic: p.metallic.unwrap_or(-1.0),
                    };
                    self.model.assets.materials.push(m);
                }
                "include" => {
                    self.parse_include(child, true)?;
                }
                "model" => {
                    self.parse_asset_model(child)?;
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Parses a `<asset><model name="X" file="Y.xml"/>` declaration:
    /// loads `Y.xml` as a standalone [`Model`], rewrites every asset
    /// file path inside it to an absolute path (so the assets continue
    /// to resolve after being merged into a parent model with a
    /// different `meshdir` / `texturedir`), and stores the result under
    /// `X` for later `<attach>` references.
    pub(super) fn parse_asset_model(&mut self, node: Node) -> Result<(), ParseError> {
        let Some(name) = node.attribute("name") else {
            log::warn!("<asset><model>: missing `name` attribute; skipping");
            return Ok(());
        };
        let Some(file) = node.attribute("file") else {
            log::warn!("<asset><model name={name:?}>: missing `file` attribute; skipping");
            return Ok(());
        };
        let full_path = self.base_dir.join(file);
        let mut sub = match Model::from_file(&full_path) {
            Ok(m) => m,
            Err(e) => {
                log::warn!(
                    "<asset><model name={name:?} file={file:?}>: failed to load \
                     sub-model: {e}"
                );
                return Ok(());
            }
        };
        let sub_base_dir = full_path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        absolutize_model_assets(&mut sub, &sub_base_dir);
        self.sub_models.insert(name.to_string(), sub);
        Ok(())
    }
}
