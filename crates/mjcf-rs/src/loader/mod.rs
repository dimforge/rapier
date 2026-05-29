//! XML loader. Reads MJCF files (potentially across `<include>`s) and
//! produces a flat, resolved [`Model`].
//!
//! The implementation is split across submodules — each handles one
//! conceptual slice of the MJCF schema — but the only public surface is
//! `Model::from_file` / `Model::from_str` and the asset-path resolver
//! methods at the bottom of this file.

use std::path::{Path, PathBuf};

use crate::assets::{Mesh, Texture};
use crate::error::ParseError;
use crate::model::Model;

mod asset;
mod attach;
mod blocks;
mod bodies;
mod compiler_opts;
mod defaults;
mod parse_utils;
mod prototypes;
mod state;

use state::ParseState;

impl Model {
    /// Parses an MJCF file from the given path. `<include>` files are
    /// resolved relative to the file's directory.
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, ParseError> {
        let path = path.as_ref();
        let xml = std::fs::read_to_string(path).map_err(|e| ParseError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        let base_dir = path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        Self::from_str(&xml, &base_dir)
    }

    /// Parses an MJCF string. `<include>` directives resolve relative to
    /// `base_dir`.
    pub fn from_str(xml: &str, base_dir: impl AsRef<Path>) -> Result<Self, ParseError> {
        let mut state = ParseState::new(base_dir.as_ref().to_path_buf());
        state.parse_document(xml)?;
        Ok(state.into_model())
    }

    /// Look up a texture asset's on-disk path, joining its `file=`
    /// against the `texturedir` / `assetdir` hierarchy. Returns `None`
    /// for textures without a `file=` (e.g. `builtin="checker"`).
    pub fn resolve_texture_file(&self, texture: &Texture, base_dir: &Path) -> Option<PathBuf> {
        let file = texture.file.as_ref()?;
        let path = if self.compiler.strip_path {
            Path::new(file)
                .file_name()
                .map(|f| Path::new(f).to_path_buf())
                .unwrap_or_else(|| Path::new(file).to_path_buf())
        } else {
            Path::new(file).to_path_buf()
        };
        if path.is_absolute() {
            return Some(path);
        }
        let dir = self
            .compiler
            .texture_dir
            .clone()
            .or_else(|| self.compiler.asset_dir.clone());
        if let Some(dir) = dir {
            let dirp = Path::new(&dir);
            return Some(if dirp.is_absolute() {
                dirp.join(&path)
            } else {
                base_dir.join(dirp).join(&path)
            });
        }
        Some(base_dir.join(&path))
    }

    /// Look up a mesh asset by name, joining its `file=` against the mesh
    /// directory hierarchy.
    pub fn resolve_mesh_file(&self, mesh: &Mesh, base_dir: &Path) -> Option<PathBuf> {
        let file = mesh.file.as_ref()?;
        let path = if self.compiler.strip_path {
            Path::new(file)
                .file_name()
                .map(|f| Path::new(f).to_path_buf())
                .unwrap_or_else(|| Path::new(file).to_path_buf())
        } else {
            Path::new(file).to_path_buf()
        };
        if path.is_absolute() {
            return Some(path);
        }
        let dir = self
            .compiler
            .mesh_dir
            .clone()
            .or_else(|| self.compiler.asset_dir.clone());
        if let Some(dir) = dir {
            let dirp = Path::new(&dir);
            return Some(if dirp.is_absolute() {
                dirp.join(&path)
            } else {
                base_dir.join(dirp).join(&path)
            });
        }
        Some(base_dir.join(&path))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::JointType;

    const SIMPLE: &str = r#"
<mujoco model="t">
  <compiler angle="degree" eulerseq="xyz"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <joint name="j1" type="hinge" axis="0 1 0" range="-90 90"/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
"#;

    #[test]
    fn parse_simple() {
        let m = Model::from_str(SIMPLE, ".").unwrap();
        assert_eq!(m.name.as_deref(), Some("t"));
        assert_eq!(m.bodies.len(), 2);
        let b = &m.bodies[1].body;
        assert_eq!(b.name.as_deref(), Some("b1"));
        assert_eq!(b.joints.len(), 1);
        assert_eq!(b.geoms.len(), 1);
        let r = b.joints[0].range.unwrap();
        // -90/+90 deg → -π/2 / π/2 rad
        assert!((r[0] + std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((r[1] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn parse_radians() {
        let xml = r#"<mujoco>
            <compiler angle="radian"/>
            <worldbody>
                <body><joint type="hinge" axis="0 0 1" range="-1.0 1.0"/></body>
            </worldbody></mujoco>"#;
        let m = Model::from_str(xml, ".").unwrap();
        let r = m.bodies[1].body.joints[0].range.unwrap();
        assert!((r[0] - -1.0).abs() < 1e-12);
        assert!((r[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn parse_default_inheritance() {
        let xml = r#"<mujoco>
            <default>
                <joint damping="0.5" type="hinge"/>
                <default class="leg">
                    <joint damping="0.3" range="-30 30"/>
                </default>
            </default>
            <worldbody>
                <body>
                    <joint name="j1"/>
                    <joint class="leg" name="j2"/>
                </body>
            </worldbody></mujoco>"#;
        let m = Model::from_str(xml, ".").unwrap();
        let body = &m.bodies[1].body;
        assert_eq!(body.joints[0].damping, 0.5);
        assert_eq!(body.joints[0].range, None);
        // class-leg overrides damping but inherits type from main.
        assert_eq!(body.joints[1].damping, 0.3);
        let r = body.joints[1].range.unwrap();
        // -30 / 30 deg.
        assert!((r[0] + 30.0_f64.to_radians()).abs() < 1e-12);
    }

    #[test]
    fn parse_freejoint() {
        let xml = r#"<mujoco><worldbody>
            <body name="free"><freejoint/></body>
        </worldbody></mujoco>"#;
        let m = Model::from_str(xml, ".").unwrap();
        let b = &m.bodies[1].body;
        assert_eq!(b.joints.len(), 1);
        assert_eq!(b.joints[0].type_, JointType::Free);
    }

    /// MuJoCo: when `<mesh>` / `<hfield>` / `<texture>` has no `name=` but
    /// has a `file=`, the asset is registered under the file's basename
    /// without extension (so `<geom mesh="hip">` resolves a `<mesh
    /// file="hip.obj"/>`). Regression for the unitree_a1 scene.
    #[test]
    fn asset_name_defaults_to_file_stem() {
        let xml = r#"<mujoco>
          <asset>
            <mesh file="assets/hip.obj"/>
            <mesh name="explicit" file="thigh.obj"/>
            <mesh file="calf"/>
            <hfield file="terrain.png" nrow="2" ncol="2" size="1 1 1 0"/>
            <texture type="2d" file="ground.png"/>
          </asset>
        </mujoco>"#;
        let m = Model::from_str(xml, ".").unwrap();
        assert!(m.assets.mesh("hip").is_some(), "hip.obj → \"hip\"");
        assert!(
            m.assets.mesh("explicit").is_some(),
            "explicit name preserved"
        );
        assert!(m.assets.mesh("calf").is_some(), "extension-less file");
        assert!(m.assets.hfield("terrain").is_some(), "hfield default name");
        assert!(
            m.assets
                .textures
                .iter()
                .any(|t| t.name.as_deref() == Some("ground")),
            "texture default name"
        );
    }

    /// `<default><geom mesh="…"/></default>` should propagate the mesh
    /// reference (and `hfield="…"`) onto geoms that use the class.
    /// Regression for wonik_allegro, whose visual classes look like
    /// `<default class="base_visual"><geom mesh="link_0.0"/></default>`
    /// and whose individual finger geoms are bare `<geom class="base_visual"/>`.
    #[test]
    fn default_geom_propagates_mesh_and_hfield() {
        let xml = r#"<mujoco>
            <default>
                <default class="link_visual">
                    <geom type="mesh" contype="0" conaffinity="0" mesh="link_proto"/>
                </default>
            </default>
            <asset>
                <mesh name="link_proto" file="link.obj"/>
                <mesh name="link_override" file="other.obj"/>
            </asset>
            <worldbody>
                <body>
                    <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                    <geom class="link_visual"/>
                    <geom class="link_visual" mesh="link_override"/>
                </body>
            </worldbody>
        </mujoco>"#;
        let m = Model::from_str(xml, ".").unwrap();
        let geoms = &m.bodies[1].body.geoms;
        assert_eq!(geoms.len(), 2);
        // No `mesh=` on the instance → inherit from the class default.
        assert_eq!(geoms[0].mesh.as_deref(), Some("link_proto"));
        // Instance `mesh=` overrides the class default.
        assert_eq!(geoms[1].mesh.as_deref(), Some("link_override"));
    }

    /// `<asset><model>` + `<attach>` splices a sub-model's body subtree
    /// under the attach point, prefixing every name. Regression for
    /// iit_softfoot, which is composed this way.
    #[test]
    fn attach_splices_sub_model_with_prefix() {
        use std::fs;
        let dir = tempfile::tempdir().unwrap();
        // The sub-model: one body with one joint, one mesh asset, one
        // equality referencing the body.
        let sub = dir.path().join("sub.xml");
        fs::write(
            &sub,
            r#"<mujoco>
              <asset>
                <mesh name="brick" file="brick.stl"/>
              </asset>
              <worldbody>
                <body name="attach_point" pos="0 0 1">
                  <joint name="hinge" type="hinge" axis="0 0 1"/>
                  <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                  <geom type="box" size="0.1 0.1 0.1" mesh="brick"/>
                  <body name="tip" pos="0.5 0 0">
                    <joint name="slider" type="slide" axis="1 0 0"/>
                    <inertial mass="0.5" diaginertia="0.05 0.05 0.05"/>
                  </body>
                </body>
              </worldbody>
              <equality>
                <weld body1="attach_point"/>
              </equality>
            </mujoco>"#,
        )
        .unwrap();

        let main = dir.path().join("main.xml");
        fs::write(
            &main,
            r#"<mujoco>
              <asset>
                <model name="sub_model" file="sub.xml"/>
              </asset>
              <worldbody>
                <body name="rig" pos="1 0 0">
                  <freejoint/>
                  <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                  <attach model="sub_model" body="attach_point" prefix="A_"/>
                </body>
              </worldbody>
            </mujoco>"#,
        )
        .unwrap();

        let m = Model::from_file(&main).unwrap();
        // Bodies: world + rig + A_attach_point + A_tip == 4.
        let names: Vec<_> = m
            .bodies
            .iter()
            .filter_map(|b| b.body.name.as_deref())
            .collect();
        assert!(names.contains(&"rig"), "main body preserved");
        assert!(
            names.contains(&"A_attach_point"),
            "attach root prefixed: {names:?}"
        );
        assert!(
            names.contains(&"A_tip"),
            "attach descendant prefixed: {names:?}"
        );
        // Joints from the sub-model get prefixed too.
        let joint_names: Vec<_> = m
            .bodies
            .iter()
            .flat_map(|b| b.body.joints.iter().filter_map(|j| j.name.as_deref()))
            .collect();
        assert!(
            joint_names.contains(&"A_hinge"),
            "joint prefixed: {joint_names:?}"
        );
        assert!(
            joint_names.contains(&"A_slider"),
            "child joint prefixed: {joint_names:?}"
        );
        // Assets get prefixed.
        assert!(
            m.assets.mesh("A_brick").is_some(),
            "mesh prefixed; got {:?}",
            m.assets
                .meshes
                .iter()
                .map(|x| x.name.clone())
                .collect::<Vec<_>>()
        );
        // Geom mesh ref points at the prefixed asset.
        let attach_body = m
            .bodies
            .iter()
            .find(|b| b.body.name.as_deref() == Some("A_attach_point"))
            .unwrap();
        assert_eq!(attach_body.body.geoms[0].mesh.as_deref(), Some("A_brick"));
        // Equality body refs got prefixed.
        assert_eq!(m.equality.len(), 1);
        match &m.equality[0] {
            crate::equality::Equality::Weld(w) => {
                assert_eq!(w.body1, "A_attach_point");
            }
            other => panic!("expected weld, got {other:?}"),
        }
    }
}
