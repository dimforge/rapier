//! Top-level `Model` AST.

use crate::assets::Assets;
use crate::body::Body;
use crate::compiler::{Compiler, SimOption};
use crate::contact::Contact;
use crate::equality::Equality;
use crate::extras::{Actuator, Keyframe, Sensor};

/// Index into [`Model::bodies`]. Index 0 is reserved for the implicit world body.
pub type BodyId = usize;

/// A parsed MJCF document.
///
/// This is a flattened, fully-resolved view of the model:
/// - `<include>` files have been inlined
/// - `<default>` class inheritance has been baked into each element
/// - all angle attributes are in radians
/// - the body tree is stored as a flat `Vec<BodyEntry>` where each entry
///   knows its parent (the implicit world body is at index 0)
#[derive(Clone, Debug, Default)]
pub struct Model {
    /// `<mujoco model="…">`.
    pub name: Option<String>,
    /// Resolved compiler settings.
    pub compiler: Compiler,
    /// `<option>` settings.
    pub option: SimOption,
    /// Asset library.
    pub assets: Assets,
    /// Bodies, in insertion order. Index 0 is the implicit world body.
    pub bodies: Vec<BodyEntry>,
    /// `<contact>` data.
    pub contact: Contact,
    /// `<equality>` constraints.
    pub equality: Vec<Equality>,
    /// `<actuator>` elements (preserved verbatim).
    pub actuators: Vec<Actuator>,
    /// `<sensor>` elements.
    pub sensors: Vec<Sensor>,
    /// `<keyframe><key>` entries.
    pub keyframes: Vec<Keyframe>,
    /// `<tendon><fixed>` elements (spatial tendons are not represented).
    pub tendons: Vec<crate::tendon::FixedTendon>,
}

/// One entry in the flat body list.
#[derive(Clone, Debug)]
pub struct BodyEntry {
    /// Parent body id (`None` only for the world body at index 0).
    pub parent: Option<BodyId>,
    /// Body data (or the empty default for the world).
    pub body: Body,
}

impl Model {
    /// Iterates over every non-world body in the model.
    pub fn bodies_iter(&self) -> impl Iterator<Item = (BodyId, &BodyEntry)> {
        self.bodies.iter().enumerate().filter(|(id, _)| *id != 0)
    }

    /// Looks up a body by name. World body has no name and is unreachable
    /// through this method — pass `BodyId::default()` for the world.
    pub fn body_id_by_name(&self, name: &str) -> Option<BodyId> {
        self.bodies
            .iter()
            .position(|e| e.body.name.as_deref() == Some(name))
    }

    /// Looks up a geom by name across all bodies. Returns `(body_id, geom_index)`.
    pub fn geom_by_name(&self, name: &str) -> Option<(BodyId, usize)> {
        for (bi, e) in self.bodies.iter().enumerate() {
            for (gi, g) in e.body.geoms.iter().enumerate() {
                if g.name.as_deref() == Some(name) {
                    return Some((bi, gi));
                }
            }
        }
        None
    }

    /// Looks up a joint by name across all bodies. Returns `(body_id, joint_index)`.
    pub fn joint_by_name(&self, name: &str) -> Option<(BodyId, usize)> {
        for (bi, e) in self.bodies.iter().enumerate() {
            for (ji, j) in e.body.joints.iter().enumerate() {
                if j.name.as_deref() == Some(name) {
                    return Some((bi, ji));
                }
            }
        }
        None
    }

    /// Looks up a site by name. Returns `(body_id, site_index)`.
    pub fn site_by_name(&self, name: &str) -> Option<(BodyId, usize)> {
        for (bi, e) in self.bodies.iter().enumerate() {
            for (si, s) in e.body.sites.iter().enumerate() {
                if s.name.as_deref() == Some(name) {
                    return Some((bi, si));
                }
            }
        }
        None
    }
}
