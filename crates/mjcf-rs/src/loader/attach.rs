//! `<attach>` body splicing: copy a previously-registered sub-model's
//! body subtree under the current parent, prefix every name and every
//! external reference, then carry the sub-model's equality / contact /
//! actuator / sensor entries across with the same prefix.

use roxmltree::Node;

use crate::error::ParseError;
use crate::model::{BodyEntry, BodyId, Model};

use super::parse_utils::{prefix_geom_refs, prefix_site_refs, prefixed};
use super::state::ParseState;

impl ParseState {
    /// Splices a sub-model's body subtree (declared via
    /// `<asset><model>`) under the body identified by `parent_id`. All
    /// names within the spliced content are prefixed by the `prefix=`
    /// attribute so they don't collide with the parent's namespace.
    /// Sub-model assets (meshes, materials, textures, hfields),
    /// equality constraints, contact entries, actuators and sensors are
    /// also brought across with prefixed references.
    pub(super) fn parse_attach(&mut self, node: Node, parent_id: BodyId) -> Result<(), ParseError> {
        let model_name = node.attribute("model").unwrap_or("");
        let body_name = node.attribute("body").unwrap_or("");
        let prefix = node.attribute("prefix").unwrap_or("").to_string();

        // Clone the sub-model so we can borrow `self` mutably while
        // splicing — sub-models are read-only after parse.
        let Some(sub) = self.sub_models.get(model_name).cloned() else {
            log::warn!(
                "<attach model={model_name:?} body={body_name:?}>: unknown sub-model \
                 (no matching <asset><model name=…>); skipping"
            );
            return Ok(());
        };
        // Body 0 in any sub-model is the implicit world. We want a real
        // declared body, so skip index 0 when matching by name.
        let sub_body_id = sub.bodies.iter().enumerate().skip(1).find_map(|(i, b)| {
            if b.body.name.as_deref() == Some(body_name) {
                Some(i)
            } else {
                None
            }
        });
        let Some(sub_body_id) = sub_body_id else {
            log::warn!(
                "<attach model={model_name:?} body={body_name:?}>: body not found in \
                 sub-model; skipping"
            );
            return Ok(());
        };

        // Assets first so geom/material refs inside the spliced subtree
        // already resolve.
        self.merge_assets(&sub, &prefix);
        // Body subtree (and embedded joints / geoms / sites).
        self.splice_body_subtree(&sub, sub_body_id, parent_id, &prefix);
        // Bring over cross-cutting MJCF sections — bodies + assets
        // exist by name at this point, so name remapping just needs to
        // prepend the prefix.
        self.merge_equality(&sub, &prefix);
        self.merge_contact(&sub, &prefix);
        self.merge_actuators(&sub, &prefix);
        self.merge_sensors(&sub, &prefix);
        Ok(())
    }

    /// Deep-copies the body tree rooted at `sub_id` (within the
    /// sub-model `sub`) under `new_parent_id` in `self.model.bodies`,
    /// prefixing every name and asset reference inside the subtree.
    fn splice_body_subtree(
        &mut self,
        sub: &Model,
        sub_id: BodyId,
        new_parent_id: BodyId,
        prefix: &str,
    ) {
        let mut body = sub.bodies[sub_id].body.clone();
        body.name = body.name.map(|n| prefixed(prefix, &n));
        for j in &mut body.joints {
            if let Some(n) = j.name.take() {
                j.name = Some(prefixed(prefix, &n));
            }
        }
        for g in &mut body.geoms {
            prefix_geom_refs(g, prefix);
        }
        for s in &mut body.sites {
            prefix_site_refs(s, prefix);
        }
        let new_id = self.model.bodies.len();
        self.model.bodies.push(BodyEntry {
            parent: Some(new_parent_id),
            body,
        });
        // Recurse into the sub-model's child bodies that point at
        // `sub_id`. We walk all entries; the IDs are stable across the
        // clone since we never mutate `sub`.
        let children: Vec<BodyId> = sub
            .bodies
            .iter()
            .enumerate()
            .filter(|(_, b)| b.parent == Some(sub_id))
            .map(|(i, _)| i)
            .collect();
        for child_id in children {
            self.splice_body_subtree(sub, child_id, new_id, prefix);
        }
    }

    fn merge_assets(&mut self, sub: &Model, prefix: &str) {
        for mesh in &sub.assets.meshes {
            let mut m = mesh.clone();
            if let Some(n) = m.name.take() {
                m.name = Some(prefixed(prefix, &n));
            }
            self.model.assets.meshes.push(m);
        }
        for hf in &sub.assets.hfields {
            let mut h = hf.clone();
            if let Some(n) = h.name.take() {
                h.name = Some(prefixed(prefix, &n));
            }
            self.model.assets.hfields.push(h);
        }
        for tex in &sub.assets.textures {
            let mut t = tex.clone();
            if let Some(n) = t.name.take() {
                t.name = Some(prefixed(prefix, &n));
            }
            self.model.assets.textures.push(t);
        }
        for mat in &sub.assets.materials {
            let mut m = mat.clone();
            if let Some(n) = m.name.take() {
                m.name = Some(prefixed(prefix, &n));
            }
            if let Some(n) = m.texture.take() {
                m.texture = Some(prefixed(prefix, &n));
            }
            self.model.assets.materials.push(m);
        }
    }

    fn merge_equality(&mut self, sub: &Model, prefix: &str) {
        for eq in &sub.equality {
            match eq {
                crate::equality::Equality::Connect(c) => {
                    let mut new_c = c.clone();
                    if let Some(n) = new_c.common.name.take() {
                        new_c.common.name = Some(prefixed(prefix, &n));
                    }
                    new_c.body1 = prefixed(prefix, &new_c.body1);
                    if let Some(b2) = new_c.body2.take() {
                        new_c.body2 = Some(prefixed(prefix, &b2));
                    }
                    self.model
                        .equality
                        .push(crate::equality::Equality::Connect(new_c));
                }
                crate::equality::Equality::Weld(w) => {
                    let mut new_w = w.clone();
                    if let Some(n) = new_w.common.name.take() {
                        new_w.common.name = Some(prefixed(prefix, &n));
                    }
                    new_w.body1 = prefixed(prefix, &new_w.body1);
                    if let Some(b2) = new_w.body2.take() {
                        new_w.body2 = Some(prefixed(prefix, &b2));
                    }
                    self.model
                        .equality
                        .push(crate::equality::Equality::Weld(new_w));
                }
                crate::equality::Equality::Joint(j) => {
                    let mut new_j = j.clone();
                    if let Some(n) = new_j.common.name.take() {
                        new_j.common.name = Some(prefixed(prefix, &n));
                    }
                    new_j.joint1 = prefixed(prefix, &new_j.joint1);
                    if let Some(j2) = new_j.joint2.take() {
                        new_j.joint2 = Some(prefixed(prefix, &j2));
                    }
                    self.model
                        .equality
                        .push(crate::equality::Equality::Joint(new_j));
                }
            }
        }
    }

    fn merge_contact(&mut self, sub: &Model, prefix: &str) {
        for p in &sub.contact.pairs {
            let mut new_p = p.clone();
            if let Some(n) = new_p.name.take() {
                new_p.name = Some(prefixed(prefix, &n));
            }
            new_p.geom1 = prefixed(prefix, &new_p.geom1);
            new_p.geom2 = prefixed(prefix, &new_p.geom2);
            self.model.contact.pairs.push(new_p);
        }
        for ex in &sub.contact.excludes {
            let mut new_e = ex.clone();
            if let Some(n) = new_e.name.take() {
                new_e.name = Some(prefixed(prefix, &n));
            }
            new_e.body1 = prefixed(prefix, &new_e.body1);
            new_e.body2 = prefixed(prefix, &new_e.body2);
            self.model.contact.excludes.push(new_e);
        }
    }

    fn merge_actuators(&mut self, sub: &Model, prefix: &str) {
        for a in &sub.actuators {
            let mut new_a = a.clone();
            if let Some(n) = new_a.name.take() {
                new_a.name = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_a.joint.take() {
                new_a.joint = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_a.tendon.take() {
                new_a.tendon = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_a.body.take() {
                new_a.body = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_a.site.take() {
                new_a.site = Some(prefixed(prefix, &n));
            }
            self.model.actuators.push(new_a);
        }
    }

    fn merge_sensors(&mut self, sub: &Model, prefix: &str) {
        for s in &sub.sensors {
            let mut new_s = s.clone();
            if let Some(n) = new_s.name.take() {
                new_s.name = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_s.objname.take() {
                new_s.objname = Some(prefixed(prefix, &n));
            }
            if let Some(n) = new_s.refname.take() {
                new_s.refname = Some(prefixed(prefix, &n));
            }
            self.model.sensors.push(new_s);
        }
    }
}
