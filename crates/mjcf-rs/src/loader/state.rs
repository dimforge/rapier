//! Mutable loader state shared across the parse passes, plus
//! document-entry and include-resolution methods.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use roxmltree::{Document, Node};

use crate::body::Body;
use crate::error::ParseError;
use crate::model::{BodyEntry, Model};

use super::prototypes::DefaultsClass;

pub(super) struct ParseState {
    pub(super) base_dir: PathBuf,
    /// Stack of files currently being included, for cycle detection.
    pub(super) include_stack: Vec<PathBuf>,
    pub(super) model: Model,
    pub(super) defaults: HashMap<String, DefaultsClass>,
    /// Class-name lookup for `<default>` parent chain (newest first).
    pub(super) defaults_parent_of: HashMap<String, Option<String>>,
    /// Sub-models declared via `<asset><model name="X" file="Y.xml"/>`,
    /// keyed by `X`. The sub-model has already been parsed; its asset
    /// `file=` attributes have been rewritten to absolute paths so they
    /// resolve correctly after being merged into a parent that may have
    /// a different `meshdir` / `texturedir`.
    pub(super) sub_models: HashMap<String, Model>,
}

impl ParseState {
    pub(super) fn new(base_dir: PathBuf) -> Self {
        let mut model = Model {
            // Reserve index 0 for the world body.
            bodies: vec![BodyEntry {
                parent: None,
                body: Body::default(),
            }],
            ..Model::default()
        };
        // Default empty "main" class.
        let mut defaults = HashMap::new();
        defaults.insert("main".to_string(), DefaultsClass::default());
        let mut defaults_parent_of = HashMap::new();
        defaults_parent_of.insert("main".to_string(), None);
        // Make sure the world body has empty defaults.
        let _ = &mut model;
        Self {
            base_dir,
            include_stack: Vec::new(),
            model,
            defaults,
            defaults_parent_of,
            sub_models: HashMap::new(),
        }
    }

    pub(super) fn into_model(self) -> Model {
        self.model
    }

    pub(super) fn parse_document(&mut self, xml: &str) -> Result<(), ParseError> {
        let doc = Document::parse(xml).map_err(ParseError::Xml)?;
        let root = doc.root_element();
        if root.tag_name().name() != "mujoco" {
            return Err(ParseError::InvalidModel(format!(
                "expected <mujoco> root, found <{}>",
                root.tag_name().name()
            )));
        }
        if let Some(name) = root.attribute("model") {
            if self.model.name.is_none() {
                self.model.name = Some(name.to_string());
            }
        }
        self.process_root_children(root)?;
        Ok(())
    }

    pub(super) fn process_root_children(&mut self, root: Node) -> Result<(), ParseError> {
        // Process top-level elements in two passes:
        // 1. compiler / option / size / default / asset / contact / equality / tendon
        //    — anything that doesn't depend on the body tree
        // 2. worldbody / actuator / sensor / keyframe
        // <include> can appear anywhere; we resolve recursively in place.
        let mut deferred_bodies: Vec<Node> = Vec::new();
        let mut deferred_actuators: Vec<Node> = Vec::new();
        let mut deferred_sensors: Vec<Node> = Vec::new();
        let mut deferred_keyframes: Vec<Node> = Vec::new();
        for child in root.children() {
            if !child.is_element() {
                continue;
            }
            let name = child.tag_name().name();
            match name {
                "compiler" => self.parse_compiler(child)?,
                "option" => self.parse_option(child)?,
                "size" | "statistic" | "visual" | "custom" | "extension" => {
                    // Pass-through / ignored for now.
                }
                "default" => self.parse_default(child, None)?,
                "asset" => self.parse_asset(child)?,
                "include" => self.parse_include(child, /*nested=*/ false)?,
                "worldbody" => deferred_bodies.push(child),
                "contact" => self.parse_contact(child)?,
                "equality" => self.parse_equality(child)?,
                "tendon" => {
                    // Out of scope; record nothing.
                }
                "actuator" => deferred_actuators.push(child),
                "sensor" => deferred_sensors.push(child),
                "keyframe" => deferred_keyframes.push(child),
                _ => {
                    log::warn!("unhandled top-level element <{name}>");
                }
            }
        }
        // Pass 2 — needs the full <default>/<asset> table.
        for n in deferred_bodies {
            self.parse_worldbody(n)?;
        }
        for n in deferred_actuators {
            self.parse_actuator_block(n)?;
        }
        for n in deferred_sensors {
            self.parse_sensor_block(n)?;
        }
        for n in deferred_keyframes {
            self.parse_keyframe_block(n)?;
        }
        Ok(())
    }

    pub(super) fn parse_include(&mut self, node: Node, _nested: bool) -> Result<(), ParseError> {
        let file = node
            .attribute("file")
            .ok_or_else(|| ParseError::BadAttribute {
                tag: "include".to_string(),
                attr: "file".to_string(),
                message: "missing".to_string(),
            })?;
        let resolved = self.resolve_path(file);
        let canonical = resolved.canonicalize().unwrap_or_else(|_| resolved.clone());
        if self.include_stack.iter().any(|p| p == &canonical) {
            return Err(ParseError::BadInclude {
                file: resolved,
                message: format!(
                    "include cycle (currently importing: {:?})",
                    self.include_stack
                ),
            });
        }
        let xml = std::fs::read_to_string(&resolved).map_err(|e| ParseError::Io {
            path: resolved.clone(),
            source: e,
        })?;
        let prev_base = self.base_dir.clone();
        if let Some(p) = resolved.parent() {
            self.base_dir = p.to_path_buf();
        }
        self.include_stack.push(canonical);
        let doc = Document::parse(&xml).map_err(ParseError::Xml)?;
        let inc_root = doc.root_element();
        if inc_root.tag_name().name() != "mujoco" {
            return Err(ParseError::BadInclude {
                file: resolved,
                message: "included file must have <mujoco> root".to_string(),
            });
        }
        // Adopt model name if absent.
        if let Some(name) = inc_root.attribute("model") {
            if self.model.name.is_none() {
                self.model.name = Some(name.to_string());
            }
        }
        // Process the included file's top-level children just like we do
        // for the main file. Defaults / assets / etc. all merge.
        let res = self.process_root_children(inc_root);
        self.include_stack.pop();
        self.base_dir = prev_base;
        res
    }

    pub(super) fn resolve_path(&self, file: &str) -> PathBuf {
        let p = Path::new(file);
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            self.base_dir.join(p)
        }
    }

    #[allow(dead_code)]
    pub(super) fn resolve_mesh_path(&self, file: &str) -> PathBuf {
        if self.model.compiler.strip_path {
            // Strip directory components.
            let stripped = Path::new(file).file_name().unwrap_or(file.as_ref());
            return self.resolve_path(&Path::new(stripped).to_string_lossy());
        }
        let dir = self
            .model
            .compiler
            .mesh_dir
            .clone()
            .or_else(|| self.model.compiler.asset_dir.clone());
        if let Some(dir) = dir {
            let candidate = if Path::new(&dir).is_absolute() {
                Path::new(&dir).join(file)
            } else {
                self.base_dir.join(&dir).join(file)
            };
            return candidate;
        }
        self.resolve_path(file)
    }
}
