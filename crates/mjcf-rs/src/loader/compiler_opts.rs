//! Parsers for the top-level `<compiler>` and `<option>` blocks.

use roxmltree::Node;

use crate::compiler::InertiaFromGeom;
use crate::error::ParseError;
use crate::types::Coordinate;

use super::parse_utils::{parse_bool, parse_f64, parse_vec3};
use super::state::ParseState;

impl ParseState {
    pub(super) fn parse_compiler(&mut self, node: Node) -> Result<(), ParseError> {
        let c = &mut self.model.compiler;
        for attr in node.attributes() {
            match attr.name() {
                "angle" => match attr.value() {
                    "degree" => c.angle_is_degree = true,
                    "radian" => c.angle_is_degree = false,
                    other => {
                        return Err(ParseError::BadAttributeValue {
                            tag: "compiler".to_string(),
                            attr: "angle".to_string(),
                            value: other.to_string(),
                            message: "expected `degree` or `radian`".to_string(),
                        });
                    }
                },
                "eulerseq" => c.eulerseq = attr.value().to_string(),
                "coordinate" => match attr.value() {
                    "local" => c.coordinate = Coordinate::Local,
                    "global" => {
                        return Err(ParseError::Unsupported {
                            feature: "<compiler coordinate=\"global\">".to_string(),
                            hint: Some(
                                "deprecated MJCF feature; convert your file to local coordinates"
                                    .to_string(),
                            ),
                        });
                    }
                    other => {
                        return Err(ParseError::BadAttributeValue {
                            tag: "compiler".to_string(),
                            attr: "coordinate".to_string(),
                            value: other.to_string(),
                            message: "expected `local` or `global`".to_string(),
                        });
                    }
                },
                "autolimits" => c.autolimits = parse_bool(attr.value())?,
                "inertiafromgeom" => {
                    c.inertia_from_geom = match attr.value() {
                        "true" => InertiaFromGeom::True,
                        "false" => InertiaFromGeom::False,
                        "auto" => InertiaFromGeom::Auto,
                        v => {
                            return Err(ParseError::BadAttributeValue {
                                tag: "compiler".to_string(),
                                attr: "inertiafromgeom".to_string(),
                                value: v.to_string(),
                                message: "expected `true`/`false`/`auto`".to_string(),
                            });
                        }
                    }
                }
                "meshdir" => c.mesh_dir = Some(attr.value().to_string()),
                "texturedir" => c.texture_dir = Some(attr.value().to_string()),
                "assetdir" => c.asset_dir = Some(attr.value().to_string()),
                "strippath" => c.strip_path = parse_bool(attr.value())?,
                "discardvisual" => c.discard_visual = parse_bool(attr.value())?,
                "convexhull" => c.convex_hull = parse_bool(attr.value())?,
                "exactmeshinertia" => c.exact_mesh_inertia = parse_bool(attr.value())?,
                "balanceinertia" => c.balance_inertia = parse_bool(attr.value())?,
                "boundmass" => c.bound_mass = parse_f64(attr.value())?,
                "boundinertia" => c.bound_inertia = parse_f64(attr.value())?,
                "settotalmass" => c.set_total_mass = parse_f64(attr.value())?,
                "inertiadensity" => c.inertia_density = parse_f64(attr.value())?,
                _ => {
                    // Silently ignore other compiler options (not load-relevant).
                }
            }
        }
        Ok(())
    }

    pub(super) fn parse_option(&mut self, node: Node) -> Result<(), ParseError> {
        let o = &mut self.model.option;
        for attr in node.attributes() {
            match attr.name() {
                "timestep" => o.timestep = parse_f64(attr.value())?,
                "gravity" => o.gravity = parse_vec3(attr.value(), "option", "gravity")?,
                _ => {}
            }
        }
        Ok(())
    }
}
