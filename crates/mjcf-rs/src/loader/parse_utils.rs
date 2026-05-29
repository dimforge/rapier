//! Tiny attribute parsers (`parse_bool`, `parse_f64`, `parse_vec*`, …) and
//! cross-cutting helpers used by the rest of the loader (name prefixing
//! for `<attach>` splicing, asset-path absolutization, default-name
//! computation, the non-`quat` rotation specifications).

use std::path::Path;

use crate::body::{Geom, GeomType, JointType, Site};
use crate::error::ParseError;
use crate::model::Model;
use crate::types::Tristate;
use glamx::glam::{DMat3, DQuat, DVec3};

pub(super) fn parse_bool(s: &str) -> Result<bool, ParseError> {
    match s {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        v => Err(ParseError::BadAttributeValue {
            tag: "_".to_string(),
            attr: "_".to_string(),
            value: v.to_string(),
            message: "expected `true`/`false`".to_string(),
        }),
    }
}

pub(super) fn parse_f64(s: &str) -> Result<f64, ParseError> {
    s.trim()
        .parse::<f64>()
        .map_err(|e| ParseError::BadAttribute {
            tag: "_".to_string(),
            attr: "_".to_string(),
            message: format!("not a float: {e}"),
        })
}

pub(super) fn parse_u32(s: &str) -> Result<u32, ParseError> {
    s.trim()
        .parse::<u32>()
        .map_err(|e| ParseError::BadAttribute {
            tag: "_".to_string(),
            attr: "_".to_string(),
            message: format!("not an unsigned integer: {e}"),
        })
}

pub(super) fn parse_i32(s: &str) -> Result<i32, ParseError> {
    s.trim()
        .parse::<i32>()
        .map_err(|e| ParseError::BadAttribute {
            tag: "_".to_string(),
            attr: "_".to_string(),
            message: format!("not an integer: {e}"),
        })
}

pub(super) fn parse_f64_list(s: &str) -> Result<Vec<f64>, ParseError> {
    s.split_ascii_whitespace()
        .map(|t| {
            t.parse::<f64>().map_err(|e| ParseError::BadAttribute {
                tag: "_".to_string(),
                attr: "_".to_string(),
                message: format!("not a float: {e}"),
            })
        })
        .collect()
}

pub(super) fn parse_vec_n<const N: usize>(
    s: &str,
    tag: &str,
    attr: &str,
) -> Result<[f64; N], ParseError> {
    let v = parse_f64_list(s)?;
    if v.len() != N {
        return Err(ParseError::BadAttributeValue {
            tag: tag.to_string(),
            attr: attr.to_string(),
            value: s.to_string(),
            message: format!("expected {N} floats, got {}", v.len()),
        });
    }
    let mut out = [0.0; N];
    out.copy_from_slice(&v);
    Ok(out)
}

pub(super) fn parse_vec2(s: &str, tag: &str, attr: &str) -> Result<[f64; 2], ParseError> {
    parse_vec_n::<2>(s, tag, attr)
}
pub(super) fn parse_vec3(s: &str, tag: &str, attr: &str) -> Result<[f64; 3], ParseError> {
    parse_vec_n::<3>(s, tag, attr)
}
pub(super) fn parse_vec4(s: &str, tag: &str, attr: &str) -> Result<[f64; 4], ParseError> {
    parse_vec_n::<4>(s, tag, attr)
}
pub(super) fn parse_vec6(s: &str, tag: &str, attr: &str) -> Result<[f64; 6], ParseError> {
    parse_vec_n::<6>(s, tag, attr)
}

/// `size` accepts 1..=3 floats; we right-pad with zeros.
pub(super) fn parse_vec3_lax(s: &str, _tag: &str, _attr: &str) -> Result<[f64; 3], ParseError> {
    let v = parse_f64_list(s)?;
    let mut out = [0.0; 3];
    for (i, x) in v.iter().take(3).enumerate() {
        out[i] = *x;
    }
    Ok(out)
}

/// `friction` accepts 1..=3 floats with the missing entries filling the
/// MJCF defaults `(slide=1.0, spin=0.005, roll=0.0001)`.
pub(super) fn parse_friction(s: &str) -> Result<[f64; 3], ParseError> {
    let v = parse_f64_list(s)?;
    let mut out = [1.0, 0.005, 0.0001];
    for (i, x) in v.iter().take(3).enumerate() {
        out[i] = *x;
    }
    Ok(out)
}

/// `gear` is up to 6 floats.
pub(super) fn parse_gear(s: &str) -> Result<[f64; 6], ParseError> {
    let v = parse_f64_list(s)?;
    let mut out = [0.0; 6];
    if v.is_empty() {
        out[0] = 1.0;
        return Ok(out);
    }
    for (i, x) in v.iter().take(6).enumerate() {
        out[i] = *x;
    }
    Ok(out)
}

pub(super) fn parse_quat(s: &str, tag: &str, attr: &str) -> Result<DQuat, ParseError> {
    let v = parse_vec4(s, tag, attr)?;
    Ok(quat_wxyz(v))
}

/// Build a unit quaternion from MJCF's canonical `(w, x, y, z)` order. A zero
/// (degenerate) input collapses to the identity rather than producing NaNs.
fn quat_wxyz(v: [f64; 4]) -> DQuat {
    // glam stores quaternions as (x, y, z, w).
    let q = DQuat::from_xyzw(v[1], v[2], v[3], v[0]);
    if q.length_squared() <= 1e-30 {
        DQuat::IDENTITY
    } else {
        q.normalize()
    }
}

pub(super) fn parse_tristate(s: &str) -> Result<Tristate, ParseError> {
    match s {
        "true" | "1" => Ok(Tristate::True),
        "false" | "0" => Ok(Tristate::False),
        "auto" => Ok(Tristate::Auto),
        v => Err(ParseError::BadAttributeValue {
            tag: "_".to_string(),
            attr: "_".to_string(),
            value: v.to_string(),
            message: "expected `true`/`false`/`auto`".to_string(),
        }),
    }
}

pub(super) fn parse_joint_type(s: &str) -> Result<JointType, ParseError> {
    match s {
        "hinge" => Ok(JointType::Hinge),
        "slide" => Ok(JointType::Slide),
        "ball" => Ok(JointType::Ball),
        "free" => Ok(JointType::Free),
        v => Err(ParseError::BadAttributeValue {
            tag: "joint".to_string(),
            attr: "type".to_string(),
            value: v.to_string(),
            message: "expected hinge/slide/ball/free".to_string(),
        }),
    }
}

pub(super) fn parse_geom_type(s: &str) -> Result<GeomType, ParseError> {
    match s {
        "plane" => Ok(GeomType::Plane),
        "hfield" => Ok(GeomType::Hfield),
        "sphere" => Ok(GeomType::Sphere),
        "capsule" => Ok(GeomType::Capsule),
        "ellipsoid" => Ok(GeomType::Ellipsoid),
        "cylinder" => Ok(GeomType::Cylinder),
        "box" => Ok(GeomType::Box),
        "mesh" => Ok(GeomType::Mesh),
        "sdf" => Ok(GeomType::Sdf),
        v => Err(ParseError::BadAttributeValue {
            tag: "geom".to_string(),
            attr: "type".to_string(),
            value: v.to_string(),
            message: "unknown geom type".to_string(),
        }),
    }
}

pub(super) fn deg_to_rad(d: f64) -> f64 {
    d * std::f64::consts::PI / 180.0
}

/// Parse one of the rotation specifications other than `quat`. Reads
/// `angle_is_degree` / `eulerseq` off the compiler block so callers don't
/// have to thread them through.
pub(super) fn parse_rotation_attr(
    name: &str,
    value: &str,
    tag: &str,
    compiler: &crate::compiler::Compiler,
) -> Result<DQuat, ParseError> {
    match name {
        "axisangle" => {
            let v = parse_vec4(value, tag, "axisangle")?;
            let mut angle = v[3];
            if compiler.angle_is_degree {
                angle = deg_to_rad(angle);
            }
            Ok(quat_from_axis_angle([v[0], v[1], v[2]], angle))
        }
        "euler" => {
            let mut v = parse_vec3(value, tag, "euler")?;
            if compiler.angle_is_degree {
                v[0] = deg_to_rad(v[0]);
                v[1] = deg_to_rad(v[1]);
                v[2] = deg_to_rad(v[2]);
            }
            Ok(quat_from_euler(v, &compiler.eulerseq))
        }
        "xyaxes" => {
            let v = parse_vec6(value, tag, "xyaxes")?;
            Ok(quat_from_xy_axes([v[0], v[1], v[2]], [v[3], v[4], v[5]]))
        }
        "zaxis" => {
            let v = parse_vec3(value, tag, "zaxis")?;
            Ok(quat_from_z_axis(v))
        }
        _ => unreachable!(),
    }
}

/// Quaternion from an axis (auto-normalized) and an angle in radians. A
/// zero-length axis yields the identity.
fn quat_from_axis_angle(axis: [f64; 3], angle_rad: f64) -> DQuat {
    let axis = DVec3::from_array(axis);
    let len = axis.length();
    if len <= 1e-15 {
        DQuat::IDENTITY
    } else {
        DQuat::from_axis_angle(axis / len, angle_rad)
    }
}

/// Quaternion from Euler angles using the given axis sequence.
///
/// `seq` is a 3-character string drawn from `{x, y, z, X, Y, Z}` that names
/// the axes in *intrinsic* application order (MJCF default `"xyz"`).
/// Lowercase = intrinsic (rotating frame), uppercase = extrinsic (fixed
/// frame). MJCF uses intrinsic by default.
fn quat_from_euler(angles: [f64; 3], seq: &str) -> DQuat {
    let mut q = DQuat::IDENTITY;
    for (i, ch) in seq.chars().enumerate() {
        let angle = angles[i];
        let intrinsic = ch.is_ascii_lowercase();
        let axis = match ch.to_ascii_lowercase() {
            'x' => DVec3::X,
            'y' => DVec3::Y,
            'z' => DVec3::Z,
            _ => continue,
        };
        let r = DQuat::from_axis_angle(axis, angle);
        q = if intrinsic { q * r } else { r * q };
    }
    q.normalize()
}

/// Rotation that maps the world Z axis onto `axis`. Used for `zaxis="..."`.
fn quat_from_z_axis(axis: [f64; 3]) -> DQuat {
    let axis = DVec3::from_array(axis);
    let len = axis.length();
    if len <= 1e-15 {
        DQuat::IDENTITY
    } else {
        DQuat::from_rotation_arc(DVec3::Z, axis / len)
    }
}

/// Rotation from `xyaxes="x1 x2 x3 y1 y2 y3"`. The Z axis is X × Y; Y is then
/// re-orthonormalized (Z × X) to guarantee an orthonormal basis.
fn quat_from_xy_axes(x: [f64; 3], y: [f64; 3]) -> DQuat {
    let x = DVec3::from_array(x);
    let nx = x.length();
    if nx <= 1e-15 {
        return DQuat::IDENTITY;
    }
    let x = x / nx;
    let z = x.cross(DVec3::from_array(y));
    let nz = z.length();
    if nz <= 1e-15 {
        return DQuat::IDENTITY;
    }
    let z = z / nz;
    let y = z.cross(x);
    DQuat::from_mat3(&DMat3::from_cols(x, y, z)).normalize()
}

/// Prefix a name so attached sub-model entities don't collide with the
/// parent's namespace. Empty `prefix` is a no-op (some MJCFs `<attach>`
/// without one; that's their problem if it collides).
pub(super) fn prefixed(prefix: &str, name: &str) -> String {
    if prefix.is_empty() {
        name.to_string()
    } else {
        format!("{prefix}{name}")
    }
}

/// Prefix every external reference carried by a geom: its own name plus
/// the `mesh=` / `material=` / `hfield=` asset references.
pub(super) fn prefix_geom_refs(g: &mut Geom, prefix: &str) {
    if let Some(n) = g.name.take() {
        g.name = Some(prefixed(prefix, &n));
    }
    if let Some(n) = g.mesh.take() {
        g.mesh = Some(prefixed(prefix, &n));
    }
    if let Some(n) = g.material.take() {
        g.material = Some(prefixed(prefix, &n));
    }
    if let Some(n) = g.hfield.take() {
        g.hfield = Some(prefixed(prefix, &n));
    }
}

pub(super) fn prefix_site_refs(s: &mut Site, prefix: &str) {
    if let Some(n) = s.name.take() {
        s.name = Some(prefixed(prefix, &n));
    }
    if let Some(n) = s.material.take() {
        s.material = Some(prefixed(prefix, &n));
    }
}

/// Rewrite every asset `file=` path in `model` to an absolute path,
/// resolved against the model's own `base_dir` and `meshdir` /
/// `texturedir`. Used right after loading a sub-model so the parent
/// can merge the sub-model's assets without dragging along the
/// sub-model's directory layout. Mesh resolution is delegated to the
/// existing [`Model::resolve_mesh_file`] / [`Model::resolve_texture_file`]
/// helpers so the asset-dir / strip-path rules stay consistent.
pub(super) fn absolutize_model_assets(model: &mut Model, base_dir: &Path) {
    // We have to clone the relevant fields first because the resolver
    // methods borrow `&Model`.
    let mesh_indices: Vec<_> = (0..model.assets.meshes.len()).collect();
    for i in mesh_indices {
        let mesh_snapshot = model.assets.meshes[i].clone();
        if let Some(abs) = model.resolve_mesh_file(&mesh_snapshot, base_dir) {
            model.assets.meshes[i].file = Some(abs.to_string_lossy().into_owned());
        }
    }
    let tex_indices: Vec<_> = (0..model.assets.textures.len()).collect();
    for i in tex_indices {
        let tex_snapshot = model.assets.textures[i].clone();
        if let Some(abs) = model.resolve_texture_file(&tex_snapshot, base_dir) {
            model.assets.textures[i].file = Some(abs.to_string_lossy().into_owned());
        }
    }
    // Hfields use the same base_dir / assetdir convention; do them inline
    // because `Model` doesn't currently expose a resolver helper for them.
    for h in &mut model.assets.hfields {
        if let Some(file) = &h.file {
            let path = Path::new(file);
            if !path.is_absolute() {
                let dir = model
                    .compiler
                    .asset_dir
                    .as_deref()
                    .map(Path::new)
                    .map(Path::to_path_buf);
                let resolved = if let Some(dir) = dir {
                    if dir.is_absolute() {
                        dir.join(path)
                    } else {
                        base_dir.join(dir).join(path)
                    }
                } else {
                    base_dir.join(path)
                };
                h.file = Some(resolved.to_string_lossy().into_owned());
            }
        }
    }
    // After the rewrite, every remaining `file=` path is absolute; the
    // parent's `meshdir` / `texturedir` won't interfere.
    model.compiler.mesh_dir = None;
    model.compiler.texture_dir = None;
    model.compiler.asset_dir = None;
}

/// MuJoCo's fallback for `<mesh>`, `<hfield>`, `<texture>`: when the
/// `name=` attribute is omitted, the asset is registered under the file's
/// basename without extension (e.g. `assets/hip.obj` → `"hip"`).
pub(super) fn asset_default_name(file: Option<&str>) -> Option<String> {
    let file = file?;
    let stem = std::path::Path::new(file).file_stem()?.to_str()?;
    Some(stem.to_string())
}
