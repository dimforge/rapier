//! Geom → shape / collider conversion. Covers analytic shapes, ellipsoids
//! (via a tiny icosphere), `fromto`, mesh / heightfield assets, plus the
//! per-collider plumbing (groups, friction, margin, hooks).

use mjcf_rs::Pose as MPose;
use mjcf_rs::body as mb;
use mjcf_rs::glam::{DQuat, DVec3};
use mjcf_rs::model::{BodyEntry, BodyId};
use rapier3d::dynamics::RigidBody;
#[cfg(feature = "__meshloader_is_enabled")]
use rapier3d::geometry::MeshConverter;
use rapier3d::geometry::{Collider, Group, InteractionGroups, SharedShape};
use rapier3d::math::{Pose, Real, Rotation, Vector};
use rapier3d::parry::utils::Array2;
use rapier3d::pipeline::ActiveHooks;

use super::conversion::Conversion;
use super::options::ContactFilterMode;

impl<'a> Conversion<'a> {
    /// Build the rapier shape + the geom's body-frame pose. The returned
    /// pose already composes the geom's `pos`/`quat` (or `fromto`) attribute
    /// with the shape's intrinsic offset (e.g. cylinder Z→Y rotation).
    /// Returns `None` for geoms we can't convert (sdf, missing meshes) or
    /// that are filtered out by the loader options (e.g. plane geoms when
    /// `skip_plane_geoms` is `true`).
    pub(super) fn build_geom_shape(&self, g: &mb::Geom) -> Option<(SharedShape, Pose)> {
        self.build_geom_shape_with(g, /*force_trimesh=*/ false)
    }

    /// The geom's body-frame pose: its `pos`/`quat` (or the equivalent
    /// derived from `fromto`), scaled by [`MjcfLoaderOptions::scale`].
    /// The result locates the geom's *origin* in the parent body's
    /// local frame — multiply by any asset-intrinsic offset to obtain
    /// the full pose to assign to the rapier collider / renderer.
    pub(super) fn geom_body_frame_pose(&self, g: &mb::Geom) -> Pose {
        let s = self.options.scale;
        let mut pose = if let Some(ft) = g.fromto {
            from_to_size_and_pose(g.type_, ft, &g.size).1
        } else {
            g.pose
        };
        pose.translation *= s as f64;
        Pose::from(pose)
    }

    /// Same as [`Self::build_geom_shape`] but forces `<mesh>` assets to
    /// be loaded as trimeshes regardless of `<compiler convexhull>` or
    /// [`MjcfLoaderOptions::mesh_converter`](super::MjcfLoaderOptions::mesh_converter).
    /// Used for visual-only geoms where the convex hull is wrong (a
    /// non-convex visual would otherwise render as its hull) and where
    /// the hull computation itself can fail on degenerate visual meshes.
    pub(super) fn build_geom_shape_with(
        &self,
        g: &mb::Geom,
        force_trimesh: bool,
    ) -> Option<(SharedShape, Pose)> {
        if self.options.skip_plane_geoms && matches!(g.type_, mb::GeomType::Plane) {
            return None;
        }
        let s = self.options.scale;
        // Apply optional `fromto`.
        let (size, _pose) = if let Some(ft) = g.fromto {
            from_to_size_and_pose(g.type_, ft, &g.size)
        } else {
            (g.size, g.pose)
        };
        let body_frame_pose = self.geom_body_frame_pose(g);
        let s_real = s;
        let shape = match g.type_ {
            mb::GeomType::Plane => {
                // Half-space normal = +Z.
                Some((SharedShape::halfspace(Vector::Z), Pose::IDENTITY))
            }
            mb::GeomType::Sphere => {
                let r = (size[0] as Real) * s_real;
                Some((SharedShape::ball(r), Pose::IDENTITY))
            }
            mb::GeomType::Capsule => {
                let r = (size[0] as Real) * s_real;
                let half = (size[1] as Real) * s_real;
                Some((SharedShape::capsule_z(half, r), Pose::IDENTITY))
            }
            mb::GeomType::Cylinder => {
                let r = (size[0] as Real) * s_real;
                let half = (size[1] as Real) * s_real;
                // MJCF cylinders are along Z; rapier cylinders along Y. Add
                // a 90° rotation around X.
                let rot = Pose::from_parts(
                    Vector::ZERO,
                    Rotation::from_axis_angle(Vector::X, std::f32::consts::FRAC_PI_2),
                );
                Some((SharedShape::cylinder(half, r), rot))
            }
            mb::GeomType::Box => {
                let x = (size[0] as Real) * s_real;
                let y = (size[1] as Real) * s_real;
                let z = (size[2] as Real) * s_real;
                Some((SharedShape::cuboid(x, y, z), Pose::IDENTITY))
            }
            mb::GeomType::Ellipsoid => {
                // Approximate by an icosphere convex hull scaled per-axis.
                let pts = icosphere_unit();
                let scaled: Vec<_> = pts
                    .iter()
                    .map(|p| {
                        Vector::new(
                            p.x * (size[0] as Real) * s_real,
                            p.y * (size[1] as Real) * s_real,
                            p.z * (size[2] as Real) * s_real,
                        )
                    })
                    .collect();
                SharedShape::convex_hull(&scaled).map(|s| (s, Pose::IDENTITY))
            }
            mb::GeomType::Mesh => self.build_mesh_shape(g, force_trimesh),
            mb::GeomType::Hfield => self.build_hfield_shape(g),
            mb::GeomType::Sdf => {
                log::warn!(
                    "<geom name={:?} type=\"sdf\"> is unsupported in rapier; skipping",
                    g.name,
                );
                None
            }
        };
        shape.map(|(sh, extra)| (sh, body_frame_pose * extra))
    }

    #[cfg(feature = "__meshloader_is_enabled")]
    fn build_mesh_shape(&self, g: &mb::Geom, force_trimesh: bool) -> Option<(SharedShape, Pose)> {
        self.load_mesh_asset(g, force_trimesh)
            .map(|m| (m.shape, m.pose))
    }

    /// Loads the `<mesh>` asset referenced by `g` and returns both the
    /// rapier shape (built through the [`MeshConverter`]) and the
    /// renderer-relevant extras (per-vertex UVs and the resolved
    /// MTL diffuse texture path, when the source asset carried them).
    /// Visual-only geoms route through here so they can drive a
    /// textured renderer; the collider/mass paths drop the extras via
    /// [`Self::build_mesh_shape`].
    #[cfg(feature = "__meshloader_is_enabled")]
    fn load_mesh_asset(&self, g: &mb::Geom, force_trimesh: bool) -> Option<LoadedMesh> {
        let Some(asset_name) = g.mesh.as_deref() else {
            log::warn!(
                "<geom name={:?} type=\"mesh\"> has no `mesh=` attribute; skipping",
                g.name,
            );
            return None;
        };
        let Some(mesh) = self.model.assets.mesh(asset_name) else {
            log::warn!(
                "<geom name={:?} mesh=\"{asset_name}\"> references a mesh that is not declared in <asset>; skipping",
                g.name,
            );
            return None;
        };
        let Some(path) = self.model.resolve_mesh_file(mesh, self.base_dir) else {
            log::warn!(
                "<geom name={:?} mesh=\"{asset_name}\">: mesh has no `file=` attribute and no inline vertex/face data; skipping",
                g.name,
            );
            return None;
        };
        // Warn about file extensions we don't (yet) load. The rapier3d-meshloader
        // backend handles `.stl` (behind `stl`) and `.obj` (behind `wavefront`).
        // `.msh` requires a separate parser that this loader doesn't currently
        // call, even with `mjcf-rs/msh` enabled.
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        match ext.as_deref() {
            Some("msh") => {
                log::warn!(
                    "<geom name={:?} mesh=\"{asset_name}\" file=\"{}\">: MuJoCo's `.msh` binary mesh format is not loaded by rapier3d-mjcf (only `.stl` and `.obj` are supported); skipping",
                    g.name,
                    path.display(),
                );
                return None;
            }
            Some("stl") if !cfg!(feature = "stl") => {
                log::warn!(
                    "<geom name={:?} mesh=\"{asset_name}\" file=\"{}\">: loading STL meshes requires the `stl` cargo feature of rapier3d-mjcf; skipping",
                    g.name,
                    path.display(),
                );
                return None;
            }
            Some("obj") if !cfg!(feature = "wavefront") => {
                log::warn!(
                    "<geom name={:?} mesh=\"{asset_name}\" file=\"{}\">: loading OBJ meshes requires the `wavefront` cargo feature of rapier3d-mjcf; skipping",
                    g.name,
                    path.display(),
                );
                return None;
            }
            _ => {}
        }
        let scale = Vector::new(
            (mesh.scale[0] as Real) * self.options.scale,
            (mesh.scale[1] as Real) * self.options.scale,
            (mesh.scale[2] as Real) * self.options.scale,
        );
        let converter = if force_trimesh {
            // Visual mesh: never go through the convex-hull path, no
            // matter what the compiler / options say. Non-convex
            // visuals would otherwise render as their hull, and the
            // hull computation itself can fail on degenerate visual
            // geometry (coplanar / collinear vertex sets).
            //
            // Also: use *empty* flags here, not `self.options.trimesh_flags`.
            // The default flags include `MERGE_DUPLICATE_VERTICES`, which
            // collapses the raw per-face-corner vertex layout that
            // mesh-loader produces for OBJs. Per-vertex UVs are
            // produced parallel to that layout — once vertices get
            // merged the UV buffer no longer aligns, the renderer
            // discards UVs, and the texture samples at uv=(0,0)
            // uniformly.
            MeshConverter::TriMeshWithFlags(rapier3d::geometry::TriMeshFlags::empty())
        } else {
            self.options
                .mesh_converter
                .unwrap_or(if self.model.compiler.convex_hull {
                    MeshConverter::ConvexHull
                } else {
                    MeshConverter::TriMeshWithFlags(self.options.trimesh_flags)
                })
        };
        let loaded = match rapier3d_meshloader::load_from_path(&path, &converter, scale) {
            Ok(v) => v,
            Err(e) => {
                log::warn!(
                    "<geom name={:?} mesh=\"{asset_name}\" file=\"{}\">: failed to load mesh: {e}",
                    g.name,
                    path.display(),
                );
                return None;
            }
        };
        // Take the first successfully-loaded sub-mesh.
        if let Some(m) = loaded.into_iter().flatten().next() {
            let uvs = if force_trimesh {
                Some(m.raw_mesh.texcoords[0].clone()).filter(|uvs| !uvs.is_empty())
            } else {
                None
            };
            // Recompute smooth, crease-aware vertex normals from the
            // geometry the way MuJoCo does, rather than trusting the file's.
            // Most menagerie robots (e.g. unitree_g1) ship STL, a faceted
            // format: it stores one normal per *triangle* with fully
            // unshared vertices, so neither the raw file normals nor a
            // naive per-face recompute can ever produce anything but flat
            // shading. Welding coincident vertices to recover shared edges
            // and blending the adjacent face normals (honoring the asset's
            // `smoothnormal` crease threshold) yields the smooth look the
            // source models intend. Visual path only — the collision/mass
            // paths have no use for normals.
            let normals = if force_trimesh {
                m.shape.as_trimesh().map(|tm| {
                    let verts: Vec<[f64; 3]> = tm
                        .vertices()
                        .iter()
                        .map(|p| [p.x as f64, p.y as f64, p.z as f64])
                        .collect();
                    mjcf_rs::normals::smooth_vertex_normals(
                        &verts,
                        tm.indices(),
                        mesh.smoothnormal > 0.5,
                    )
                })
            } else {
                None
            };
            let diffuse_texture = m.material.texture.diffuse.clone();
            return Some(LoadedMesh {
                shape: m.shape,
                pose: m.pose,
                uvs,
                normals,
                diffuse_texture,
            });
        }
        log::warn!(
            "<geom name={:?} mesh=\"{asset_name}\" file=\"{}\">: file loaded but no usable sub-mesh could be produced; skipping",
            g.name,
            path.display(),
        );
        None
    }

    #[cfg(not(feature = "__meshloader_is_enabled"))]
    fn build_mesh_shape(&self, g: &mb::Geom, _force_trimesh: bool) -> Option<(SharedShape, Pose)> {
        let asset_name = g.mesh.as_deref().unwrap_or("");
        let file = g
            .mesh
            .as_deref()
            .and_then(|n| self.model.assets.mesh(n))
            .and_then(|m| m.file.as_deref())
            .unwrap_or("");
        log::warn!(
            "<geom name={:?} type=\"mesh\" mesh=\"{asset_name}\" file=\"{file}\">: \
             mesh loading is disabled by default; enable one of rapier3d-mjcf's \
             `stl` or `wavefront` cargo features; skipping",
            g.name,
        );
        None
    }

    fn build_hfield_shape(&self, g: &mb::Geom) -> Option<(SharedShape, Pose)> {
        let Some(name) = g.hfield.as_deref() else {
            log::warn!(
                "<geom name={:?} type=\"hfield\"> has no `hfield=` attribute; skipping",
                g.name,
            );
            return None;
        };
        let Some(h) = self.model.assets.hfield(name) else {
            log::warn!(
                "<geom name={:?} hfield=\"{name}\"> references a heightfield that is not declared in <asset>; skipping",
                g.name,
            );
            return None;
        };
        if h.elevation.is_none() {
            log::warn!(
                "<hfield {name}>: file-based heightfields are not supported yet (use inline `elevation`); skipping",
            );
            return None;
        }
        let elev = h.elevation.as_ref().unwrap();
        let nrow = h.nrow as usize;
        let ncol = h.ncol as usize;
        if elev.len() != nrow * ncol {
            log::warn!(
                "<hfield {name}>: elevation length {} doesn't match nrow×ncol = {}; skipping",
                elev.len(),
                nrow * ncol
            );
            return None;
        }
        let s = self.options.scale;
        let radius_x = (h.size[0] as Real) * s;
        let radius_y = (h.size[1] as Real) * s;
        let elevation_z = (h.size[2] as Real) * s;
        let data: Vec<Real> = elev.iter().map(|e| (*e as Real) * elevation_z).collect();
        let heights = Array2::new(nrow, ncol, data);
        let scale = Vector::new(2.0 * radius_x, 1.0, 2.0 * radius_y);
        let shape = SharedShape::heightfield(heights, scale);
        Some((shape, Pose::IDENTITY))
    }

    /// Convert one MJCF geom into a rapier collider.
    pub(super) fn build_collider(&self, g: &mb::Geom) -> Option<Collider> {
        // `compiler/discardvisual` drops visual-only geoms during compile;
        // honor it the same way at load time.
        let is_visual = g.contype == 0 && g.conaffinity == 0;
        if is_visual && self.model.compiler.discard_visual {
            return None;
        }
        if is_visual && !self.options.create_colliders_from_visual_shapes {
            return None;
        }
        if !is_visual && !self.options.create_colliders_from_collision_shapes {
            return None;
        }
        let (shape, body_frame_pose) = self.build_geom_shape(g)?;
        let mut builder = self.options.collider_blueprint.clone();
        builder.shape = shape;
        builder = builder.position(body_frame_pose);
        builder = builder.friction(g.friction[0] as Real);
        builder = builder.contact_skin(g.margin as Real);
        builder = builder.collision_groups(self.interaction_groups(g));
        // Enable contact-pair filtering / contact-modification hooks
        // unconditionally — the hooks themselves only kick in if the user
        // installs `MjcfContactHooks`. The cost in the absence of hooks
        // is one function-pointer dispatch per pair, which is negligible.
        builder = builder
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS);
        Some(builder.build())
    }

    fn interaction_groups(&self, g: &mb::Geom) -> InteractionGroups {
        match self.options.contact_filter_mode {
            ContactFilterMode::Symmetric => {
                let bits = g.contype | g.conaffinity;
                InteractionGroups::all()
                    .with_memberships(Group::from(bits))
                    .with_filter(Group::from(bits))
            }
            ContactFilterMode::Asymmetric => InteractionGroups::all()
                .with_memberships(Group::from(g.contype))
                .with_filter(Group::from(g.conaffinity)),
        }
    }

    /// Workspace for collider build; cleared every body.
    ///
    /// Also records the name of each created collider into the
    /// `staged_collider_names` workspace so the lookup map can be built
    /// against actual collider indices, not MJCF geom indices (some geoms
    /// get filtered out, which would otherwise misalign the mapping).
    pub(super) fn attach_colliders_to_body(
        &mut self,
        _mjcf_id: BodyId,
        entry: &BodyEntry,
        _world_pose: MPose,
        _body: &mut RigidBody,
    ) {
        self.staged_colliders.clear();
        self.staged_collider_names.clear();
        self.staged_visual_meshes.clear();
        for g in &entry.body.geoms {
            if let Some(c) = self.build_collider(g) {
                self.staged_colliders.push(c);
                self.staged_collider_names.push(g.name.clone());
            } else if let Some(vm) = self.build_visual_mesh(g) {
                self.staged_visual_meshes.push(vm);
            }
        }
        self.synthesize_visuals_for_collider_meshes(&entry.body.geoms);
    }

    /// Bridges the gap between MuJoCo's "every geom is also a visual"
    /// model and rapier's "colliders are the only physics primitive"
    /// model. When a `<geom type="mesh">` is collision-active
    /// (`contype != 0`), the loader turns it into a rapier collider
    /// — typically a convex hull, because MuJoCo's default
    /// `<compiler convexhull="true">` requires it. That collider
    /// follows the `Render colliders` toggle in the testbed, but its
    /// convex-hull approximation is a poor stand-in for the
    /// high-fidelity source mesh under the `Render visual meshes`
    /// toggle.
    ///
    /// To keep the visual channel useful for models that don't split
    /// collision and rendering into separate `<geom>` elements (e.g.
    /// ufactory_xarm7, leap_hand's fingertips), we synthesize a
    /// `MjcfVisualMesh` carrying the **un-hulled** trimesh for each
    /// such geom — gated by MuJoCo's `<geom group>` convention:
    ///
    /// - Groups 0–2 are "render me by default" in MuJoCo's viewer;
    ///   we synthesize a visual for them.
    /// - Group 3+ is "collision proxy / hidden by default"; we skip
    ///   synthesis so models like franka_emika_panda (which carries
    ///   separate `linkN_c.stl` collision meshes in `group=3`
    ///   alongside the high-fidelity OBJs in `group=2`) don't
    ///   double-render.
    ///
    /// This isn't a perfect signal — group 3 is convention, not law
    /// — but it matches what the MuJoCo viewer would show by default,
    /// which is the same answer most users want.
    pub(super) fn synthesize_visuals_for_collider_meshes(&mut self, geoms: &[mb::Geom]) {
        const COLLISION_ONLY_GROUP_THRESHOLD: i32 = 3;
        for g in geoms {
            if g.contype == 0 && g.conaffinity == 0 {
                continue;
            }
            if !matches!(g.type_, mb::GeomType::Mesh) {
                continue;
            }
            if g.mesh.is_none() {
                continue;
            }
            if g.group >= COLLISION_ONLY_GROUP_THRESHOLD {
                continue;
            }
            if let Some(vm) = self.build_visual_mesh_data(g) {
                self.staged_visual_meshes.push(vm);
            }
        }
    }

    /// Build a render-only mesh for a visual MJCF geom (one with
    /// `contype = conaffinity = 0`) when the loader has been configured
    /// to *not* turn visual geoms into colliders. Returns `None` for
    /// geoms that should be skipped entirely (`compiler/discardvisual`,
    /// SDFs, etc.) or that aren't visual to begin with.
    pub(super) fn build_visual_mesh(&self, g: &mb::Geom) -> Option<super::types::MjcfVisualMesh> {
        let is_visual = g.contype == 0 && g.conaffinity == 0;
        if !is_visual {
            return None;
        }
        if self.model.compiler.discard_visual {
            return None;
        }
        if self.options.create_colliders_from_visual_shapes {
            // Caller already built this as a collider.
            return None;
        }
        self.build_visual_mesh_data(g)
    }

    /// Build the `MjcfVisualMesh` payload for `g` without consulting
    /// the visibility gates ([`Self::build_visual_mesh`] applies them).
    /// Useful for synthesizing visuals from a collision-active mesh
    /// geom — those aren't "visual" in MJCF's contype-based sense but
    /// still need a high-fidelity render counterpart.
    fn build_visual_mesh_data(&self, g: &mb::Geom) -> Option<super::types::MjcfVisualMesh> {
        // Mesh assets get the specialized path that also harvests UVs
        // and the MTL diffuse texture. Primitives go through the
        // standard analytic shape path (no UVs, no MTL).
        let (shape, local_pose, uvs, normals, mtl_texture) =
            if matches!(g.type_, mb::GeomType::Mesh) {
                let loaded = self.load_visual_mesh_asset(g)?;
                // `loaded.pose` is the asset-level intrinsic offset (the
                // converter's adjustment — identity for trimeshes). The
                // geom's own `pos`/`quat` haven't been applied yet; multiply
                // by the body-frame pose so the mesh lands where the MJCF
                // places it within its parent body. Without this every
                // mesh visual sits at the body's origin and only the
                // collider path (which composes via `build_geom_shape`)
                // looks correct.
                let body_frame_pose = self.geom_body_frame_pose(g);
                (
                    loaded.shape,
                    body_frame_pose * loaded.pose,
                    loaded.uvs,
                    loaded.normals,
                    loaded.diffuse_texture,
                )
            } else {
                // `build_geom_shape_with` already composes `body_frame_pose * extra`,
                // so the returned `local_pose` is fully placed in the body's frame.
                let (shape, local_pose) =
                    self.build_geom_shape_with(g, /*force_trimesh=*/ true)?;
                (shape, local_pose, None, None, None)
            };

        let (material_rgba, material_texture, material) = self.resolve_geom_material(g);
        let rgba = g
            .rgba
            .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32, c[3] as f32])
            .or(material_rgba);
        let texture = material_texture.or(mtl_texture);

        Some(super::types::MjcfVisualMesh {
            shape,
            local_pose,
            rgba,
            uvs,
            normals,
            texture,
            material,
        })
    }

    /// Mesh-asset variant of [`Self::build_visual_mesh`] that also
    /// surfaces UVs and the OBJ MTL diffuse texture path. Stub when
    /// mesh loading isn't compiled in.
    #[cfg(feature = "__meshloader_is_enabled")]
    fn load_visual_mesh_asset(&self, g: &mb::Geom) -> Option<LoadedMesh> {
        self.load_mesh_asset(g, /*force_trimesh=*/ true)
    }

    #[cfg(not(feature = "__meshloader_is_enabled"))]
    fn load_visual_mesh_asset(&self, _g: &mb::Geom) -> Option<LoadedMesh> {
        None
    }

    /// Resolve a `<geom material="…">` reference into a tuple of
    /// `(rgba, resolved_texture_path)`. Returns `(None, None)` when
    /// the geom has no `material` attribute or the named material
    /// isn't declared.
    fn resolve_geom_material(
        &self,
        g: &mb::Geom,
    ) -> (
        Option<[f32; 4]>,
        Option<std::path::PathBuf>,
        Option<super::types::MjcfRenderMaterial>,
    ) {
        let Some(name) = g.material.as_deref() else {
            return (None, None, None);
        };
        let Some(material) = self.model.assets.material(name) else {
            return (None, None, None);
        };
        let rgba = material
            .rgba
            .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32, c[3] as f32]);
        let texture_path = material
            .texture
            .as_deref()
            .and_then(|tname| self.model.assets.texture(tname))
            .filter(|t| t.file.is_some())
            .and_then(|t| self.model.resolve_texture_file(t, self.base_dir));

        // Map MuJoCo's material model onto metallic-roughness PBR. `metallic`
        // and `roughness` carry the -1 "unset" sentinel (set at parse time);
        // when unset, fall back to the legacy Phong — dielectric (metallic 0)
        // with roughness derived from `shininess`. `specular` becomes the
        // dielectric reflectance, and `emission` scales the material color into
        // an emissive term.
        let metallic = if material.metallic >= 0.0 {
            material.metallic as f32
        } else {
            0.0
        };
        let roughness = if material.roughness >= 0.0 {
            material.roughness as f32
        } else {
            1.0 - material.shininess as f32
        };
        let e = material.emission as f32;
        let base = material.rgba.unwrap_or([1.0, 1.0, 1.0, 1.0]);
        let pbr = super::types::MjcfRenderMaterial {
            metallic: metallic.clamp(0.0, 1.0),
            // A literal 0 roughness is a perfect mirror that reads as a shading
            // bug; clamp to a small floor like real renderers do.
            roughness: roughness.clamp(0.04, 1.0),
            reflectance: (material.specular as f32).clamp(0.0, 1.0),
            emissive: [base[0] as f32 * e, base[1] as f32 * e, base[2] as f32 * e],
        };
        (rgba, texture_path, Some(pbr))
    }
}

/// Bundle returned by the mesh-asset loader so the visual-mesh path can
/// pick up the rapier shape plus the renderer's extras (UVs + MTL
/// diffuse texture) in one shot.
pub(super) struct LoadedMesh {
    pub(super) shape: SharedShape,
    pub(super) pose: Pose,
    pub(super) uvs: Option<Vec<[f32; 2]>>,
    pub(super) normals: Option<Vec<[f32; 3]>>,
    pub(super) diffuse_texture: Option<std::path::PathBuf>,
}

pub(super) fn icosphere_unit() -> Vec<Vector> {
    // Subdivision-1 icosphere (12 vertices). Hard-coded for portability.
    let phi = ((1.0_f64 + 5f64.sqrt()) / 2.0) as Real;
    let mut pts = Vec::with_capacity(12);
    let raw = [
        (-1.0, phi, 0.0),
        (1.0, phi, 0.0),
        (-1.0, -phi, 0.0),
        (1.0, -phi, 0.0),
        (0.0, -1.0, phi),
        (0.0, 1.0, phi),
        (0.0, -1.0, -phi),
        (0.0, 1.0, -phi),
        (phi, 0.0, -1.0),
        (phi, 0.0, 1.0),
        (-phi, 0.0, -1.0),
        (-phi, 0.0, 1.0),
    ];
    for (x, y, z) in raw {
        let n = (x * x + y * y + z * z).sqrt();
        pts.push(Vector::new(x / n, y / n, z / n));
    }
    pts
}

pub(super) fn from_to_size_and_pose(
    type_: mb::GeomType,
    ft: [f64; 6],
    explicit: &[f64; 3],
) -> ([f64; 3], MPose) {
    let p1 = DVec3::new(ft[0], ft[1], ft[2]);
    let p2 = DVec3::new(ft[3], ft[4], ft[5]);
    let mid = (p1 + p2) * 0.5;
    let dir = p2 - p1;
    let len = dir.length();
    let half = len * 0.5;
    // Rotation mapping the world Z axis onto the capsule/box long axis.
    let quat = if len <= 1e-15 {
        DQuat::IDENTITY
    } else {
        DQuat::from_rotation_arc(DVec3::Z, dir / len)
    };
    let mut size = *explicit;
    if matches!(type_, mb::GeomType::Capsule | mb::GeomType::Cylinder) {
        size[1] = half;
    }
    if matches!(type_, mb::GeomType::Box) {
        size[2] = half;
    }
    (size, MPose::from_parts(mid, quat))
}
