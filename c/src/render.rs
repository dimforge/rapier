//! Shape geometry for foreign renderers. Tessellation is independent of graphics libraries.
use crate::*;
use rapier::parry::shape::{Shape, TypedShape};

/// Owned tessellated shape: flat triangle vertices and independent line segments, in local space.
/// Rounded 3D shapes use their inner surface (as in the Rust testbed). Halfspaces use a finite
/// patch.
/// @ingroup shapes
pub struct RprShapeMesh {
    triangles: Vec<RprVector>,
    lines: Vec<RprVector>,
}

impl RprShapeMesh {
    fn triangle_mesh(&mut self, pose: Pose, vertices: Vec<Vector>, indices: Vec<[u32; 3]>) {
        for tri in indices {
            for i in tri {
                self.triangles.push((pose * vertices[i as usize]).into());
            }
        }
    }
    fn line_mesh(&mut self, pose: Pose, vertices: Vec<Vector>, indices: Vec<[u32; 2]>) {
        for segment in indices {
            for i in segment {
                self.lines.push((pose * vertices[i as usize]).into());
            }
        }
    }
    #[cfg(feature = "dim2")]
    fn polygon(&mut self, pose: Pose, vertices: Vec<Vector>) {
        for i in 1..vertices.len().saturating_sub(1) {
            for j in [0, i, i + 1] {
                self.triangles.push((pose * vertices[j]).into());
            }
        }
    }
    fn append(&mut self, shape: &dyn Shape, pose: Pose, n: u32) -> Result<()> {
        match shape.as_typed_shape() {
            TypedShape::Compound(c) => {
                for (p, s) in c.shapes() {
                    self.append(s.as_ref(), pose * *p, n)?;
                }
            }
            TypedShape::Segment(s) => self.line_mesh(pose, vec![s.a, s.b], vec![[0, 1]]),
            TypedShape::Polyline(p) => {
                self.line_mesh(pose, p.vertices().to_vec(), p.indices().to_vec())
            }
            TypedShape::Triangle(t) => {
                self.triangle_mesh(pose, vec![t.a, t.b, t.c], vec![[0, 1, 2]])
            }
            TypedShape::RoundTriangle(t) => self.append(&t.inner_shape, pose, n)?,
            TypedShape::TriMesh(m) => {
                self.triangle_mesh(pose, m.vertices().to_vec(), m.indices().to_vec())
            }
            TypedShape::HalfSpace(h) => {
                #[cfg(feature = "dim2")]
                {
                    let tangent = Vector::new(-h.normal.y, h.normal.x) * 1000.0;
                    self.line_mesh(pose, vec![-tangent, tangent], vec![[0, 1]]);
                }
                #[cfg(feature = "dim3")]
                {
                    let normal = h.normal;
                    let tangent = normal.any_orthonormal_vector() * 1000.0;
                    let bitangent = normal.cross(tangent);
                    self.triangle_mesh(
                        pose,
                        vec![
                            -tangent - bitangent,
                            tangent - bitangent,
                            tangent + bitangent,
                            -tangent + bitangent,
                        ],
                        vec![[0, 1, 2], [0, 2, 3]],
                    );
                }
            }
            #[cfg(feature = "dim2")]
            TypedShape::Ball(s) => self.polygon(pose, s.to_polyline(n)),
            #[cfg(feature = "dim2")]
            TypedShape::Cuboid(s) => self.polygon(pose, s.to_polyline()),
            #[cfg(feature = "dim2")]
            TypedShape::RoundCuboid(s) => self.polygon(pose, s.to_polyline(n / 4)),
            #[cfg(feature = "dim2")]
            TypedShape::Capsule(s) => self.polygon(pose, s.to_polyline(n)),
            #[cfg(feature = "dim2")]
            TypedShape::ConvexPolygon(s) => self.polygon(pose, s.points().to_vec()),
            #[cfg(feature = "dim2")]
            TypedShape::RoundConvexPolygon(s) => self.polygon(pose, s.to_polyline(n / 4)),
            #[cfg(feature = "dim2")]
            TypedShape::HeightField(s) => {
                let (v, i) = s.to_polyline();
                self.line_mesh(pose, v, i);
            }
            #[cfg(feature = "dim2")]
            TypedShape::Voxels(s) => {
                let (v, i) = s.to_polyline();
                self.line_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Ball(s) => {
                let (v, i) = s.to_trimesh(n, n / 2);
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Cuboid(s) => {
                let (v, i) = s.to_trimesh();
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Capsule(s) => {
                let (v, i) = s.to_trimesh(n, n / 2);
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Cylinder(s) => {
                let (v, i) = s.to_trimesh(n);
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Cone(s) => {
                let (v, i) = s.to_trimesh(n);
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::RoundCuboid(s) => self.append(&s.inner_shape, pose, n)?,
            #[cfg(feature = "dim3")]
            TypedShape::RoundCylinder(s) => self.append(&s.inner_shape, pose, n)?,
            #[cfg(feature = "dim3")]
            TypedShape::RoundCone(s) => self.append(&s.inner_shape, pose, n)?,
            #[cfg(feature = "dim3")]
            TypedShape::ConvexPolyhedron(s) => {
                let (v, i) = s.to_trimesh();
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::RoundConvexPolyhedron(s) => self.append(&s.inner_shape, pose, n)?,
            #[cfg(feature = "dim3")]
            TypedShape::HeightField(s) => {
                let (v, i) = s.to_trimesh();
                self.triangle_mesh(pose, v, i);
            }
            #[cfg(feature = "dim3")]
            TypedShape::Voxels(s) => {
                let (v, i) = s.to_trimesh();
                self.triangle_mesh(pose, v, i);
            }
            TypedShape::Custom(_) => {
                return Err((
                    RPR_UNSUPPORTED,
                    "custom shapes cannot be tessellated".into(),
                ));
            }
        }
        Ok(())
    }
}
/// Process-local identity of the immutable shape allocation, for render caches. Keep an owned
/// SharedShape clone alive while caching this value. Not serializable; does not identify equal
/// geometry.
pub(crate) unsafe fn native_collider_shape_identity(
    collider: *const RprCollider,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        output(
            out,
            get(collider)?.0.shape() as *const dyn Shape as *const () as usize,
        )
    })
}
/// Return owned local-space rendering geometry; release it with rpr_free_shape_mesh. subdivisions
/// controls curved-shape resolution.
/// @ingroup shapes
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_tessellate(
    shape: *const RprSharedShape,
    subdivisions: u32,
) -> *mut RprShapeMesh {
    ffi_value(|out: *mut *mut RprShapeMesh| {
        ffi(|| unsafe {
            out_ptr(out)?;
            ensure(
                (8..=128).contains(&subdivisions),
                "subdivisions must be between 8 and 128",
            )?;
            let mut mesh = RprShapeMesh {
                triangles: Vec::new(),
                lines: Vec::new(),
            };
            mesh.append(get(shape)?.0.as_ref(), Pose::IDENTITY, subdivisions)?;
            output(out, Box::into_raw(Box::new(mesh)))
        })
    })
}
/// Flat groups of three vertices. Standard output-buffer convention.
/// @see @ref output_buffers
/// @ingroup shapes
#[rapier_export(shape_mesh)]
pub unsafe extern "C" fn rpr_shape_mesh_triangles(
    mesh: *const RprShapeMesh,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe { copy_out(&get(mesh)?.triangles, buffer, capacity, count) })
    })
}
/// Flat groups of two vertices. Standard output-buffer convention.
/// @see @ref output_buffers
/// @ingroup shapes
#[rapier_export(shape_mesh)]
pub unsafe extern "C" fn rpr_shape_mesh_lines(
    mesh: *const RprShapeMesh,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe { copy_out(&get(mesh)?.lines, buffer, capacity, count) })
    })
}
/// Release an owned shape mesh. NULL is allowed. Do not pass borrowed pointers or free the object
/// twice.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_free_shape_mesh(mesh: *mut RprShapeMesh) -> RprStatus {
    ffi(|| unsafe {
        if !mesh.is_null() {
            get(mesh)?;
            drop(Box::from_raw(mesh));
        }
        Ok(())
    })
}
/// Create an owned round cylinder shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_round_cylinder_shared_shape(
    half_height: RprReal,
    radius: RprReal,
    border_radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(
                out,
                Box::into_raw(Box::new(RprSharedShape(SharedShape::round_cylinder(
                    positive(half_height)?,
                    positive(radius)?,
                    nonnegative(border_radius)?,
                )))),
            )
        })
    })
}

/// Owned indexed geometry from Parry's shape tessellation, preserving its vertex order.
/// @ingroup shapes
#[cfg(feature = "dim3")]
pub struct RprTriMeshData {
    vertices: Vec<RprVector>,
    indices: Vec<u32>,
}

/// Tessellate a ball or capsule with independent longitude/latitude subdivision counts.
/// Cuboids, cones, cylinders, convex polyhedra, trimeshes, and heightfields are also supported.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_to_trimesh(
    shape: *const RprSharedShape,
    ntheta: u32,
    nphi: u32,
) -> *mut RprTriMeshData {
    ffi_value(|out: *mut *mut RprTriMeshData| {
        ffi(|| unsafe {
            out_ptr(out)?;
            ensure(
                (3..=4096).contains(&ntheta) && (2..=4096).contains(&nphi),
                "invalid tessellation subdivisions",
            )?;
            let (vertices, indices) = match get(shape)?.0.as_typed_shape() {
                TypedShape::Ball(s) => s.to_trimesh(ntheta, nphi),
                TypedShape::Capsule(s) => s.to_trimesh(ntheta, nphi),
                TypedShape::Cuboid(s) => s.to_trimesh(),
                TypedShape::Cone(s) => s.to_trimesh(ntheta),
                TypedShape::Cylinder(s) => s.to_trimesh(ntheta),
                TypedShape::ConvexPolyhedron(s) => s.to_trimesh(),
                TypedShape::TriMesh(s) => (s.vertices().to_vec(), s.indices().to_vec()),
                TypedShape::HeightField(s) => s.to_trimesh(),
                _ => return Err((RPR_UNSUPPORTED, "shape has no indexed tessellation".into())),
            };
            let mesh = RprTriMeshData {
                vertices: vertices.into_iter().map(Into::into).collect(),
                indices: indices.into_iter().flatten().collect(),
            };
            output(out, Box::into_raw(Box::new(mesh)))
        })
    })
}

/// Copy vertices.
/// @see @ref output_buffers
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export(tri_mesh_data)]
pub unsafe extern "C" fn rpr_tri_mesh_data_vertices(
    mesh: *const RprTriMeshData,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe { copy_out(&get(mesh)?.vertices, buffer, capacity, count) })
    })
}

/// Flat triangle indices; count and capacity are numbers of u32 entries.
/// @see @ref output_buffers
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export(tri_mesh_data)]
pub unsafe extern "C" fn rpr_tri_mesh_data_indices(
    mesh: *const RprTriMeshData,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe { copy_out(&get(mesh)?.indices, buffer, capacity, count) })
    })
}

/// Release an owned tri mesh data. NULL is allowed. Do not pass borrowed pointers or free the
/// object twice.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_free_tri_mesh_data(mesh: *mut RprTriMeshData) -> RprStatus {
    ffi(|| unsafe {
        if !mesh.is_null() {
            drop(Box::from_raw(mesh));
        }
        Ok(())
    })
}
