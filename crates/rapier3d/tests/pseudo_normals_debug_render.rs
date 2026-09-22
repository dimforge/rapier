//! The debug-renderer draws a mesh's pseudo-normals when it has them, and nothing otherwise.
#![cfg(feature = "debug-render")]

use rapier3d::parry::shape::TriMeshFlags;
use rapier3d::pipeline::{
    DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline, DebugRenderStyle,
};
use rapier3d::prelude::*;

#[derive(Default)]
struct LineCollector {
    lines: Vec<(Vector, Vector)>,
}

impl DebugRenderBackend for LineCollector {
    fn draw_line(&mut self, _: DebugRenderObject, a: Vector, b: Vector, _: [f32; 4]) {
        self.lines.push((a, b));
    }
}

fn render_cube_pseudo_normals(flags: TriMeshFlags) -> (LineCollector, usize, usize) {
    let (vtx, idx) = Cuboid::new(Vector::splat(0.5)).to_trimesh();
    let (num_vertices, num_triangles) = (vtx.len(), idx.len());

    let bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(ColliderBuilder::trimesh_with_flags(vtx, idx, flags).unwrap());

    // Only `PSEUDO_NORMALS`: the pseudo-normals must not depend on the shapes being drawn.
    let mut pipeline =
        DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::PSEUDO_NORMALS);
    let mut backend = LineCollector::default();
    pipeline.render_colliders(&mut backend, &bodies, &colliders);
    (backend, num_vertices, num_triangles)
}

#[test]
fn oriented_trimesh_draws_one_normal_per_vertex_and_triangle_edge() {
    let (backend, num_vertices, num_triangles) = render_cube_pseudo_normals(TriMeshFlags::ORIENTED);
    assert_eq!(backend.lines.len(), num_vertices + 3 * num_triangles);

    let len = DebugRenderStyle::default().pseudo_normal_length;
    for (a, b) in &backend.lines {
        assert!(((*b - *a).length() - len).abs() < 1.0e-5);
    }
}

#[test]
fn trimesh_without_pseudo_normals_draws_nothing() {
    let (backend, ..) = render_cube_pseudo_normals(TriMeshFlags::empty());
    assert!(backend.lines.is_empty());
}
