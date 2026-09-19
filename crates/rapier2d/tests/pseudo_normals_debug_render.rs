//! The debug-renderer draws a polyline's pseudo-normals when it has them, and nothing otherwise.
#![cfg(feature = "debug-render")]

use rapier2d::parry::shape::{Polyline, PolylineFlags};
use rapier2d::pipeline::{
    DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline, DebugRenderStyle,
};
use rapier2d::prelude::*;

#[derive(Default)]
struct LineCollector {
    lines: Vec<(Vector, Vector)>,
}

impl DebugRenderBackend for LineCollector {
    fn draw_line(&mut self, _: DebugRenderObject, a: Vector, b: Vector, _: [f32; 4]) {
        self.lines.push((a, b));
    }
}

fn render_square_pseudo_normals(flags: PolylineFlags) -> (LineCollector, usize) {
    // A counter-clockwise square, so the pseudo-normals point outward.
    let vertices = vec![
        Vector::new(-1.0, -1.0),
        Vector::new(1.0, -1.0),
        Vector::new(1.0, 1.0),
        Vector::new(-1.0, 1.0),
    ];
    let indices = vec![[0, 1], [1, 2], [2, 3], [3, 0]];
    let num_vertices = vertices.len();
    let polyline = Polyline::with_flags(vertices, Some(indices), flags);

    let bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(ColliderBuilder::new(SharedShape::new(polyline)));

    // Only `PSEUDO_NORMALS`: the pseudo-normals must not depend on the shapes being drawn.
    let mut pipeline =
        DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::PSEUDO_NORMALS);
    let mut backend = LineCollector::default();
    pipeline.render_colliders(&mut backend, &bodies, &colliders);
    (backend, num_vertices)
}

#[test]
fn oriented_polyline_draws_one_normal_per_vertex() {
    let (backend, num_vertices) = render_square_pseudo_normals(PolylineFlags::ORIENTED);
    assert_eq!(backend.lines.len(), num_vertices);

    let len = DebugRenderStyle::default().pseudo_normal_length;
    for (a, b) in &backend.lines {
        let normal = *b - *a;
        assert!((normal.length() - len).abs() < 1.0e-5);
        // A square's corner pseudo-normals are its outward diagonals.
        assert!(normal.dot(*a) > 0.0);
    }
}

#[test]
fn polyline_without_pseudo_normals_draws_nothing() {
    let (backend, _) = render_square_pseudo_normals(PolylineFlags::empty());
    assert!(backend.lines.is_empty());
}
