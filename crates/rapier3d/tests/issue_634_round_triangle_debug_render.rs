//! Regression test for #634 (3D side): debug-rendering a `RoundTriangle`
//! collider must not panic. Parry doesn’t provide a rounded-triangle outline in
//! 3D yet, so the renderer falls back to the flat triangle.
#![cfg(feature = "debug-render")]

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

#[test]
fn round_triangle_debug_render_does_not_panic() {
    let bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(ColliderBuilder::round_triangle(
        Vector::new(-1.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        0.3,
    ));

    let mut pipeline = DebugRenderPipeline::new(
        DebugRenderStyle::default(),
        DebugRenderMode::COLLIDER_SHAPES,
    );
    let mut backend = LineCollector::default();
    pipeline.render_colliders(&mut backend, &bodies, &colliders);
    assert!(!backend.lines.is_empty());
}
