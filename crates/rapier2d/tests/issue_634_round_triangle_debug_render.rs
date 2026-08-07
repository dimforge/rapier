//! Regression test for #634: the debug-renderer ignored the border radius of
//! `RoundTriangle` colliders and rendered them as flat triangles.
#![cfg(feature = "debug-render")]

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

fn render_collider_lines(builder: ColliderBuilder) -> Vec<(Vector, Vector)> {
    let bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(builder);

    let mut pipeline = DebugRenderPipeline::new(
        DebugRenderStyle::default(),
        DebugRenderMode::COLLIDER_SHAPES,
    );
    let mut backend = LineCollector::default();
    pipeline.render_colliders(&mut backend, &bodies, &colliders);
    backend.lines
}

#[test]
fn round_triangle_renders_rounded_border() {
    let (a, b, c) = (
        Vector::new(-1.0, 0.0),
        Vector::new(1.0, 0.0),
        Vector::new(0.0, 1.0),
    );
    let radius = 0.3;

    let flat = render_collider_lines(ColliderBuilder::triangle(a, b, c));
    let round = render_collider_lines(ColliderBuilder::round_triangle(a, b, c, radius));

    // The flat triangle is exactly 3 segments; the rounded one must include the
    // dilated border (arcs at each corner), hence strictly more segments.
    assert_eq!(flat.len(), 3);
    assert!(
        round.len() > flat.len(),
        "round triangle rendered with only {} segments",
        round.len()
    );

    // The rendered rounded boundary must actually be inflated by the border
    // radius: some vertex must lie outside the flat triangle's AABB.
    let max_y = round
        .iter()
        .flat_map(|(p1, p2)| [p1.y, p2.y])
        .fold(f32::MIN, f32::max);
    assert!(
        max_y > 1.0 + radius * 0.5,
        "rounded border not inflated (max_y = {max_y})"
    );
}
