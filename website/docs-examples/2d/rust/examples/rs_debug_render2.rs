use rapier2d::prelude::*;

// DOCUSAURUS: DebugRenderBackend start
// The backend receives the lines to be drawn. A real one would push them to the renderer of the
// application instead of collecting them.
struct LineCollector {
    lines: Vec<(Vector, Vector, DebugColor)>,
}

impl DebugRenderBackend for LineCollector {
    fn draw_line(&mut self, _object: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        self.lines.push((a, b, color));
    }
}
// DOCUSAURUS: DebugRenderBackend stop

fn main() {
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1), None);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0)),
        ColliderBuilder::ball(0.5),
    );

    // DOCUSAURUS: DebugRender start
    // The style gives the colors and sizes, the mode selects what is drawn.
    let mut debug_render = DebugRenderPipeline::new(
        DebugRenderStyle::default(),
        DebugRenderMode::COLLIDER_SHAPES | DebugRenderMode::CONTACTS,
    );
    let mut backend = LineCollector { lines: vec![] };

    for _ in 0..10 {
        world.step();

        // The debug-rendering is done after the step, once per frame to be drawn.
        backend.lines.clear();
        debug_render.render(
            &mut backend,
            &world.bodies,
            &world.colliders,
            &world.impulse_joints,
            &world.multibody_joints,
            &world.narrow_phase,
            &world.soft_bodies,
        );
    }

    println!("{} lines to draw", backend.lines.len());
    // DOCUSAURUS: DebugRender stop
}
