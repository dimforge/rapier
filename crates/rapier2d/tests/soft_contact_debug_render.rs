//! The debug-renderer draws a soft body's contacts at their witness points (not at the touched
//! element's centroid), self contacts included.
#![cfg(feature = "debug-render")]

use rapier2d::pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
    DebugRenderStyle,
};
use rapier2d::prelude::*;

#[derive(Default)]
struct LineCollector {
    lines: Vec<(Vector, Vector, DebugColor)>,
}

impl DebugRenderBackend for LineCollector {
    fn draw_line(&mut self, _: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        self.lines.push((a, b, color));
    }
}

/// A soft column, slender and soft enough to buckle and fold onto itself under gravity. Its
/// cells are far wider than the particles are thick, so a segment drawn to an element's centroid
/// is much longer than the contact it stands for.
const SPACING: Real = 0.6;
const RADIUS: Real = 0.05;

fn buckling_column() -> SoftBodyBuilder {
    let (nx, ny) = (3u32, 26u32);
    let mut positions = vec![];
    for i in 0..nx {
        for j in 0..ny {
            positions.push(Vector::new(
                i as Real * SPACING - SPACING,
                1.0 + j as Real * SPACING,
            ));
        }
    }
    let mut cells = vec![];
    for i in 0..nx - 1 {
        for j in 0..ny - 1 {
            let (a, b, c, d) = (
                i * ny + j,
                i * ny + j + 1,
                (i + 1) * ny + j,
                (i + 1) * ny + j + 1,
            );
            cells.push([a, b, c]);
            cells.push([b, d, c]);
        }
    }
    SoftBodyBuilder::new(positions)
        .cells(cells)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.4,
            ..Default::default()
        })
        .particle_radius(RADIUS)
        .self_contacts(true)
}

#[test]
fn soft_contacts_are_drawn_at_their_witness_points() {
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(20.0, 1.0), None);
    let handle = world.insert_soft_body(buckling_column());

    // Step until the column has folded onto itself.
    let mut folded = false;
    for _ in 0..600 {
        world.step();
        let sb = &world.soft_bodies[handle];
        if sb.vertex_contact_segments(&world.soft_bodies).count() > 0 {
            folded = true;
            break;
        }
    }
    assert!(folded, "the column never self-collided; tune the scene");

    // Every contact here is a self contact: the column is the only soft body.
    let sb = &world.soft_bodies[handle];
    let contacts: Vec<_> = sb
        .edge_contact_segments(&world.soft_bodies)
        .chain(sb.vertex_contact_segments(&world.soft_bodies))
        .collect();
    assert!(!contacts.is_empty());

    let mut pipeline =
        DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::SOFT_BODIES);
    let mut backend = LineCollector::default();
    pipeline.render_soft_bodies(&mut backend, &world.soft_bodies);

    let style = DebugRenderStyle::default();
    let depth: Vec<_> = backend
        .lines
        .iter()
        .filter(|(.., color)| *color == style.contact_depth_color)
        .collect();
    let normals: Vec<_> = backend
        .lines
        .iter()
        .filter(|(.., color)| *color == style.contact_normal_color)
        .collect();

    // The self contacts reach the renderer, with a normal each so they are visible against the
    // cage they lie on.
    assert_eq!(depth.len(), contacts.len());
    assert!(!normals.is_empty());
    for (a, b, _) in &normals {
        assert!(((*b - *a).length() - style.contact_normal_length).abs() < 1.0e-5);
    }

    // Witness pairs are at most a contact apart (a centroid could be half an element away, which
    // this scene keeps well above), and both witness points lie on the body's own surface.
    for (a, b, _) in &depth {
        for p in [a, b] {
            let on_surface = sb.meshes().any(|mesh| {
                (0..mesh.vertex_count()).any(|i| (mesh.vertex(sb, i) - *p).length() < SPACING)
            });
            assert!(on_surface, "contact witness point {p:?} is off the surface");
        }
    }
}

/// With `SOFT_BODY_STRESS`, an element's color runs from the slack color to the loaded color
/// with its tear load: an edge bearing half its tear force is drawn halfway, an unloaded edge
/// in the slack color; without the flag every element keeps the element color.
#[test]
fn soft_elements_are_colored_by_their_load() {
    let mut world = PhysicsWorld::new();
    // Edge 0 hangs a unit mass (9.81 N, half the tear force); edge 1 joins two pinned
    // particles and bears nothing.
    let builder = SoftBodyBuilder::new(vec![
        Vector::new(0.0, 2.0),
        Vector::new(1.0, 2.0),
        Vector::new(0.0, 1.0),
        Vector::new(1.0, 1.0),
    ])
    .edges(vec![[0, 2], [1, 3]])
    .pinned_particles([0, 1, 3])
    .particle_mass(1.0)
    .softness(SpringCoefficients::new(120.0, 1.0))
    .tear_force(2.0 * 9.81)
    .no_surface_collider()
    .can_sleep(false);
    let handle = world.insert_soft_body(builder);
    for _ in 0..60 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    let loaded = sb.edges()[0].stress();
    assert!((loaded - 0.5).abs() < 0.05, "edge 0 bears {loaded} of its threshold");
    assert!(sb.edges()[1].stress().abs() < 1.0e-6);

    let style = DebugRenderStyle::default();
    let render = |mode: DebugRenderMode| {
        let mut pipeline = DebugRenderPipeline::new(style.clone(), mode);
        let mut backend = LineCollector::default();
        pipeline.render_soft_bodies(&mut backend, &world.soft_bodies);
        backend.lines
    };
    // The lines come out sorted by particle pair: edge 0 first.
    let plain = render(DebugRenderMode::SOFT_BODIES);
    assert_eq!(plain.len(), 2);
    assert!(plain.iter().all(|(.., c)| *c == style.soft_body_element_color));

    let by_load = render(DebugRenderMode::SOFT_BODIES | DebugRenderMode::SOFT_BODY_STRESS);
    assert_eq!(by_load.len(), 2);
    let (slack, full) = (style.soft_body_slack_color, style.soft_body_loaded_color);
    let expected: DebugColor = core::array::from_fn(|k| slack[k] + (full[k] - slack[k]) * loaded);
    for k in 0..4 {
        assert!((by_load[0].2[k] - expected[k]).abs() < 1.0e-4, "loaded edge color {:?}", by_load[0].2);
    }
    assert_eq!(by_load[1].2, slack);
}
