//! 3D intersection-volume constraints (see the 2D `overlap_constraints`): two closed cell-less balloons
//! inserted overlapping de-overlap, each retracting its own intruding patch.

use rapier3d::prelude::*;

fn segment_crosses_triangle(p: Vector, q: Vector, tri: &[Vector; 3]) -> bool {
    let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    let dp = (p - tri[0]).dot(n);
    let dq = (q - tri[0]).dot(n);
    if dp * dq >= 0.0 {
        return false;
    }
    let pq = q - p;
    let d = [
        pq.dot((tri[0] - p).cross(tri[1] - p)),
        pq.dot((tri[1] - p).cross(tri[2] - p)),
        pq.dot((tri[2] - p).cross(tri[0] - p)),
    ];
    d.iter().all(|x| *x >= 0.0) || d.iter().all(|x| *x <= 0.0)
}

fn triangles_cross(a: &[Vector; 3], b: &[Vector; 3]) -> bool {
    (0..3).any(|k| segment_crosses_triangle(a[k], a[(k + 1) % 3], b))
        || (0..3).any(|k| segment_crosses_triangle(b[k], b[(k + 1) % 3], a))
}

fn inside(point: Vector, mesh: &SoftCollisionMesh, body: &SoftBody) -> bool {
    let q = point + Vector::new(0.5341, 0.6432, 0.5487) * 100.0;
    let mut crossings = 0;
    for e in mesh.indices() {
        let tri: [Vector; 3] = core::array::from_fn(|k| mesh.vertex(body, e[k] as usize));
        if segment_crosses_triangle(point, q, &tri) {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

fn overlap(world: &PhysicsWorld, a: SoftBodyHandle, b: SoftBodyHandle) -> (usize, usize, usize) {
    let (sa, sb) = (&world.soft_bodies[a], &world.soft_bodies[b]);
    let (ma, mb) = (sa.meshes().next().unwrap(), sb.meshes().next().unwrap());
    let tris = |sb: &SoftBody, mesh: &SoftCollisionMesh| -> Vec<[Vector; 3]> {
        mesh.indices()
            .iter()
            .map(|e| core::array::from_fn(|k| mesh.vertex(sb, e[k] as usize)))
            .collect()
    };
    let (ta, tb) = (tris(sa, ma), tris(sb, mb));
    let mut crossings = 0;
    for x in &ta {
        for y in &tb {
            if triangles_cross(x, y) {
                crossings += 1;
            }
        }
    }
    let a_in_b = (0..ma.vertex_count())
        .filter(|&v| inside(ma.vertex(sa, v), mb, sb))
        .count();
    let b_in_a = (0..mb.vertex_count())
        .filter(|&v| inside(mb.vertex(sb, v), ma, sa))
        .count();
    (crossings, a_in_b, b_in_a)
}

fn overlap_only(world: &mut PhysicsWorld) {
    if std::env::var("SOFT_DEPEN_PRESET").as_deref() == Ok("defaults") {
        return;
    }
    // The reference settings: prevention without edge speculation, detection and the
    // stand-down, the constraints on.
    let r = &mut world.integration_parameters.soft_bodies.recovery;
    r.overlap_constraints = true;
    r.edge_speculation = false;
    // Diagnostic: the closed-closed edge constraints off.
    if std::env::var("SOFT_DEPEN_NO_EDGE").is_ok() {
        r.edge_speculation = false;
    }
}

fn balloon(center: Vector, radius: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::sphere(center, radius, 1)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
}

#[test]
fn overlapping_balloons_deoverlap() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(6.0, 0.5, 6.0),
    );
    let a = world.insert_soft_body(balloon(Vector::new(-0.35, 0.55, 0.0), 0.5));
    let b = world.insert_soft_body(balloon(Vector::new(0.35, 0.55, 0.0), 0.5));
    overlap_only(&mut world);
    for step in 0..=900 {
        if step % 100 == 0 {
            let (c, ab, ba) = overlap(&world, a, b);
            println!("step {step:4}: crossings={c} a_in_b={ab} b_in_a={ba}");
        }
        world.step();
    }
    let (c, ab, ba) = overlap(&world, a, b);
    assert_eq!((c, ab, ba), (0, 0, 0), "the balloons still overlap");
}
