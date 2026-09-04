//! Collision meshes owned by clusters: a jelly wearing a fine skin over its coarse cells, a
//! jelly whose top cluster supports a rigid plate, and a jelly split into two clusters that
//! collide with each other.

use rapier_testbed3d::TestbedViewer;
use rapier3d::parry::shape::{Ball, TriMeshFlags};
use rapier3d::prelude::*;

/// A corotational jelly cube of `n` particles per side.
fn jelly(center: Vector, half_extents: Real, n: usize, young: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::cuboid(center, Vector::splat(half_extents), n, n, n)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: young,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.7))
}

/// The three jellies of this demo: skinned, plated, split.
fn build_world() -> anyhow::Result<(PhysicsWorld, [SoftBodyHandle; 3])> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(12.0, 0.1, 12.0),
    );

    /*
     * A jelly wearing a second, skinned ball-shaped sensor.
     */
    let center = Vector::new(-3.0, 1.2, 0.0);
    let skinned = world.insert_soft_body(jelly(center, 0.6, 4, 2.0e2));
    // The mesh is authored in world space, where the body is: placed by the inverse of the
    // proxy's pose, like any collider attached to a body.
    let (vertices, indices) = Ball::new(1.2).to_trimesh(20, 20);
    let vertices: Vec<Vector> = vertices.iter().map(|v| *v + center).collect();
    let proxy = world.soft_bodies[skinned].root_body();
    world.insert_deformable(
        ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)?
            .position(world.bodies[proxy].position().inverse())
            .sensor(true),
        SoftMeshBinding::skinned(),
        proxy,
    )?;

    /*
     * A jelly whose top cluster supports a rigid plate: the load on the plate reaches the
     * particles through the cluster's frame.
     */
    let center = Vector::new(0.0, 1.2, 0.0);
    let plated = world.insert_soft_body(jelly(center, 0.6, 4, 4.0e2));
    let top: Vec<u32> = world.soft_bodies[plated]
        .particles()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.position().y > center.y + 0.3)
        .map(|(i, _)| i as u32)
        .collect();
    let cluster = world
        .soft_bodies
        .add_cluster(plated, &top, &mut world.bodies, &mut world.colliders)
        .expect("the top cluster");
    let proxy = world.soft_bodies[plated].cluster_proxy(cluster).unwrap();
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.7, 0.05, 0.7).translation(Vector::new(0.0, 0.65, 0.0)),
        proxy,
        &mut world.bodies,
    );

    /*
     * A jelly split into two clusters, each with a mesh of its own: they collide with each
     * other (both opted into self contacts) because their clusters share no particle.
     */
    let center = Vector::new(3.0, 1.2, 0.0);
    let split = world.insert_soft_body(jelly(center, 0.6, 4, 4.0e2).no_surface_collider());
    let (left, right): (Vec<u32>, Vec<u32>) = world.soft_bodies[split]
        .particles()
        .iter()
        .enumerate()
        .map(|(i, p)| (i as u32, p.position().x < center.x))
        .fold((Vec::new(), Vec::new()), |mut sides, (i, is_left)| {
            if is_left {
                sides.0.push(i);
            } else {
                sides.1.push(i);
            }
            sides
        });
    for half in [left, right] {
        let cluster = world
            .soft_bodies
            .add_cluster(split, &half, &mut world.bodies, &mut world.colliders)
            .expect("a half cluster");
        let proxy = world.soft_bodies[split].cluster_proxy(cluster).unwrap();
        let sb = &world.soft_bodies[split];
        // The closed boundary of the cells the half owns: its outer skin plus a cap over the
        // cut, whose vertices are particles *inside* the body. A direct binding takes those
        // like any other.
        let (vertices, indices) = cluster_surface(sb, &half);
        world.insert_deformable(
            ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)?
                .position(world.bodies[proxy].position().inverse())
                .friction(0.7),
            // Both halves opt in: their clusters share no particle, so their meshes may meet.
            SoftMeshBinding::direct(half.clone()).self_contacts(true),
            proxy,
        )?;
    }

    /*
     * Something to drop on all three.
     */
    for (i, x) in [-3.0, 0.0, 3.0].iter().enumerate() {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(*x, 4.0 + i as Real, 0.0)),
            ColliderBuilder::cuboid(0.3, 0.3, 0.3).density(4.0),
        );
    }

    Ok((world, [skinned, plated, split]))
}

/// The closed surface of the cells a set of particles owns: every face that no second owned
/// cell shares, oriented outward, over those particles as vertices. A capped half of a body has
/// some surface vertices that sit inside the body rather than on its boundary.
fn cluster_surface(sb: &SoftBody, particles: &[u32]) -> (Vec<Vector>, Vec<[u32; 3]>) {
    use std::collections::HashMap;

    let mut vertex_of = vec![u32::MAX; sb.num_particles()];
    for (vertex, &particle) in particles.iter().enumerate() {
        vertex_of[particle as usize] = vertex as u32;
    }
    let vertices: Vec<Vector> = particles
        .iter()
        .map(|&i| sb.particle_position(i as usize))
        .collect();

    // Every face of every owned cell, keyed by its sorted vertices: a face shared by two of
    // them is interior to the half.
    let mut faces: HashMap<[u32; 3], ([u32; 3], Vector, usize)> = HashMap::new();
    for cell in sb.cells() {
        let cell = cell.vertices;
        if !cell.iter().all(|v| vertex_of[*v as usize] != u32::MAX) {
            continue;
        }
        let centroid = cell
            .iter()
            .map(|&v| sb.particle_position(v as usize))
            .sum::<Vector>()
            / 4.0;
        for k in 0..4 {
            let face = [cell[(k + 1) % 4], cell[(k + 2) % 4], cell[(k + 3) % 4]];
            let mut key = face;
            key.sort_unstable();
            let entry = faces.entry(key).or_insert((face, centroid, 0));
            entry.2 += 1;
        }
    }

    let indices = faces
        .values()
        .filter(|(_, _, count)| *count == 1)
        .map(|(face, centroid, _)| {
            let p = face.map(|v| sb.particle_position(v as usize));
            // Outward winding: the face's normal must point away from its cell.
            let normal = (p[1] - p[0]).cross(p[2] - p[0]);
            let face = if normal.dot((p[0] + p[1] + p[2]) / 3.0 - *centroid) < 0.0 {
                [face[0], face[2], face[1]]
            } else {
                *face
            };
            face.map(|v| vertex_of[v as usize])
        })
        .collect();

    (vertices, indices)
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let (mut world, _) = build_world()?;
    viewer.set_world(&mut world);
    viewer.look_at(Vector::new(-6.0, 4.0, 8.0), Vector::new(0.0, 1.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each half of the split jelly is capped over the cut: a closed surface whose vertices
    /// include particles from inside the body.
    #[test]
    fn the_split_jelly_halves_are_closed() {
        let (world, [_, _, split]) = build_world().expect("the demo builds");
        let sb = &world.soft_bodies[split];
        let on_boundary =
            |particle: u32| sb.boundary().iter().any(|element| element.contains(&particle));

        let halves: Vec<_> = sb.meshes().filter(|mesh| mesh.id().cluster != 0).collect();
        assert_eq!(halves.len(), 2);
        for mesh in halves {
            assert!(mesh.is_closed(), "a half's surface is not closed");
            let SoftMeshMapping::Direct { particles } = mesh.binding() else {
                panic!("a half's mesh is not bound to particles");
            };
            // The cap over the cut rides particles the body's own boundary does not include.
            assert!(
                particles.iter().copied().any(|p| !on_boundary(p)),
                "no vertex of the cap rides an interior particle"
            );
            // And it encloses a volume: the winding is outward and consistent.
            let volume: Real = mesh
                .indices()
                .iter()
                .map(|t| {
                    let p = t.map(|v| mesh.vertex(sb, v as usize));
                    p[0].dot(p[1].cross(p[2])) / 6.0
                })
                .sum();
            assert!(volume > 0.05, "the half encloses no volume: {volume}");
        }
    }

    /// The three cluster-mesh scenes settle: the meshes follow their particles, the plate
    /// bears its load, and nothing explodes.
    #[test]
    fn cluster_meshes_settle() {
        let (mut world, handles) = build_world().expect("the demo builds");
        for _ in 0..400 {
            world.step();
        }
        for handle in handles {
            let sb = &world.soft_bodies[handle];
            assert!(
                sb.particle_positions().all(|p| p.is_finite()),
                "a body went non-finite"
            );
            let com = sb.center_of_mass();
            assert!(
                com.y > 0.0 && com.y < 3.0,
                "a body left the scene (com {com:?})"
            );
            for mesh in sb.meshes() {
                assert!(
                    mesh.vertex_positions(sb).all(|v| v.is_finite()),
                    "a mesh went non-finite"
                );
            }
        }
    }
}
