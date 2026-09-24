use rapier3d::prelude::*;

fn main() {
    // DOCUSAURUS: Creation start
    // A world with a ground.
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(10.0, 0.1, 10.0), None);

    // Builder for a rope of 20 particles between two points.
    let _ = SoftBodyBuilder::rope(Vector::new(0.0, 3.0, 0.0), Vector::new(2.0, 3.0, 0.0), 20);
    // Builder for a cloth: `nu` by `nv` particles, particle `(i, j)` at `origin + i * du + j * dv`.
    let _ = SoftBodyBuilder::cloth(
        Vector::new(-1.0, 2.0, -1.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        20,
        20,
    );
    // Builder for a box of `nx * ny * nz` particles filled with tetrahedral cells.
    let _ = SoftBodyBuilder::cuboid(Vector::new(3.0, 1.0, 0.0), Vector::splat(0.5), 4, 4, 4);
    // Builder for a hollow sphere holding its volume (a balloon).
    let _ = SoftBodyBuilder::sphere(Vector::new(0.0, 3.0, 3.0), 0.8, 2);
    // Builder over raw particle positions; the elements are added by the setters.
    let n = 20;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-1.0, 2.0, -1.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    // Particles held in place.
    .pinned_particles([0, (n - 1) as u32, (n * (n - 1)) as u32, (n * n - 1) as u32])
    // A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
    .softness(SpringCoefficients::new(30.0, 1.0))
    // The mass of each particle.
    // Default: 1.0
    .particle_mass(0.05)
    // The thickness of the particles, for collisions.
    // Default: 0.01
    .particle_radius(0.02)
    // The template of the body's colliders: its shape is replaced by the deformable surface.
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.8))
    // Whether the surface collides with itself.
    // Default: false
    .self_contacts(true)
    // Whether the body may fall asleep.
    // Default: true
    .can_sleep(true);
    // Insert the soft body: this creates its hidden root rigid body and its colliders.
    let cloth_handle = world.insert_soft_body(cloth);
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: Volumetric start
    // Fill a closed, outward-oriented triangle mesh with tetrahedral cells of about 0.2 in size.
    let (vertices, indices) = Cuboid::new(Vector::new(0.5, 0.25, 0.25)).to_trimesh();
    let block = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2)
        .expect("the mesh must be closed and enclose some volume")
        .translated(Vector::new(-3.0, 1.0, 0.0));
    let _block_handle = world.insert_soft_body(block);
    // DOCUSAURUS: Volumetric stop

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    let points: Vec<Vector> = (0..27)
        .map(|i| Vector::new((i % 3) as f32, (i / 3 % 3) as f32 + 4.0, (i / 9) as f32) * 0.3)
        .collect();
    let blob = SoftBodyBuilder::new(points)
        .shape_matching(true)
        .material(SoftBodyMaterial {
            // How fast the particles are pulled back toward their rest shape.
            shape_matching_softness: SpringCoefficients::new(5.0, 1.0),
            ..Default::default()
        })
        .particle_radius(0.1);
    let _blob_handle = world.insert_soft_body(blob);
    // DOCUSAURUS: ShapeMatching stop

    // DOCUSAURUS: Sets start
    // The sets can also be used directly, without the `PhysicsWorld` façade.
    let mut soft_body_set = SoftBodySet::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let rope = SoftBodyBuilder::rope(Vector::new(0.0, 3.0, 0.0), Vector::new(2.0, 3.0, 0.0), 20);
    let rope_handle = soft_body_set.insert(rope, &mut rigid_body_set, &mut collider_set);
    let soft_body = &soft_body_set[rope_handle];
    assert_eq!(soft_body.num_particles(), 20);
    // DOCUSAURUS: Sets stop

    // DOCUSAURUS: Oriented start
    // A shell: a closed surface that is not oriented, so its inner side holds the bodies put
    // inside it (a bowl, a box, a container). A closed surface is oriented by default.
    let bowl = SoftBodyBuilder::sphere(Vector::new(-3.0, 2.0, 0.0), 0.8, 2)
        .oriented(false)
        .softness(SpringCoefficients::new(60.0, 1.0));
    let _bowl_handle = world.insert_soft_body(bowl);
    // DOCUSAURUS: Oriented stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly cube with corotational linear elasticity.
    let jelly = SoftBodyBuilder::cuboid(Vector::new(3.0, 1.0, 0.0), Vector::splat(0.5), 4, 4, 4)
        // The constitutive model of the cells: `Volume` (per-cell volume constraints,
        // the shape is held by the edges), `Corotational` or `NeoHookean`.
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            // Stiffness of the elastic cells.
            young_modulus: 2.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            // Plasticity: the rest shape flows past 5% strain, at 20 per second.
            plastic_yield: 0.05,
            plastic_creep: 20.0,
            // Tearing: an element past 40% strain tears.
            tear_strain: Some(0.4),
            ..Default::default()
        })
        .particle_mass(0.2);
    let jelly_handle = world.insert_soft_body(jelly);

    // A material shared by the edges, bending constraints and volume constraints.
    let material = SoftBodyMaterial {
        // Softness of the bending constraints, on top of a uniform 30 Hz softness.
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    };
    world.soft_bodies[cloth_handle].set_material(material);
    // DOCUSAURUS: Material stop

    // DOCUSAURUS: Fem start
    // A stiff beam simulated by the FEM solver (requires the `fem` cargo feature): its stiffness
    // doesn't depend on the number of solver iterations.
    let beam = SoftBodyBuilder::cuboid(Vector::new(0.0, 2.0, -3.0), Vector::new(1.0, 0.1, 0.1), 11, 3, 3)
        .solver(SoftBodySolver::Fem)
        .cell_model(SoftBodyCellModel::NeoHookean)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            poisson_ratio: 0.3,
            ..Default::default()
        })
        // The particles of the face at `x = -1` are the first 3 × 3 ones.
        .pinned_particles(0..9);
    let _beam_handle = world.insert_soft_body(beam);

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    let fem = &mut world.integration_parameters.soft_bodies.fem;
    fem.linear_tolerance = 1.0e-5;
    fem.max_linear_iterations = 20;
    // DOCUSAURUS: Fem stop

    // DOCUSAURUS: Particles start
    let soft_body = &mut world.soft_bodies[cloth_handle];
    // Read the particles.
    let position = soft_body.particle_position(0);
    let velocity = soft_body.particle_velocity(0);
    let positions: Vec<Vector> = soft_body.particle_positions().collect();
    assert_eq!(positions.len(), soft_body.num_particles());
    // Move a particle.
    soft_body.set_particle_position(1, position + Vector::new(0.0, 0.1, 0.0));
    soft_body.set_particle_velocity(1, velocity);
    // Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
    soft_body.set_particle_pinned(2, true);
    soft_body.set_particle_kinematic_target(2, Vector::new(-1.0, 2.5, -0.8));
    // The elements: edges, cells and the boundary triangles.
    let num_edges = soft_body.edges().len();
    let num_cells = soft_body.cells().len();
    let boundary: &[[u32; 3]] = soft_body.boundary();
    assert!(num_edges > 0 && num_cells == 0 && !boundary.is_empty());
    // DOCUSAURUS: Particles stop

    // DOCUSAURUS: Forces start
    let soft_body = &mut world.soft_bodies[cloth_handle];
    // The `true` argument makes sure the soft body is awake.
    soft_body.reset_forces(true); // Reset the forces to zero.
    soft_body.add_force(Vector::new(0.0, 1.0, 0.0), true); // Spread over the particles by mass.
    soft_body.add_particle_force(3, Vector::new(0.0, 1.0, 0.0), true);
    soft_body.apply_impulse(Vector::new(0.0, 0.1, 0.0), true);
    soft_body.apply_particle_impulse(3, Vector::new(0.0, 0.1, 0.0), true);
    // An impulse on the particles within 0.5 of a point, scaled down with the distance.
    soft_body.apply_impulse_at_point(
        Vector::new(0.0, 0.1, 0.0),
        Vector::new(0.0, 2.0, 0.0),
        0.5,
        true,
    );
    // A blast pushing the particles away from a center.
    soft_body.apply_radial_impulse(Vector::new(0.0, 2.0, 0.0), 0.1, 1.0, true);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid ball, at the particle's position.
    let rope = SoftBodyBuilder::rope(Vector::new(-0.5, 5.0, 3.0), Vector::new(2.5, 5.0, 3.0), 30)
        .pinned_particles([0])
        .softness(SpringCoefficients::new(40.0, 1.0));
    let rope_handle = world.insert_soft_body(rope);
    let last_position = world.soft_bodies[rope_handle].particle_position(29);
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(last_position - Vector::new(0.0, 0.3, 0.0)),
        ColliderBuilder::ball(0.25).density(2.0),
    );
    world.soft_bodies[rope_handle].attach_particle(29, ball, &world.bodies);
    // DOCUSAURUS: Attachments stop

    // DOCUSAURUS: RootBody start
    // The rigid body the engine created for the whole soft body, read back after its insertion.
    let root: RigidBodyHandle = world.soft_bodies[jelly_handle].root_body();
    assert!(world.bodies[root].is_soft_frame());

    // A rigid collider attached to it follows the frame of the whole body: here a sensor
    // detecting what comes close to the jelly.
    let _sensor = world.insert_collider(ColliderBuilder::ball(1.0).sensor(true), Some(root));

    // A joint attached to it acts on the soft body as a whole: this one hangs the jelly under a
    // fixed anchor by a spring.
    let anchor =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(3.0, 4.0, 0.0)));
    world.insert_impulse_joint(anchor, root, SpringJointBuilder::new(2.5, 60.0, 2.0));
    // DOCUSAURUS: RootBody stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly: a rigid proxy that joints and
    // colliders can attach to.
    let top: Vec<u32> = {
        let jelly = &world.soft_bodies[jelly_handle];
        (0..jelly.num_particles() as u32)
            .filter(|&i| jelly.particle_position(i as usize).y > 1.3)
            .collect()
    };
    let cluster = world
        .add_soft_body_cluster(jelly_handle, &top)
        .expect("at least one valid particle");
    let proxy: RigidBodyHandle = world.soft_bodies[jelly_handle]
        .cluster_proxy(cluster)
        .unwrap();
    // A rigid plate welded onto the cluster.
    let (plate, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(3.0, 1.9, 0.0)),
        ColliderBuilder::cuboid(0.7, 0.05, 0.7).density(0.4),
    );
    world.insert_impulse_joint(
        plate,
        proxy,
        FixedJointBuilder::new().local_anchor1(Vector::new(0.0, -0.1, 0.0)),
    );
    // A cluster can be pinned, driven or tuned as a whole.
    let jelly = &mut world.soft_bodies[jelly_handle];
    jelly.set_cluster_stiffness_scale(cluster, 2.0);
    jelly.enable_cluster_shape_matching(cluster, true);
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: ClusterControl start
    // Pin every particle of the cluster, then move it along a path: the cluster behaves like a
    // kinematic rigid part dragging the rest of the body.
    let jelly = &mut world.soft_bodies[jelly_handle];
    jelly.set_cluster_pinned(cluster, true);
    jelly.set_cluster_kinematic_target(
        cluster,
        Pose::from_translation(Vector::new(3.0, 2.0, 0.0)),
    );
    // Release it: the cluster is simulated again.
    jelly.set_cluster_pinned(cluster, false);
    // DOCUSAURUS: ClusterControl stop

    // DOCUSAURUS: DeformableColliders start
    // A deformable triangle mesh bound to the jelly: each vertex is embedded in the cell
    // holding it (`skinned`), or follows one particle (`direct`). The mesh is given in the
    // frame of the proxy it is attached to.
    let root = world.soft_bodies[jelly_handle].root_body();
    let root_pose = *world.bodies[root].position();
    let center = world.soft_bodies[jelly_handle].center_of_mass();
    let r = 1.0;
    let vertices: Vec<Vector> = [
        Vector::new(r, 0.0, 0.0),
        Vector::new(-r, 0.0, 0.0),
        Vector::new(0.0, r, 0.0),
        Vector::new(0.0, -r, 0.0),
        Vector::new(0.0, 0.0, r),
        Vector::new(0.0, 0.0, -r),
    ]
    .iter()
    .map(|v| root_pose.inverse() * (center + *v))
    .collect();
    let indices = vec![
        [0, 2, 4],
        [2, 1, 4],
        [1, 3, 4],
        [3, 0, 4],
        [2, 0, 5],
        [1, 2, 5],
        [3, 1, 5],
        [0, 3, 5],
    ];
    let skin = ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)
        .unwrap()
        .sensor(true);
    let skin_handle = world
        .insert_deformable(skin, SoftMeshBinding::skinned(), root)
        .expect("a deformable mesh bound to a cluster proxy");
    // The mesh follows the particles: read its current vertices back.
    let jelly = &world.soft_bodies[jelly_handle];
    let mesh = jelly.mesh_of(skin_handle).unwrap();
    let skin_vertices: Vec<Vector> = mesh.vertex_positions(jelly).collect();
    assert_eq!(skin_vertices.len(), 6);
    // DOCUSAURUS: DeformableColliders stop

    // DOCUSAURUS: Skinning start
    // A detailed mesh held by a coarse cage of cells: only the cells are simulated, and the mesh
    // (the skin) follows their deformation.
    let (vertices, indices) = Ball::new(0.5).to_trimesh(24, 24);
    let skinned = SoftBodyBuilder::volumetric_skinned(&vertices, &indices, 0.25)
        .expect("the mesh must be closed and enclose some volume")
        // Collide through the skin instead of the boundary of the cage.
        .skin_collision(true)
        .translated(Vector::new(0.0, 4.0, 3.0));
    let skinned_handle = world.insert_soft_body(skinned);
    // The skin is the body's collision mesh: read its vertices back to render it.
    let body = &world.soft_bodies[skinned_handle];
    let skin = body.collision_mesh().expect("the skin collides");
    let skin_vertices: Vec<Vector> = skin.vertex_positions(body).collect();
    assert_eq!(skin_vertices.len(), vertices.len());
    // DOCUSAURUS: Skinning stop

    // DOCUSAURUS: Plasticity start
    // The jelly has elastic (corotational) cells: the plasticity of `Volume` cells has no effect.
    let material = world.soft_bodies[jelly_handle].material_mut();
    // Cells: the rest shape flows toward the current one past 5% strain, at a rate of 20 per
    // second, up to a total permanent deformation of 50%.
    material.plastic_yield = 0.05;
    material.plastic_creep = 20.0;
    material.plastic_max = 0.5;
    // Edges: the rest length flows past 10% strain, up to half the initial length, but only
    // when squeezed (a dent stays, a stretch springs back).
    material.edge_plastic_yield = 0.1;
    material.edge_plastic_creep = 10.0;
    material.edge_plastic_max = 0.5;
    material.edge_plastic_flow = SoftEdgePlasticFlow::Compression;
    // Every permanent deformation can be undone at once.
    world.soft_bodies[jelly_handle].reset_plasticity();
    // DOCUSAURUS: Plasticity stop

    // DOCUSAURUS: TearingMaterial start
    let material = world.soft_bodies[cloth_handle].material_mut();
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    material.tear_strain = Some(0.4);
    material.tear_force = Some(50.0);
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    material.tear_smoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    material.interior_strength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    material.min_piece = Some(10);
    // DOCUSAURUS: TearingMaterial stop

    // DOCUSAURUS: Tearing start
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    world.soft_bodies[cloth_handle].tear_edge(10); // Applied at the end of the next step.
                                                   // Tear at once along edges and through cells; pieces the tear disconnects become soft
                                                   // bodies of their own.
    let event = world.tear_soft_body(cloth_handle, &[11, 12], &[]);
    if let Some(event) = event {
        println!("{} edges torn", event.torn_edges.len());
    }
    // Cut along a blade (a triangle in 3D), without removing material.
    let blade = [
        Vector::new(-0.1, -10.0, -10.0),
        Vector::new(-0.1, 10.0, 0.0),
        Vector::new(-0.1, -10.0, 10.0),
    ];
    if let Some(event) = world.cut_soft_body(cloth_handle, &blade) {
        for piece in &event.pieces {
            println!(
                "piece {:?} has {} particles",
                piece.soft_body,
                piece.particles.len()
            );
        }
        // Where a particle of the torn body went.
        if let Some((body, index)) = event.particle_destination(n as u32 * n as u32 - 1) {
            println!(
                "particle {} is now particle {} of {:?}",
                n * n - 1,
                index,
                body
            );
        }
    }
    // DOCUSAURUS: Tearing stop

    // DOCUSAURUS: Events start
    // Tears applied during a step are reported through the event handler.
    let (collision_send, _collision_recv) = std::sync::mpsc::channel();
    let (contact_force_send, _contact_force_recv) = std::sync::mpsc::channel();
    let (soft_body_tear_send, soft_body_tear_recv) = std::sync::mpsc::channel();
    let event_handler =
        ChannelEventCollector::new(collision_send, contact_force_send, soft_body_tear_send);
    world.step_with_events(&(), &event_handler);
    while let Ok(tear_event) = soft_body_tear_recv.try_recv() {
        println!("Soft body {:?} tore", tear_event.soft_body);
    }
    // DOCUSAURUS: Events stop

    // DOCUSAURUS: Settings start
    // Settings shared by every soft body of the world.
    let settings = &mut world.integration_parameters.soft_bodies;
    // Strain beyond which a constraint is re-solved after the contacts of every substep.
    // Default: 0.75
    settings.resweep_strain = 0.75;
    // Extra substeps a soft body requests while it is hit fast; 0 disables them.
    // Default: 4
    settings.max_extra_substeps = 4;
    // Stiffening of the soft-body contacts relative to the rigid ones.
    // Default: 4.0
    settings.contact_stiffening = 4.0;
    // The tangle detection and recovery stack can be switched off mechanism by mechanism.
    settings.recovery.crossing_repulsion = true;
    // DOCUSAURUS: Settings stop

    // DOCUSAURUS: Removal start
    // Removing a soft body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    world.remove_soft_body(rope_handle);
    // A cluster can be removed on its own.
    world.remove_soft_body_cluster(jelly_handle, cluster);
    // DOCUSAURUS: Removal stop

    for _ in 0..10 {
        world.step();
    }
}
