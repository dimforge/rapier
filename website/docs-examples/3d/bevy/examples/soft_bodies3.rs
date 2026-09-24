use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_rapier3d::parry::shape::{Ball, Cuboid as CuboidShape};
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, (setup_graphics, setup_physics, configure_fem))
        .add_systems(Startup, configure_soft_bodies)
        .add_systems(
            Update,
            (
                read_soft_bodies,
                read_deformable_colliders,
                read_tear_events,
                control_particles.run_if(input_just_pressed(KeyCode::KeyP)),
                apply_forces.run_if(input_just_pressed(KeyCode::KeyF)),
                drive_cluster.run_if(input_just_pressed(KeyCode::KeyK)),
                release_cluster.run_if(input_just_pressed(KeyCode::KeyL)),
                configure_plasticity.run_if(input_just_pressed(KeyCode::KeyM)),
                configure_tearing.run_if(input_just_pressed(KeyCode::KeyN)),
                tear_cloth.run_if(input_just_pressed(KeyCode::KeyT)),
                remove_soft_bodies.run_if(input_just_pressed(KeyCode::KeyR)),
            ),
        )
        .run();
}

/// Marks the cloth entity.
#[derive(Component)]
struct Cloth;

/// Marks the jelly entity.
#[derive(Component)]
struct Jelly;

/// Marks the rope entity.
#[derive(Component)]
struct Rope;

/// Marks the cluster carrying the plate.
#[derive(Component)]
struct PlateCluster;

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 5.0, 10.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    // DOCUSAURUS: Creation start
    // A ground.
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(10.0, 0.1, 10.0),
    ));

    // A rope of 20 particles between two points (in the local frame of the entity).
    let _ = SoftBody::rope(Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0), 20);
    // A cloth: `nu` by `nv` particles, particle `(i, j)` at `origin + i * du + j * dv`.
    let _ = SoftBody::cloth(Vec3::ZERO, Vec3::X * 0.1, Vec3::Z * 0.1, 20, 20);
    // A box of `nx * ny * nz` particles filled with tetrahedral cells, centered on the entity.
    let _ = SoftBody::cuboid(Vec3::splat(0.5), 4, 4, 4);
    // A hollow sphere holding its volume (a balloon), centered on the entity.
    let _ = SoftBody::sphere(0.8, 2);
    // Any constructor of the Rapier builder can be used too.
    let _ = SoftBody::new(SoftBodyBuilder::cloth_tube(
        Vec3::ZERO,
        Vec3::Y,
        0.3,
        0.3,
        12,
        10,
    ));

    let n = 20;
    let corners = [0, (n - 1) as u32, (n * (n - 1)) as u32, (n * n - 1) as u32];
    let cloth = commands
        .spawn((
            Cloth,
            // The particles are placed by the transform of the entity when the soft-body is
            // created. Then, the entity follows the center of mass of the particles.
            Transform::from_xyz(-1.0, 2.0, -1.0),
            SoftBody::cloth(Vec3::ZERO, Vec3::X * 0.1, Vec3::Z * 0.1, n, n).map(|builder| {
                // Particles held in place.
                builder
                    .pinned_particles(corners)
                    // A uniform softness (frequency in Hz, damping ratio) for every constraint.
                    .softness(SpringCoefficients::new(30.0, 1.0))
                    // The mass of each particle.
                    // Default: 1.0
                    .particle_mass(0.05)
                    // The thickness of the particles, for collisions.
                    // Default: 0.01
                    .particle_radius(0.02)
                    // Whether the surface collides with itself.
                    // Default: false
                    .self_contacts(true)
                    // Whether the body may fall asleep.
                    // Default: true
                    .can_sleep(true)
            }),
            // The collider components of the entity configure the colliders of its surface.
            Friction::coefficient(0.8),
            // Render the soft-body with a mesh kept in sync with its particles.
            SoftBodyMeshSync::default(),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.8, 0.2, 0.2),
                double_sided: true,
                cull_mode: None,
                ..default()
            })),
        ))
        .id();
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: Volumetric start
    // Fill a closed, outward-oriented triangle mesh with tetrahedral cells of about 0.2 in size.
    let (vertices, indices) = CuboidShape::new(Vec3::new(0.5, 0.25, 0.25)).to_trimesh();
    let block = SoftBody::volumetric(&vertices, &indices, 0.2)
        .expect("the mesh must be closed and enclose some volume");
    commands.spawn((Transform::from_xyz(-3.0, 1.0, 0.0), block));
    // DOCUSAURUS: Volumetric stop

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    let points: Vec<Vec3> = (0..27)
        .map(|i| Vec3::new((i % 3) as f32, (i / 3 % 3) as f32, (i / 9) as f32) * 0.3)
        .collect();
    commands.spawn((
        Transform::from_xyz(0.0, 4.0, 0.0),
        SoftBody::new(
            SoftBodyBuilder::new(points)
                .shape_matching(true)
                .particle_radius(0.1),
        ),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            // How fast the particles are pulled back toward their rest shape.
            shape_matching_softness: SpringCoefficients::new(5.0, 1.0),
            ..default()
        }),
    ));
    // DOCUSAURUS: ShapeMatching stop

    // DOCUSAURUS: Oriented start
    // A shell: a closed surface that is not oriented, so its inner side holds the bodies put
    // inside it (a bowl, a box, a container). A closed surface is oriented by default.
    commands.spawn((
        Transform::from_xyz(-3.0, 2.0, 0.0),
        SoftBody::sphere(0.8, 2).map(|builder| {
            builder
                .oriented(false)
                .softness(SpringCoefficients::new(60.0, 1.0))
        }),
    ));
    // DOCUSAURUS: Oriented stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly cube with corotational linear elasticity.
    let jelly_body = SoftBody::cuboid(Vec3::splat(0.5), 4, 4, 4).map(|builder| {
        // The constitutive model of the cells: `Volume` (per-cell volume constraints,
        // the shape is held by the edges), `Corotational` or `NeoHookean`.
        builder
            .cell_model(SoftBodyCellModel::Corotational)
            .particle_mass(0.2)
    });
    let jelly = commands
        .spawn((
            Jelly,
            Transform::from_xyz(3.0, 1.0, 0.0),
            jelly_body.clone(),
            // The material of the soft-body: modifying this component updates the soft-body.
            SoftBodyMaterial(RapierSoftBodyMaterial {
                // Stiffness of the elastic cells.
                young_modulus: 2.0e3,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.5,
                // Plasticity: the rest shape flows past 5% strain, at 20 per second.
                plastic_yield: 0.05,
                plastic_creep: 20.0,
                // Tearing: an element past 40% strain tears.
                tear_strain: Some(0.4),
                ..default()
            }),
        ))
        .id();

    // A balloon inflated by volume preservation.
    commands.spawn((
        Transform::from_xyz(0.0, 3.0, 3.0),
        SoftBody::sphere(0.8, 2),
        SoftBodyMaterial::uniform(20.0, 1.0),
        // Target volume multiplier (`> 1` inflates the body).
        SoftBodyVolumeFactor(1.1),
    ));

    // A material shared by the edges, bending constraints and volume constraints.
    commands
        .entity(cloth)
        .insert(SoftBodyMaterial(RapierSoftBodyMaterial {
            // Softness of the bending constraints, on top of a uniform 30 Hz softness.
            bend_softness: SpringCoefficients::new(3.0, 1.0),
            ..RapierSoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
        }));
    // DOCUSAURUS: Material stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid ball, at the particle's position.
    let ball = commands
        .spawn((
            Transform::from_xyz(2.5, 4.7, 3.0),
            RigidBody::Dynamic,
            Collider::ball(0.25),
            ColliderMassProperties::Density(2.0),
        ))
        .id();
    commands.spawn((
        Rope,
        Transform::from_xyz(-0.5, 5.0, 3.0),
        SoftBody::rope(Vec3::ZERO, Vec3::new(3.0, 0.0, 0.0), 30).map(|builder| {
            builder
                .pinned_particles([0])
                .softness(SpringCoefficients::new(40.0, 1.0))
        }),
        SoftBodyAttachments(vec![SoftBodyAttachment {
            particle: 29,
            body: ball,
        }]),
    ));
    // DOCUSAURUS: Attachments stop

    // DOCUSAURUS: RootBody start
    // The soft-body entity stands for its root body. A joint attached to it acts on the soft-body
    // as a whole: this one hangs the jelly under a fixed anchor by a spring.
    let anchor = commands
        .spawn((Transform::from_xyz(3.0, 4.0, 0.0), RigidBody::Fixed))
        .id();
    commands.entity(jelly).insert(ImpulseJoint::new(
        anchor,
        SpringJointBuilder::new(2.5, 60.0, 2.0),
    ));

    // A rigid collider on a child of the soft-body entity is attached to its root body: here a
    // sensor detecting what comes close to the jelly.
    commands
        .entity(jelly)
        .with_child((Transform::default(), Collider::ball(1.0), Sensor));
    // DOCUSAURUS: RootBody stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly (their indices are read from its builder,
    // in the local frame of the jelly entity).
    let top: Vec<u32> = (0..)
        .zip(jelly_body.builder.particle_positions())
        .filter(|(_, p)| p.y > 0.3)
        .map(|(i, _)| i)
        .collect();
    // A rigid plate welded onto the cluster.
    let plate = commands
        .spawn((
            Transform::from_xyz(3.0, 1.9, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(0.7, 0.05, 0.7),
            ColliderMassProperties::Density(0.4),
        ))
        .id();
    // The cluster entity gets the proxy rigid-body of the cluster, which joints and colliders
    // can be attached to like to any rigid-body.
    commands.spawn((
        PlateCluster,
        SoftBodyCluster::new(jelly, top),
        ImpulseJoint::new(
            plate,
            FixedJointBuilder::new().local_anchor1(Vec3::new(0.0, -0.1, 0.0)),
        ),
        // A cluster can be tuned as a whole.
        SoftBodyClusterMaterial {
            stiffness_scale: 2.0,
            ..default()
        },
        SoftBodyClusterShapeMatching::default(),
    ));
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: DeformableColliders start
    // A deformable triangle mesh bound to the jelly: each vertex is embedded in the cell
    // holding it (`skinned`), or follows one particle (`direct`). The vertices are placed by the
    // transform of the collider entity when the collider is created.
    let (vertices, indices) = Ball::new(1.0).to_trimesh(10, 10);
    commands.spawn((
        Transform::from_xyz(3.0, 1.0, 0.0),
        Collider::trimesh_with_flags(vertices, indices, TriMeshFlags::DEFORMABLE)
            .expect("a valid triangle mesh"),
        Sensor,
        DeformableCollider::new(jelly, SoftMeshBinding::skinned()),
    ));
    // DOCUSAURUS: DeformableColliders stop

    // DOCUSAURUS: Skinning start
    // A detailed mesh held by a coarse cage of cells: only the cells are simulated, and the mesh
    // (the skin) follows their deformation.
    let (vertices, indices) = Ball::new(0.5).to_trimesh(24, 24);
    let skinned = SoftBody::volumetric_skinned(&vertices, &indices, 0.25)
        .expect("the mesh must be closed and enclose some volume")
        // Collide through the skin instead of the boundary of the cage.
        .map(|builder| builder.skin_collision(true));
    commands.spawn((
        Transform::from_xyz(0.0, 4.0, 3.0),
        skinned,
        // The synchronized mesh renders the skin, since it is the collision mesh of the body.
        SoftBodyMeshSync::default(),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.3))),
    ));
    // DOCUSAURUS: Skinning stop
}

// DOCUSAURUS: Fem start
fn configure_fem(
    mut commands: Commands,
    mut simulation: Single<&mut RapierContextSimulation, With<DefaultRapierContext>>,
) {
    // A stiff beam simulated by the FEM solver (requires the `fem` feature): its stiffness
    // doesn't depend on the number of solver iterations.
    commands.spawn((
        Transform::from_xyz(0.0, 2.0, -3.0),
        SoftBody::cuboid(Vec3::new(1.0, 0.1, 0.1), 11, 3, 3).map(|builder| {
            builder
                .cell_model(SoftBodyCellModel::NeoHookean)
                // The particles of the face at `x = -1` are the first 3 × 3 ones.
                .pinned_particles(0..9)
        }),
        SoftBodyElasticitySolver(SoftBodySolver::Fem),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            young_modulus: 1.0e5,
            poisson_ratio: 0.3,
            ..default()
        }),
    ));

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    let fem = &mut simulation.integration_parameters.soft_bodies.fem;
    fem.linear_tolerance = 1.0e-5;
    fem.max_linear_iterations = 20;
}
// DOCUSAURUS: Fem stop

// DOCUSAURUS: Sets start
fn read_soft_bodies(context: ReadRapierContext, cloth: Single<Entity, With<Cloth>>) -> Result {
    let context = context.single()?;
    // The Rapier soft-bodies live in the `RapierRigidBodySet` of the context, together with the
    // map from their entity to their handle (also given by their `RapierSoftBodyHandle`).
    let Some(handle) = context.rigidbody_set.entity2soft_body().get(&*cloth) else {
        return Ok(()); // Not created yet.
    };
    let soft_body: &RapierSoftBody = &context.rigidbody_set.soft_bodies[*handle];
    // Shortcuts are provided for the most common operations.
    assert_eq!(context.soft_body_entity(*handle), Some(*cloth));
    assert_eq!(
        context.soft_body_particle_positions(*cloth).unwrap().len(),
        soft_body.num_particles()
    );
    let _center = context.soft_body_center_of_mass(*cloth);
    // The colliders of its surface, configured by its collider components.
    let surface_colliders = context
        .rigidbody_set
        .soft_body_colliders(&context.colliders.colliders, *cloth)
        .unwrap_or_default();
    for handle in surface_colliders {
        let _friction = context.colliders.colliders[handle].friction();
    }
    Ok(())
}
// DOCUSAURUS: Sets stop

// DOCUSAURUS: Particles start
fn control_particles(
    mut commands: Commands,
    mut context: WriteRapierContext,
    cloth: Single<Entity, With<Cloth>>,
) -> Result {
    let mut context = context.single_mut()?;
    let Some(soft_body) = context.soft_body_mut(*cloth) else {
        return Ok(());
    };
    // Read the particles (in world-space).
    let position = soft_body.particle_position(0);
    let velocity = soft_body.particle_velocity(0);
    let positions: Vec<Vec3> = soft_body.particle_positions().collect();
    assert_eq!(positions.len(), soft_body.num_particles());
    // Move a particle.
    soft_body.set_particle_position(1, position + Vec3::new(0.0, 0.1, 0.0));
    soft_body.set_particle_velocity(1, velocity);
    // The elements: edges, cells and the boundary triangles.
    let num_edges = soft_body.edges().len();
    let num_cells = soft_body.cells().len();
    let boundary: &[[u32; 3]] = soft_body.boundary();
    assert!(num_edges > 0 && num_cells == 0 && !boundary.is_empty());

    // Pin particles (exactly the listed ones), and drive the particle 2 kinematically.
    commands.entity(*cloth).insert((
        SoftBodyPinnedParticles(vec![0, 19, 380, 399, 2]),
        SoftBodyKinematicTargets(vec![(2, Vec3::new(-1.0, 2.5, -0.8))]),
    ));
    Ok(())
}
// DOCUSAURUS: Particles stop

// DOCUSAURUS: Forces start
fn apply_forces(
    mut commands: Commands,
    mut context: WriteRapierContext,
    cloth: Single<Entity, With<Cloth>>,
) -> Result {
    commands.entity(*cloth).insert((
        // Persistent forces: applied at each step until the component changes or is removed.
        SoftBodyExternalForce {
            force: Vec3::new(0.0, 1.0, 0.0),
            particle_forces: vec![(3, Vec3::new(0.0, 1.0, 0.0))],
        },
        // One-time impulses: applied (and reset to zero) at the next step.
        SoftBodyExternalImpulse {
            velocity_change: Vec3::new(0.0, 0.1, 0.0),
            particle_impulses: vec![(3, Vec3::new(0.0, 0.1, 0.0))],
        },
    ));

    // The other impulses are applied to the Rapier soft-body directly. The `true` argument
    // makes sure the soft-body is awake.
    let mut context = context.single_mut()?;
    if let Some(soft_body) = context.soft_body_mut(*cloth) {
        // An impulse on the particles within 0.5 of a point, scaled down with the distance.
        soft_body.apply_impulse_at_point(
            Vec3::new(0.0, 0.1, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            0.5,
            true,
        );
        // A blast pushing the particles away from a center.
        soft_body.apply_radial_impulse(Vec3::new(0.0, 2.0, 0.0), 0.1, 1.0, true);
    }
    Ok(())
}
// DOCUSAURUS: Forces stop

// DOCUSAURUS: ClusterControl start
fn drive_cluster(mut commands: Commands, cluster: Single<Entity, With<PlateCluster>>) {
    // Pin every particle of the cluster (the target inserts `SoftBodyClusterPinned`), and move
    // it to a world-space pose: the cluster behaves like a kinematic rigid part dragging the
    // rest of the body.
    commands
        .entity(*cluster)
        .insert(SoftBodyClusterKinematicTarget(Transform::from_xyz(
            3.0, 2.0, 0.0,
        )));
}

fn release_cluster(mut commands: Commands, cluster: Single<Entity, With<PlateCluster>>) {
    // Release it: the cluster is simulated again.
    commands
        .entity(*cluster)
        .remove::<(SoftBodyClusterKinematicTarget, SoftBodyClusterPinned)>();
}
// DOCUSAURUS: ClusterControl stop

// DOCUSAURUS: ReadDeformableColliders start
fn read_deformable_colliders(
    context: ReadRapierContext,
    colliders: Query<&RapierColliderHandle, With<DeformableCollider>>,
) -> Result {
    let context = context.single()?;
    for handle in &colliders {
        // The soft-body a collider follows.
        let collider = &context.colliders.colliders[handle.0];
        let Some(mesh_ref) = collider.deformable_mesh_ref() else {
            continue;
        };
        let soft_body = &context.rigidbody_set.soft_bodies[mesh_ref.body];
        // The mesh follows the particles: read its current vertices back (in world-space).
        let mesh = soft_body.mesh_of(handle.0).unwrap();
        let vertices: Vec<Vec3> = mesh.vertex_positions(soft_body).collect();
        assert!(!vertices.is_empty());
    }
    Ok(())
}
// DOCUSAURUS: ReadDeformableColliders stop

// DOCUSAURUS: Plasticity start
fn configure_plasticity(
    mut context: WriteRapierContext,
    jelly: Single<(Entity, &mut SoftBodyMaterial), With<Jelly>>,
) -> Result {
    // The jelly has elastic (corotational) cells: the plasticity of `Volume` cells has no effect.
    let (entity, mut material) = jelly.into_inner();
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
    if let Some(soft_body) = context.single_mut()?.soft_body_mut(entity) {
        soft_body.reset_plasticity();
    }
    Ok(())
}
// DOCUSAURUS: Plasticity stop

// DOCUSAURUS: TearingMaterial start
fn configure_tearing(mut material: Single<&mut SoftBodyMaterial, With<Cloth>>) {
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    material.tear_strain = Some(0.4);
    material.tear_force = Some(50.0);
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    material.tear_smoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    material.interior_strength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    material.min_piece = Some(10);
}
// DOCUSAURUS: TearingMaterial stop

// DOCUSAURUS: Tearing start
fn tear_cloth(
    mut commands: Commands,
    mut context: WriteRapierContext,
    cloth: Single<Entity, With<Cloth>>,
) -> Result {
    let mut context = context.single_mut()?;
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    if let Some(soft_body) = context.soft_body_mut(*cloth) {
        soft_body.tear_edge(10); // Applied at the end of the next step.
    }
    // Tear at once along edges and through cells. The pieces the tear disconnects become soft
    // bodies of their own, which entities are spawned right away with `commands`.
    if let Some(tear) = context.tear_soft_body(&mut commands, *cloth, &[11, 12], &[]) {
        println!("{} edges torn", tear.raw.torn_edges.len());
    }
    // Cut along a blade (a world-space triangle in 3D), without removing material.
    let blade = [
        Vec3::new(-0.1, -10.0, -10.0),
        Vec3::new(-0.1, 10.0, 0.0),
        Vec3::new(-0.1, -10.0, 10.0),
    ];
    if let Some(tear) = context.cut_soft_body(&mut commands, *cloth, &blade) {
        // The entities of the pieces (the first one being the torn entity) are known right away,
        // but they only get their components during the next writeback of the physics state.
        for (entity, piece) in tear.pieces.iter().zip(&tear.raw.pieces) {
            println!("piece {entity} has {} particles", piece.particles.len());
        }
    }
    Ok(())
}
// DOCUSAURUS: Tearing stop

// DOCUSAURUS: Events start
fn read_tear_events(context: ReadRapierContext, mut tears: MessageReader<SoftBodyTearEvent>) {
    let Ok(context) = context.single() else {
        return;
    };
    for tear in tears.read() {
        // The first piece is the torn entity itself, the others are the entities spawned for
        // the soft-bodies split off it.
        for piece in &tear.pieces {
            println!(
                "Soft body {} tore: piece {} has {} particles",
                tear.soft_body,
                piece.soft_body,
                piece.particles.len()
            );
        }
        // Where a particle of the torn body went.
        if let Some((handle, index)) = tear.raw.particle_destination(399) {
            let entity = context.soft_body_entity(handle);
            println!("particle 399 is now particle {index} of {entity:?}");
        }
    }
}
// DOCUSAURUS: Events stop

// DOCUSAURUS: Settings start
fn configure_soft_bodies(
    mut simulation: Single<&mut RapierContextSimulation, With<DefaultRapierContext>>,
) {
    // Settings shared by every soft-body of the physics context.
    let settings = &mut simulation.integration_parameters.soft_bodies;
    // Strain beyond which a constraint is re-solved after the contacts of every substep.
    // Default: 0.75
    settings.resweep_strain = 0.75;
    // Extra substeps a soft-body requests while it is hit fast; 0 disables them.
    // Default: 4
    settings.max_extra_substeps = 4;
    // Stiffening of the soft-body contacts relative to the rigid ones.
    // Default: 4.0
    settings.contact_stiffening = 4.0;
    // The tangle detection and recovery stack can be switched off mechanism by mechanism.
    settings.recovery.crossing_repulsion = true;
}
// DOCUSAURUS: Settings stop

// DOCUSAURUS: Removal start
fn remove_soft_bodies(
    mut commands: Commands,
    rope: Single<Entity, With<Rope>>,
    cluster: Single<Entity, With<PlateCluster>>,
) {
    // Despawning a soft-body entity (or removing its `SoftBody` component) removes its root
    // body, its proxies, its colliders and the joints attached to them.
    commands.entity(*rope).despawn();
    // Despawning a cluster entity (or removing its `SoftBodyCluster` component) removes the
    // cluster.
    commands.entity(*cluster).despawn();
}
// DOCUSAURUS: Removal stop
