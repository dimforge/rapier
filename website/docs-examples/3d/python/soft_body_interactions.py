import numpy as np
import rapier3d as rp

# A world with a ground.
world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.colliders.insert(rp.Collider.cuboid(10.0, 0.1, 10.0).build())

# A cloth pinned by its four corners.
n = 20
cloth_handle = world.add_soft_body(
    rp.SoftBody.cloth((-1.0, 2.0, -1.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1), n, n)
    .pinned_particles([0, n - 1, n * (n - 1), n * n - 1])
    .softness((30.0, 1.0))
    .particle_mass(0.05)
    .particle_radius(0.02)
    .surface_collider(rp.Collider.ball(0.05).friction(0.8))
    .self_contacts(True)
)

# A jelly cube with corotational elastic cells.
jelly_handle = world.add_soft_body(
    rp.SoftBody.cuboid(
        (3.0, 1.0, 0.0),
        (0.5, 0.5, 0.5),
        4,
        4,
        4,
        cell_model=rp.SoftBodyCellModel.COROTATIONAL,
        material=rp.SoftBodyMaterial(
            young_modulus=2.0e3,
            poisson_ratio=0.35,
            elastic_damping_ratio=0.5,
            plastic_yield=0.05,
            plastic_creep=20.0,
            tear_strain=0.4,
        ),
        particle_mass=0.2,
    )
)

# DOCUSAURUS: Oriented start
# A shell: a closed surface that is not oriented, so its inner side holds the bodies put
# inside it (a bowl, a box, a container). A closed surface is oriented by default.
bowl = rp.SoftBody.sphere((-3.0, 2.0, 0.0), 0.8, 2).oriented(False).softness((60.0, 1.0))
bowl_handle = world.add_soft_body(bowl)
# DOCUSAURUS: Oriented stop
bowl_surface = world.soft_bodies[bowl_handle].collision_mesh()
assert bowl_surface.is_closed and not bowl_surface.is_oriented
assert world.soft_bodies[jelly_handle].collision_mesh().is_oriented

# DOCUSAURUS: Forces start
soft_body = world.soft_bodies[cloth_handle]
# The soft-body is woken up, unless `wake_up=False` is given.
soft_body.reset_forces()  # Reset the forces to zero.
soft_body.add_force((0.0, 1.0, 0.0))  # Spread over the particles by mass.
soft_body.add_particle_force(3, (0.0, 1.0, 0.0))
soft_body.apply_impulse((0.0, 0.1, 0.0))
soft_body.apply_particle_impulse(3, (0.0, 0.1, 0.0))
# An impulse on the particles within 0.5 of a point, scaled down with the distance.
soft_body.apply_impulse_at_point((0.0, 0.1, 0.0), (0.0, 2.0, 0.0), 0.5)
# A blast pushing the particles away from a center.
soft_body.apply_radial_impulse((0.0, 2.0, 0.0), 0.1, 1.0)
# DOCUSAURUS: Forces stop

# DOCUSAURUS: Attachments start
# Attach the last particle of a rope to a rigid ball, at the particle's position.
rope = rp.SoftBody.rope((-0.5, 5.0, 3.0), (2.5, 5.0, 3.0), 30).pinned_particles([0]).softness((40.0, 1.0))
rope_handle = world.add_soft_body(rope)
last_position = world.soft_bodies[rope_handle].particle_position(29)
ball = world.add_body(
    rp.RigidBody.dynamic(translation=last_position - (0.0, 0.3, 0.0)),
    colliders=[rp.Collider.ball(0.25).density(2.0)],
)
world.soft_bodies[rope_handle].attach_particle(29, ball, world.rigid_bodies)
# DOCUSAURUS: Attachments stop
attachments = world.soft_bodies[rope_handle].particle_attachments
assert len(attachments) == 1 and attachments[0].particle == 29 and attachments[0].body == ball

# DOCUSAURUS: DeformableColliders start
# A deformable triangle mesh bound to the jelly: each vertex is embedded in the cell
# holding it (`skinned`), or follows one particle (`direct`). The mesh is given in the
# frame of the proxy it is attached to.
jelly = world.soft_bodies[jelly_handle]
root = jelly.root_body
to_root = world.rigid_bodies[root].position.inverse()
center = jelly.center_of_mass
r = 1.0
offsets = [(r, 0.0, 0.0), (-r, 0.0, 0.0), (0.0, r, 0.0), (0.0, -r, 0.0), (0.0, 0.0, r), (0.0, 0.0, -r)]
vertices = np.array([tuple(to_root.transform_point(center + v)) for v in offsets], dtype=np.float32)
indices = np.array(
    [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]],
    dtype=np.uint32,
)
skin = rp.Collider.trimesh(vertices, indices, rp.TriMeshFlags.DEFORMABLE).sensor(True)
# Raises `SoftBindingError` if the mesh can't be bound to the cluster of `root`.
skin_handle = world.insert_deformable(skin, rp.SoftMeshBinding.skinned(), root)
# The mesh follows the particles: read its current vertices back (a NumPy array).
mesh = world.soft_bodies[jelly_handle].mesh_of(skin_handle)
skin_vertices = mesh.vertices
assert skin_vertices.shape == (6, 3)
# DOCUSAURUS: DeformableColliders stop
assert world.colliders[skin_handle].deformable_mesh_ref.body == jelly_handle

# DOCUSAURUS: Skinning start
# A detailed mesh held by a coarse cage of cells: only the cells are simulated, and the mesh
# (the skin) follows their deformation.
vertices, indices = rp.Ball(0.5).to_trimesh(24, 24)
# Raises `MeshConversionError` if the mesh isn't closed or doesn't enclose any volume.
skinned = (
    rp.SoftBody.volumetric(vertices, indices, 0.25, skinned=True)
    # Collide through the skin instead of the boundary of the cage.
    .skin_collision(True)
    .translated((0.0, 4.0, 3.0))
)
skinned_handle = world.add_soft_body(skinned)
# The skin is the body's collision mesh: read its vertices back to render it.
skin = world.soft_bodies[skinned_handle].collision_mesh()
assert skin is not None and skin.is_skinned
assert skin.vertices.shape == vertices.shape
# DOCUSAURUS: Skinning stop

# DOCUSAURUS: Plasticity start
# The jelly has elastic (corotational) cells: the plasticity of `VOLUME` cells has no effect.
jelly = world.soft_bodies[jelly_handle]
# A live view of the material: setting one of its fields changes the body.
material = jelly.material
# Cells: the rest shape flows toward the current one past 5% strain, at a rate of 20 per
# second, up to a total permanent deformation of 50%.
material.plastic_yield = 0.05
material.plastic_creep = 20.0
material.plastic_max = 0.5
# Edges: the rest length flows past 10% strain, up to half the initial length, but only
# when squeezed (a dent stays, a stretch springs back).
material.edge_plastic_yield = 0.1
material.edge_plastic_creep = 10.0
material.edge_plastic_max = 0.5
material.edge_plastic_flow = rp.SoftEdgePlasticFlow.COMPRESSION
# Every permanent deformation can be undone at once.
jelly.reset_plasticity()
# DOCUSAURUS: Plasticity stop
assert world.soft_bodies[jelly_handle].material.edge_plastic_flow == rp.SoftEdgePlasticFlow.COMPRESSION

# DOCUSAURUS: TearingMaterial start
material = world.soft_bodies[cloth_handle].material
# An edge tears past 40% of stretch, or past a force of 50 along its direction (`None`
# disables a threshold).
material.tear_strain = 0.4
material.tear_force = 50.0
# The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
material.tear_smoothing = 0.1
# Undamaged interior elements are twice as tough: tears start from the surface.
material.interior_strength = 2.0
# A tear never splits off a piece smaller than 10 elements.
material.min_piece = 10
# DOCUSAURUS: TearingMaterial stop
assert world.soft_bodies[cloth_handle].material.min_piece == 10

# DOCUSAURUS: TearResistance start
# A perforation line: these edges tear at half the load of the others (1.0 restores the
# threshold of the material).
perforated = rp.SoftBody.rope((0.0, 6.0, -3.0), (3.0, 6.0, -3.0), 30).edge_tear_resistance(
    [(14, 0.5), (15, 0.5)]
)
perforated_handle = world.add_soft_body(perforated)
# The same, after the insertion.
cloth = world.soft_bodies[cloth_handle]
for edge in (30, 31, 32):
    cloth.set_edge_tear_resistance(edge, 0.5)
# Every element of the root cluster of the jelly (i.e., of the whole body) is twice as
# tough, and its first cell three times as tough.
jelly = world.soft_bodies[jelly_handle]
jelly.set_cluster_tear_resistance(0, 2.0)
jelly.set_cell_tear_resistance(0, 3.0)
# DOCUSAURUS: TearResistance stop
assert world.soft_bodies[perforated_handle].edge(14).tear_resistance == 0.5
assert world.soft_bodies[cloth_handle].edge(30).tear_resistance == 0.5
assert world.soft_bodies[jelly_handle].cell(0).tear_resistance == 3.0

# DOCUSAURUS: Tearing start
# Elements tear on their own past the material's thresholds; a tear can also be requested.
world.soft_bodies[cloth_handle].tear_edge(10)  # Applied at the end of the next step.
# Tear at once along edges and through cells; pieces the tear disconnects become soft
# bodies of their own.
event = world.tear_soft_body(cloth_handle, [11, 12], [])
if event is not None:
    print(f"{len(event.torn_edges)} edges torn")
# Cut along a blade (a triangle), without removing material.
blade = ((-0.1, -10.0, -10.0), (-0.1, 10.0, 0.0), (-0.1, -10.0, 10.0))
event = world.cut_soft_body(cloth_handle, blade)
if event is not None:
    for piece in event.pieces:
        print(f"piece {piece.soft_body} has {len(piece.particles)} particles")
    # Where a particle of the torn body went.
    destination = event.particle_destination(n * n - 1)
    if destination is not None:
        body, index = destination
        print(f"particle {n * n - 1} is now particle {index} of {body}")
# The connectivity of the cloth changed: its render mesh must be rebuilt.
assert world.soft_bodies[cloth_handle].topology_version > 0
# DOCUSAURUS: Tearing stop
assert event is not None and len(event.pieces) == 2

# DOCUSAURUS: Events start
# Tears applied during a step are reported to the event handler of the world.
collector = rp.ChannelEventCollector()
world.event_handler = collector
world.step()
for tear_event in collector.drain_soft_body_tear_events():
    print(f"Soft body {tear_event.soft_body} tore")
# DOCUSAURUS: Events stop

# DOCUSAURUS: Settings start
# Settings shared by every soft body of the world (a live view of the integration parameters).
settings = world.integration_parameters.soft_bodies
# Strain beyond which a constraint is re-solved after the contacts of every substep.
# Default: 0.75
settings.resweep_strain = 0.75
# Extra substeps a soft body requests while it is hit fast; 0 disables them.
# Default: 4
settings.max_extra_substeps = 4
# Stiffening of the soft-body contacts relative to the rigid ones.
# Default: 4.0
settings.contact_stiffening = 4.0
# The tangle detection and recovery stack can be switched off mechanism by mechanism.
settings.recovery.crossing_repulsion = True
# DOCUSAURUS: Settings stop
assert world.integration_parameters.soft_bodies.recovery.crossing_repulsion

for _ in range(10):
    world.step()
