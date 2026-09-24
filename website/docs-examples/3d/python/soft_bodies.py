import numpy as np
import rapier3d as rp

# DOCUSAURUS: Creation start
# A world with a ground.
world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.add_collider(rp.Collider.cuboid(10.0, 0.1, 10.0))

# Builder for a rope of 20 particles between two points.
_ = rp.SoftBody.rope((0.0, 3.0, 0.0), (2.0, 3.0, 0.0), 20)
# Builder for a cloth: `nu` by `nv` particles, particle `(i, j)` at `origin + i * du + j * dv`.
_ = rp.SoftBody.cloth((-1.0, 2.0, -1.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1), 20, 20)
# Builder for a box of `nx * ny * nz` particles filled with tetrahedral cells.
_ = rp.SoftBody.cuboid((3.0, 1.0, 0.0), (0.5, 0.5, 0.5), 4, 4, 4)
# Builder for a hollow sphere holding its volume (a balloon).
_ = rp.SoftBody.sphere((0.0, 3.0, 3.0), 0.8, 2)
# Builder over raw particle positions; the elements are added by the setters.
_ = rp.SoftBodyBuilder([(0.0, 3.0, 0.0), (1.0, 3.0, 0.0)]).edges([(0, 1)])
n = 20
cloth = (
    rp.SoftBody.cloth((-1.0, 2.0, -1.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1), n, n)
    # Particles held in place.
    .pinned_particles([0, n - 1, n * (n - 1), n * n - 1])
    # A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
    .softness(rp.SpringCoefficients(30.0, 1.0))
    # The mass of each particle.
    # Default: 1.0
    .particle_mass(0.05)
    # The thickness of the particles, for collisions.
    # Default: 0.01
    .particle_radius(0.02)
    # The template of the body's colliders: its shape is replaced by the deformable surface.
    .surface_collider(rp.Collider.ball(0.05).friction(0.8))
    # Whether the surface collides with itself.
    # Default: False
    .self_contacts(True)
    # Whether the body may fall asleep.
    # Default: True
    .can_sleep(True)
)
# The setters can also be given as keyword arguments of the constructors.
_ = rp.SoftBody.rope((0.0, 3.0, 0.0), (2.0, 3.0, 0.0), 20, particle_mass=0.1, pinned_particles=[0])
# Insert the soft body: this creates its hidden root rigid body and its colliders.
cloth_handle = world.add_soft_body(cloth)
# DOCUSAURUS: Creation stop

# DOCUSAURUS: Volumetric start
# Fill a closed, outward-oriented triangle mesh with tetrahedral cells of about 0.2 in size;
# this raises a `MeshConversionError` if the mesh isn't closed or encloses no volume.
vertices, indices = rp.Cuboid((0.5, 0.25, 0.25)).to_trimesh()
block = rp.SoftBody.volumetric(vertices, indices, 0.2).translated((-3.0, 1.0, 0.0))
block_handle = world.add_soft_body(block)

# The same, with the meshing parameters spelled out: the cover of the mesh is subdivided
# once around its boundary, then smoothed, so it follows the mesh more closely.
params = rp.VolumeMeshParameters(0.2, cover_subdivisions=1, cover_smoothing=4)
smooth_block = rp.SoftBody.volumetric_with(vertices, indices, params)
smooth_block_handle = world.add_soft_body(smooth_block.translated((-3.0, 2.0, 0.0)))
# DOCUSAURUS: Volumetric stop
assert world.soft_bodies[block_handle].num_cells > 0
assert world.soft_bodies[smooth_block_handle].num_cells > 0

# DOCUSAURUS: ShapeMatching start
# A cloud of particles without any element: shape matching alone pulls them back toward
# their rest shape, placed where it best fits the current one.
points = 0.3 * np.array([(i % 3, i // 3 % 3 + 4.0, i // 9) for i in range(27)])
blob = (
    rp.SoftBodyBuilder(points)
    .shape_matching(True)
    .material(
        rp.SoftBodyMaterial(
            # How fast the particles are pulled back toward their rest shape.
            shape_matching_softness=rp.SpringCoefficients(5.0, 1.0),
        )
    )
    .particle_radius(0.1)
)
blob_handle = world.add_soft_body(blob)
# DOCUSAURUS: ShapeMatching stop

# DOCUSAURUS: Sets start
# The sets can also be used directly, without the `PhysicsWorld` façade.
soft_body_set = rp.SoftBodySet()
rigid_body_set = rp.RigidBodySet()
collider_set = rp.ColliderSet()
rope = rp.SoftBody.rope((0.0, 3.0, 0.0), (2.0, 3.0, 0.0), 20)
rope_handle = soft_body_set.insert(rope, rigid_body_set, collider_set)
soft_body = soft_body_set[rope_handle]
assert soft_body.num_particles == 20
# The state of the body as a whole.
print("Mass:", soft_body.mass, "center of mass:", soft_body.center_of_mass)
print("Sleeping:", soft_body.is_sleeping)
# Every soft body of the set.
for handle, soft_body in soft_body_set:
    print(handle, soft_body.num_particles)
# DOCUSAURUS: Sets stop

# DOCUSAURUS: Material start
# Elastic cells: a jelly cube with corotational linear elasticity.
jelly = (
    rp.SoftBody.cuboid((3.0, 1.0, 0.0), (0.5, 0.5, 0.5), 4, 4, 4)
    # The constitutive model of the cells: `VOLUME` (per-cell volume constraints,
    # the shape is held by the edges), `COROTATIONAL` or `NEO_HOOKEAN`.
    .cell_model(rp.SoftBodyCellModel.COROTATIONAL)
    .material(
        rp.SoftBodyMaterial(
            # Stiffness of the elastic cells.
            young_modulus=2.0e3,
            poisson_ratio=0.35,
            elastic_damping_ratio=0.5,
            # Plasticity: the rest shape flows past 5% strain, at 20 per second.
            plastic_yield=0.05,
            plastic_creep=20.0,
            # Tearing: an element past 40% strain tears.
            tear_strain=0.4,
        )
    )
    .particle_mass(0.2)
)
jelly_handle = world.add_soft_body(jelly)

# A material shared by the edges, bending constraints and volume constraints.
material = rp.SoftBodyMaterial.uniform(rp.SpringCoefficients(30.0, 1.0))
# Softness of the bending constraints, on top of a uniform 30 Hz softness.
material.bend_softness = rp.SpringCoefficients(3.0, 1.0)
world.soft_bodies[cloth_handle].material = material
# The `material` property is also a live view: its fields can be modified in place.
world.soft_bodies[cloth_handle].material.deformation_damping = 0.1
# DOCUSAURUS: Material stop
assert world.soft_bodies[cloth_handle].material.bend_softness.natural_frequency == 3.0

# DOCUSAURUS: Fem start
# A stiff beam simulated by the FEM solver: its stiffness doesn't depend on the number of
# solver iterations.
beam = (
    rp.SoftBody.cuboid((0.0, 2.0, -3.0), (1.0, 0.1, 0.1), 11, 3, 3)
    .solver(rp.SoftBodySolver.FEM)
    .cell_model(rp.SoftBodyCellModel.NEO_HOOKEAN)
    .material(rp.SoftBodyMaterial(young_modulus=1.0e5, poisson_ratio=0.3))
    # The particles of the face at `x = -1` are the first 3 × 3 ones.
    .pinned_particles(range(9))
)
beam_handle = world.add_soft_body(beam)

# The tuning of the linear solves of the FEM solver, shared by every body using it.
fem = world.integration_parameters.soft_bodies.fem
fem.linear_tolerance = 1.0e-5
fem.max_linear_iterations = 20
# DOCUSAURUS: Fem stop
assert world.soft_bodies[beam_handle].solver == rp.SoftBodySolver.FEM
assert world.integration_parameters.soft_bodies.fem.max_linear_iterations == 20

# DOCUSAURUS: Particles start
soft_body = world.soft_bodies[cloth_handle]
# Read the particles.
position = soft_body.particle_position(0)
velocity = soft_body.particle_velocity(0)
# All the positions (or velocities) at once, as an (N, 3) NumPy array.
positions = soft_body.particle_positions
assert positions.shape == (soft_body.num_particles, 3)
# Move a particle.
soft_body.set_particle_position(1, position + rp.Vec3(0.0, 0.1, 0.0))
soft_body.set_particle_velocity(1, velocity)
# Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
soft_body.set_particle_pinned(2, True)
soft_body.set_particle_kinematic_target(2, (-1.0, 2.5, -0.8))
# The elements, as NumPy arrays of particle indices: edges, cells and the boundary triangles.
edges = soft_body.edges  # Shape (E, 2).
cells = soft_body.cells  # Shape (C, 4).
boundary = soft_body.boundary  # Shape (B, 3).
assert len(edges) > 0 and len(cells) == 0 and len(boundary) > 0
# DOCUSAURUS: Particles stop

# Every position (or velocity) at once, from an (N, 3) NumPy array.
soft_body.particle_positions = positions + np.array([0.0, 0.1, 0.0], dtype=np.float32)
soft_body.particle_velocities = np.zeros_like(positions)
assert np.allclose(soft_body.particle_positions, positions + [0.0, 0.1, 0.0])

# DOCUSAURUS: RootBody start
# The rigid body the engine created for the whole soft body, read back after its insertion.
root = world.soft_bodies[jelly_handle].root_body
assert world.rigid_bodies[root].is_soft_frame

# A rigid collider attached to it follows the frame of the whole body: here a sensor
# detecting what comes close to the jelly.
sensor = world.add_collider(rp.Collider.ball(1.0).sensor(True), parent=root)

# A joint attached to it acts on the soft body as a whole: this one hangs the jelly under a
# fixed anchor by a spring.
anchor = world.add_body(rp.RigidBody.fixed(translation=(3.0, 4.0, 0.0)))
world.impulse_joints.insert(anchor, root, rp.SpringJointBuilder(2.5, 60.0, 2.0))
# DOCUSAURUS: RootBody stop

# DOCUSAURUS: Clusters start
# A cluster over the top particles of the jelly: a rigid proxy that joints and
# colliders can attach to.
positions = world.soft_bodies[jelly_handle].particle_positions
top = np.flatnonzero(positions[:, 1] > 1.3)
cluster = world.add_soft_body_cluster(jelly_handle, top)
assert cluster is not None, "at least one valid particle"
proxy = world.soft_bodies[jelly_handle].cluster_proxy(cluster)
# A rigid plate welded onto the cluster.
plate = world.add_body(
    rp.RigidBody.dynamic(translation=(3.0, 1.9, 0.0)),
    colliders=[rp.Collider.cuboid(0.7, 0.05, 0.7).density(0.4)],
)
world.impulse_joints.insert(
    plate, proxy, rp.FixedJointBuilder().local_anchor1((0.0, -0.1, 0.0))
)
# A cluster can be pinned, driven or tuned as a whole.
jelly = world.soft_bodies[jelly_handle]
jelly.set_cluster_stiffness_scale(cluster, 2.0)
jelly.enable_cluster_shape_matching(cluster, True)
# DOCUSAURUS: Clusters stop

# The target pose of the shape matching of a cluster (None: the frame of its proxy).
jelly.set_cluster_shape_matching_target(cluster, rp.Isometry3(translation=(3.0, 1.5, 0.0)))
jelly.set_cluster_shape_matching_target(cluster, None)

# DOCUSAURUS: ClusterControl start
# Pin every particle of the cluster, then move it along a path: the cluster behaves like a
# kinematic rigid part dragging the rest of the body.
jelly = world.soft_bodies[jelly_handle]
jelly.set_cluster_pinned(cluster, True)
jelly.set_cluster_kinematic_target(cluster, rp.Isometry3(translation=(3.0, 2.0, 0.0)))
# Release it: the cluster is simulated again.
jelly.set_cluster_pinned(cluster, False)
# DOCUSAURUS: ClusterControl stop

rope_handle = world.add_soft_body(
    rp.SoftBody.rope((-0.5, 5.0, 3.0), (2.5, 5.0, 3.0), 30, pinned_particles=[0])
)

# DOCUSAURUS: Removal start
# Removing a soft body removes its root body, its proxies, its colliders and the joints
# attached to them.
world.remove_soft_body(rope_handle)
# A cluster can be removed on its own.
world.remove_soft_body_cluster(jelly_handle, cluster)
# DOCUSAURUS: Removal stop
assert rope_handle not in world.soft_bodies
assert world.soft_bodies[jelly_handle].cluster_proxy(cluster) is None

for _ in range(10):
    world.step()
