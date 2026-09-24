import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
world.add_body(rp.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)), colliders=[rp.Collider.ball(0.5)])

# DOCUSAURUS: IntegrationParameters start
# The integration parameters of the world are modified in place: the property always gives
# the same object.
params = world.integration_parameters
params.dt = 1.0 / 120.0
params.num_solver_iterations = 8
params.warmstart_joints = True
print(f"dt = {params.dt}, {world.integration_parameters.num_solver_iterations} solver iterations")

# They can also be replaced all at once, e.g., to reset them to their default values.
world.integration_parameters = rp.IntegrationParameters()
# DOCUSAURUS: IntegrationParameters stop
assert world.integration_parameters.num_solver_iterations == 4
assert params.num_solver_iterations == 4

# DOCUSAURUS: LengthUnit start
# The simulation is measured in centimeters.
world.integration_parameters.length_unit = 100.0
# The gravity isn't scaled by the length unit: it must be given in centimeters too.
world.gravity = (0.0, -981.0, 0.0)
# DOCUSAURUS: LengthUnit stop
world.step()
world.integration_parameters.length_unit = 1.0
world.gravity = (0.0, -9.81, 0.0)

# DOCUSAURUS: ContactSoftness start
params = world.integration_parameters
# The softness is given as a copy: modify it, then assign it back.
softness = params.contact_softness
print(f"{softness.natural_frequency} Hz, damping ratio {softness.damping_ratio}")
# Contacts against fixed rigid-bodies are stiffer by default: make them as soft as the others.
params.static_contact_softness = softness
# DOCUSAURUS: ContactSoftness stop
assert params.static_contact_softness.natural_frequency == softness.natural_frequency

# DOCUSAURUS: FrictionModel start
# Solve one Coulomb friction constraint per contact point.
world.integration_parameters.friction_model = rp.FrictionModel.COULOMB
# DOCUSAURUS: FrictionModel stop
assert world.integration_parameters.friction_model == rp.FrictionModel.COULOMB

world.step()
