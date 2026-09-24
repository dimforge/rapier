import rapier3d as rp

# DOCUSAURUS: BuildProfile start
# A debug build of the bindings is up to 100 times slower.
if rp.build_features().profile != "release":
    print("Warning: the rapier3d bindings are built without optimizations.")
# DOCUSAURUS: BuildProfile stop

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
ground = world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
ball_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 0.6, 0.0)),
    colliders=[rp.Collider.ball(0.5)],
)

# A non-finite velocity, e.g., after a division by a zero-length vector.
world.rigid_bodies[ball_handle].linvel = (float("nan"), 0.0, 0.0)

# DOCUSAURUS: Quarantine start
world.step()

# After the step, the objects neutralized by this step are known.
for handle in world.quarantine.bodies:
    print(f"The rigid-body {handle} went non-finite and was disabled.")
    # Once the cause is fixed, the rigid-body is brought back into the simulation.
    world.rigid_bodies[handle].is_enabled = True
# DOCUSAURUS: Quarantine stop
assert world.quarantine.bodies == [ball_handle]
assert world.rigid_bodies[ball_handle].is_enabled

world.step()
ball = world.rigid_bodies[ball_handle]

# DOCUSAURUS: PixelsPerMeter start
PIXELS_PER_METER = 50.0
# Scale the translation to convert from meters to pixels.
sprite_translation = ball.translation * PIXELS_PER_METER
# Rotations don't need to be scaled.
sprite_rotation = ball.rotation
# DOCUSAURUS: PixelsPerMeter stop

# DOCUSAURUS: Copies start
# The structures given by the properties of the integration parameters are copies.
params = world.integration_parameters
softness = params.contact_softness
softness.natural_frequency = 60.0  # Modifies the copy only.
params.contact_softness = softness  # Applies the modification.

# A rigid-body (or a collider) inserted into a set is copied into the set as well.
body = rp.RigidBody.dynamic(translation=(0.0, 3.0, 0.0)).build()
handle = world.rigid_bodies.insert(body)
body.linvel = (1.0, 0.0, 0.0)  # No effect on the rigid-body of the world.
world.rigid_bodies[handle].linvel = (1.0, 0.0, 0.0)  # Modifies the rigid-body of the world.

# The vectors are immutable: a whole new vector is given instead.
world.rigid_bodies[handle].translation = (0.0, 4.0, 0.0)
# DOCUSAURUS: Copies stop
assert world.integration_parameters.contact_softness.natural_frequency == 60.0
assert world.rigid_bodies[handle].linvel.x == 1.0
assert body.linvel.x == 1.0

# Ask for the contacts of the ground to be modified by the hooks.
world.colliders[ground].active_hooks = rp.ActiveHooks.MODIFY_SOLVER_CONTACTS
for _ in range(10):
    world.step()


# DOCUSAURUS: CallbackErrors start
class Hooks:
    def modify_solver_contacts(self, context):
        raise ValueError("Something went wrong in the hook.")


world.physics_hooks = Hooks()
try:
    world.step()
except ValueError as error:
    # Raised once the timestep is complete.
    print("The hook failed:", error)
# DOCUSAURUS: CallbackErrors stop
else:
    raise AssertionError("expected the error of the hook")
world.physics_hooks = None


# DOCUSAURUS: DeferredChanges start
class GroundContacts:
    def __init__(self):
        # The handles of the rigid-bodies touching the ground, recorded during the timestep.
        self.bodies_to_push = []

    def modify_solver_contacts(self, context):
        # The world can be read here, but not modified: only record what must be done.
        other_body = context.rigid_body2 if context.collider1 == ground else context.rigid_body1
        if other_body is not None:
            self.bodies_to_push.append(other_body)


hooks = GroundContacts()
world.physics_hooks = hooks
world.step()

# The world can be modified once the timestep is complete.
for body_handle in hooks.bodies_to_push:
    world.rigid_bodies[body_handle].apply_impulse((0.0, 0.1, 0.0))
hooks.bodies_to_push.clear()
# DOCUSAURUS: DeferredChanges stop
world.physics_hooks = None
world.step()
