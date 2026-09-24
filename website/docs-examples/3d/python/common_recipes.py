import math

import rapier3d as rp


# DOCUSAURUS: OneWayPlatform start
class OneWayPlatform:
    def __init__(self, platform):
        self.platform = platform

    def modify_solver_contacts(self, context):
        # Keep only the contacts pushing along the local +y axis of the platform; the other
        # ones (the character arriving from below) are discarded. The normal is expressed in
        # the frame of the first collider of the pair, hence the flip.
        if context.collider1 == self.platform:
            allowed_local_n1 = (0.0, 1.0, 0.0)
        else:
            allowed_local_n1 = (0.0, -1.0, 0.0)
        context.update_as_oneway_platform(allowed_local_n1, 0.1)
# DOCUSAURUS: OneWayPlatform stop


# DOCUSAURUS: ConveyorBelt start
class ConveyorBelt:
    def __init__(self, belt):
        self.belt = belt

    def modify_solver_contacts(self, context):
        # The belt drags the objects along the world-space z axis at 12 m/s. The tangent
        # velocity is the one of the surface of the second collider relative to the first
        # one, hence the flip when the belt is the second collider.
        if context.collider1 == self.belt:
            context.set_tangent_velocity((0.0, 0.0, 12.0))
        else:
            context.set_tangent_velocity((0.0, 0.0, -12.0))
# DOCUSAURUS: ConveyorBelt stop


world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

# A box falling on the platform, so the hooks have contacts to modify.
box_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 1.5, 0.0)),
    colliders=[rp.Collider.cuboid(0.25, 0.25, 0.25)],
)

# DOCUSAURUS: MovingPlatform start
platform_handle = world.add_body(
    rp.RigidBody.kinematic_position_based(translation=(0.0, 1.0, 0.0)),
    colliders=[rp.Collider.cuboid(2.0, 0.1, 2.0)],
)

for step in range(200):
    # Setting the next position of the platform, once per timestep.
    time = step * world.integration_parameters.dt
    platform = world.rigid_bodies[platform_handle]
    platform.set_next_kinematic_translation((math.sin(time) * 2.0, 1.0, 0.0))
    world.step()
# DOCUSAURUS: MovingPlatform stop

# DOCUSAURUS: Hooks start
# The hooks are only called for the colliders asking for them.
platform_collider = world.rigid_bodies[platform_handle].colliders[0]
world.colliders[platform_collider].active_hooks = rp.ActiveHooks.MODIFY_SOLVER_CONTACTS

world.physics_hooks = OneWayPlatform(platform_collider)
world.step()
# DOCUSAURUS: Hooks stop

# The box rests on the platform, which moves with it.
for _ in range(60):
    world.step()
box_y = world.rigid_bodies[box_handle].translation.y
assert abs(box_y - 1.35) < 0.05, box_y

# The platform becomes a conveyor belt. The box may be asleep by now: wake it up so the
# hook is called for its contacts.
world.physics_hooks = ConveyorBelt(platform_collider)
world.rigid_bodies[box_handle].wake_up(True)
for _ in range(60):
    world.step()
assert world.rigid_bodies[box_handle].linvel.z > 1.0, world.rigid_bodies[box_handle].linvel
