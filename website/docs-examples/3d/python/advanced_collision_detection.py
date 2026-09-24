import math

import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

# Create the ground.
collider_handle1 = world.add_collider(
    rp.Collider.cuboid(100.0, 0.1, 100.0)
    .active_events(rp.ActiveEvents.COLLISION_EVENTS | rp.ActiveEvents.CONTACT_FORCE_EVENTS)
)

# Create the bouncing ball.
ball_body_handle = world.add_body(rp.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)))
collider_handle2 = world.add_collider(rp.Collider.ball(0.5).restitution(0.7), parent=ball_body_handle)

# DOCUSAURUS: Events start
# Initialize the event collector.
event_handler = rp.ChannelEventCollector()
world.event_handler = event_handler

world.step()

for collision_event in event_handler.drain_collision_events():
    # Handle the collision event.
    print("Received collision event:", collision_event)

for contact_force_event in event_handler.drain_contact_force_events():
    # Handle the contact force event.
    print("Received contact force event:", contact_force_event)

for tear_event in event_handler.drain_soft_body_tear_events():
    # Handle the soft-body tear event.
    print("Received soft-body tear event:", tear_event)
# DOCUSAURUS: Events stop

for _ in range(20):
    world.step()


# DOCUSAURUS: EventHandler start
class MyEventHandler:
    def handle_collision_event(self, bodies, colliders, event, contact_pair):
        # The contact pair is a copy of the contacts at the time of the event
        # (it is `None` if one of the colliders is a sensor).
        if event.started and contact_pair is not None:
            deepest_contact = contact_pair.find_deepest_contact()
            print("Collision started with the contact:", deepest_contact)
        # The sets of the world can be read (but not modified) from the event handler.
        parent1 = colliders[event.collider1].parent
        print("The first collider is attached to the rigid-body:", parent1)

    def handle_contact_force_event(self, dt, bodies, colliders, contact_pair, total_force_magnitude):
        print(
            f"Contact force {total_force_magnitude} between",
            contact_pair.collider1,
            "and",
            contact_pair.collider2,
        )

    def handle_soft_body_tear_event(self, soft_bodies, event):
        # This method is optional.
        print("Soft-body torn:", event)


world.event_handler = MyEventHandler()
world.step()
# DOCUSAURUS: EventHandler stop

world.event_handler = None
world.step()
assert world.narrow_phase.contact_pair(collider_handle1, collider_handle2).has_any_active_contact

# DOCUSAURUS: ContactGraph1 start
# Find the contact pair, if it exists, between two colliders.
contact_pair = world.narrow_phase.contact_pair(collider_handle1, collider_handle2)
if contact_pair is not None:
    # The contact pair exists meaning that the broad-phase identified a potential contact.
    if contact_pair.has_any_active_contact:
        # The contact pair has active contacts, meaning that it
        # contains contacts for which contact forces were computed.
        pass

    # We may also read the contact manifolds to access the contact geometry.
    for manifold in contact_pair.manifolds:
        print("Local-space contact normal:", manifold.local_n1)
        print("Local-space contact normal:", manifold.local_n2)
        print("World-space contact normal:", manifold.data.normal)

        # Read the geometric contacts.
        for contact_point in manifold.points:
            # Keep in mind that all the geometric contact data are expressed in the local-space of the colliders.
            print("Found local contact point 1:", contact_point.local_p1)
            print("Found contact distance:", contact_point.dist)  # Negative if there is a penetration.
            print("Found contact impulse:", contact_point.impulse)
            print("Found friction impulse:", contact_point.tangent_impulse)

        # Read the solver contacts.
        for solver_contact in manifold.data.solver_contacts:
            # Solver contacts are anchored in the local-space of the body they touch, so
            # they ride rigidly with it. Resolve them through the bodies' current poses to
            # get the world-space contact point on each body's surface.
            point1, point2 = manifold.data.solver_contact_world_points(solver_contact, world.rigid_bodies)
            print("Found solver contact points:", point1, point2)
            # The solver contact distance is negative if there is a penetration.
            print("Found solver contact distance:", solver_contact.dist)
# DOCUSAURUS: ContactGraph1 stop

# DOCUSAURUS: ContactGraph2 start
# Iterate through all the contact pairs involving a specific collider.
for contact_pair in world.narrow_phase.contact_pairs_with(collider_handle1):
    if contact_pair.collider1 == collider_handle1:
        other_collider = contact_pair.collider2
    else:
        other_collider = contact_pair.collider1

    # Process the contact pair in a way similar to what we did in
    # the previous example.
# DOCUSAURUS: ContactGraph2 stop

# DOCUSAURUS: IntersectionGraph1 start
# Find the intersection pair, if it exists, between two colliders.
if world.narrow_phase.intersection_pair(collider_handle1, collider_handle2):
    print(f"The colliders {collider_handle1} and {collider_handle2} are intersecting!")
# DOCUSAURUS: IntersectionGraph1 stop

# DOCUSAURUS: IntersectionGraph2 start
# Iterate through all the intersection pairs involving a specific collider.
for collider1, collider2, intersecting in world.narrow_phase.intersection_pairs_with(collider_handle1):
    if intersecting:
        print(f"The colliders {collider1} and {collider2} are intersecting!")
# DOCUSAURUS: IntersectionGraph2 stop


# DOCUSAURUS: PhysicsHooks start
class MyPhysicsHooks:
    def filter_contact_pair(self, context):
        # This is a silly example of contact pair filter that:
        # - Enables contact and force computation if both colliders have even user-data.
        # - Enables contact computation but not force computation if both colliders have equal user-data.
        # - Disables contact computation otherwise.
        user_data1 = context.colliders[context.collider1].user_data
        user_data2 = context.colliders[context.collider2].user_data

        if user_data1 % 2 == 0 and user_data2 % 2 == 0:
            return rp.SolverFlags.COMPUTE_RIGID_IMPULSES
        elif user_data1 == user_data2:
            return rp.SolverFlags.empty()
        else:
            return None

    def filter_intersection_pair(self, context):
        # This is a silly example of intersection pair filter that
        # enables the intersection test if both colliders have odd
        # user-data.
        user_data1 = context.colliders[context.collider1].user_data
        user_data2 = context.colliders[context.collider2].user_data

        return user_data1 % 2 == 1 and user_data2 % 2 == 1


world.physics_hooks = MyPhysicsHooks()
# DOCUSAURUS: PhysicsHooks stop

world.colliders[collider_handle1].active_hooks = rp.ActiveHooks.FILTER_CONTACT_PAIRS
world.step()
world.colliders[collider_handle1].active_hooks = rp.ActiveHooks.empty()


# DOCUSAURUS: ContactModification start
class MyPhysicsHooks:
    def modify_solver_contacts(self, context):
        # This is a silly example of contact modifier that does silly things
        # for illustration purpose:
        # - Flip all the contact normals.
        # - Delete the first contact.
        # - Set the friction coefficient to 0.3
        # - Set the restitution coefficient to 0.4
        # - Set the tangent velocities to X * 10.0
        context.normal = -context.normal

        if context.num_solver_contacts() > 0:
            context.remove_solver_contact(0)

        # Friction and restitution are combined once per manifold, so they are set
        # for the whole manifold rather than per solver contact.
        context.friction = 0.3
        context.restitution = 0.4

        for i in range(context.num_solver_contacts()):
            context.set_solver_contact(i, tangent_velocity=(10.0, 0.0, 0.0))

        # Use the persistent user-data to count the number of times
        # contact modification was called for this contact manifold
        # since its creation.
        context.user_data += 1
        print(f"Contact manifold has been modified {context.user_data} times since its creation.")


world.physics_hooks = MyPhysicsHooks()
# DOCUSAURUS: ContactModification stop

world.colliders[collider_handle1].active_hooks = rp.ActiveHooks.MODIFY_SOLVER_CONTACTS
world.step()
assert world.narrow_phase.contact_pair(collider_handle1, collider_handle2).manifolds[0].data.user_data == 1
world.colliders[collider_handle1].active_hooks = rp.ActiveHooks.empty()


# DOCUSAURUS: OneWayPlatform start
class OneWayPlatformHooks:
    def __init__(self, platform):
        self.platform = platform

    def modify_solver_contacts(self, context):
        # The allowed normal is expressed in the local-space of the first collider of the pair:
        # it points upward if the platform is that first collider, and downward otherwise.
        if context.collider1 == self.platform:
            allowed_local_n1 = (0.0, 1.0, 0.0)
        else:
            allowed_local_n1 = (0.0, -1.0, 0.0)

        # Remove the contacts unless the normal is within 45 degrees of the allowed one, so
        # that the colliders can pass through the platform from below.
        context.update_as_oneway_platform(allowed_local_n1, math.pi / 4.0)


platform_handle = world.add_collider(
    rp.Collider.cuboid(2.0, 0.1, 2.0)
    .translation((0.0, 3.0, 0.0))
    .active_hooks(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS)
)
world.physics_hooks = OneWayPlatformHooks(platform_handle)
# DOCUSAURUS: OneWayPlatform stop

# A ball thrown upward from below the platform goes through it, then lands on it.
thrown_body_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 2.0, 0.0), linvel=(0.0, 10.0, 0.0)),
    colliders=[rp.Collider.ball(0.25)],
)
for _ in range(300):
    world.step()
thrown_altitude = world.rigid_bodies[thrown_body_handle].translation.y
assert abs(thrown_altitude - 3.35) < 0.05, thrown_altitude
