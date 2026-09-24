import pickle

import rapier3d as rp


def setup_physics_scene():
    """Adapted from the basic simulation example of the user guide."""
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

    # Create the ground.
    world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))

    # Create the bouncing ball.
    ball_handle = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
        colliders=[rp.Collider.ball(0.5).restitution(0.7)],
    )

    # Run the simulation for a while before serializing it.
    for _ in range(100):
        world.step()

    return world, ball_handle


world, ball_handle = setup_physics_scene()

# DOCUSAURUS: Serialization start
# Serialize the whole physics world.
serialized = world.snapshot()
# Deserialize it.
deserialized = rp.PhysicsWorld.restore(serialized)
# The simulation can continue using the deserialized world.
deserialized.step()
# DOCUSAURUS: Serialization stop
assert isinstance(serialized, bytes)

# DOCUSAURUS: Pickle start
# The world can be pickled like any other Python object, e.g., to save it on the disk.
data = pickle.dumps(world)
unpickled = pickle.loads(data)
# DOCUSAURUS: Pickle stop
assert unpickled.rigid_bodies[ball_handle].translation == world.rigid_bodies[ball_handle].translation

# DOCUSAURUS: RestoredHandles start
# The handles of the serialized world refer to the same objects in the deserialized world.
restored_ball = deserialized.rigid_bodies[ball_handle]
print("Ball altitude in the deserialized world:", restored_ball.translation.y)

# The event handler and the physics hooks aren't part of the snapshot: they must be given again.
deserialized.event_handler = rp.ChannelEventCollector()
# DOCUSAURUS: RestoredHandles stop

# DOCUSAURUS: Json start
# A human-readable (but much larger and slower) JSON snapshot, e.g., for debugging.
json_snapshot = world.snapshot_json()
from_json = rp.PhysicsWorld.restore_json(json_snapshot)
# DOCUSAURUS: Json stop
assert from_json.rigid_bodies[ball_handle].translation == world.rigid_bodies[ball_handle].translation

# DOCUSAURUS: Objects start
# Most objects can be pickled on their own too, e.g., the rigid-body and collider builders.
ball_builder = rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0))
restored_builder = pickle.loads(pickle.dumps(ball_builder))
new_ball = world.add_body(restored_builder, colliders=[rp.Collider.ball(0.5)])
# DOCUSAURUS: Objects stop
assert world.rigid_bodies[new_ball].translation.y == 10.0

# A corrupted snapshot is rejected.
try:
    rp.PhysicsWorld.restore(b"not a snapshot")
    raise AssertionError("expected a SerializationError")
except rp.SerializationError:
    pass
