import threading

import rapier3d as rp

# DOCUSAURUS: World start
# The world owns every structure needed by the simulation. Note that its gravity is zero
# unless it is given to its constructor.
world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.integration_parameters.dt = 1.0 / 60.0

# Create the ground: a collider without any parent rigid-body.
world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))

# Create the bouncing ball: the rigid-body and its colliders are inserted at once.
ball_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
    colliders=[rp.Collider.ball(0.5).restitution(0.7)],
)

# Run the game loop, stepping the simulation once per frame.
for _ in range(200):
    world.step()
    print("Ball altitude:", world.rigid_bodies[ball_handle].translation.y)
# DOCUSAURUS: World stop

# DOCUSAURUS: WorldQueries start
# The scene queries are run by the query pipeline of the world.
ray = rp.Ray((0.0, 10.0, 0.0), (0.0, -1.0, 0.0))
hit = world.query_pipeline.cast_ray(ray, max_toi=100.0, solid=True)
if hit is not None:
    handle, distance = hit
    print(f"Collider {handle} hit at distance {distance}")

# Every structure remains reachable as a property of the world.
num_pairs = len(world.narrow_phase.contact_pairs())
print(f"{num_pairs} contact pairs")
# DOCUSAURUS: WorldQueries stop
assert hit is not None

# DOCUSAURUS: Handles start
# Each object inserted into the world is identified by a handle.
box_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(2.0, 1.0, 0.0)),
    colliders=[rp.Collider.cuboid(0.5, 0.5, 0.5)],
)
print(f"Index: {box_handle.index}, generation: {box_handle.generation}")

# The object given by a set is a live view of the object it contains: modifying it
# modifies the object stored in the world.
box = world.rigid_bodies[box_handle]
box.linvel = (1.0, 0.0, 0.0)
assert world.rigid_bodies[box_handle].linvel.x == 1.0

# The sets can be iterated, and their length is their number of objects.
for handle, body in world.rigid_bodies:
    print(f"Rigid body {handle} at {body.translation}")
print(f"{len(world.colliders)} colliders")
# DOCUSAURUS: Handles stop

# DOCUSAURUS: Removal start
# Removing a rigid-body also removes its colliders and the joints attached to it.
world.remove_body(box_handle)

# The handle is now stale: it doesn't refer to any object of the world anymore.
assert box_handle not in world.rigid_bodies
assert world.rigid_bodies.get(box_handle) is None
try:
    world.rigid_bodies[box_handle]
except rp.InvalidHandle:
    print("The box was removed.")
# DOCUSAURUS: Removal stop

# DOCUSAURUS: UserData start
# The user data is an integer of your choice, e.g., the index of a game object.
game_objects = ["player", "enemy"]
enemy_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(-2.0, 1.0, 0.0)).user_data(1),
    colliders=[rp.Collider.ball(0.5).user_data(1)],
)
enemy = world.rigid_bodies[enemy_handle]
print("This rigid-body belongs to the", game_objects[enemy.user_data])
# It can be modified at any time.
enemy.user_data = 0
# DOCUSAURUS: UserData stop
assert world.rigid_bodies[enemy_handle].user_data == 0

# DOCUSAURUS: ThreadPool start
# Give this world its own pool of four worker threads.
world.set_num_threads(4)
assert world.num_threads == 4
# Run everything on the thread calling `world.step()`.
world.set_num_threads(1)
# Go back to the global pool shared by every world (one worker per logical CPU).
world.set_num_threads(None)
# DOCUSAURUS: ThreadPool stop


# DOCUSAURUS: ParallelWorlds start
def simulate(results, i):
    # Each thread simulates its own world.
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 1.0 + i, 0.0)),
        colliders=[rp.Collider.ball(0.5)],
    )
    for _ in range(100):
        # The GIL is released during the step, so the other threads keep running.
        world.step()
    results[i] = world.rigid_bodies[ball].translation.y


results = [None] * 4
threads = [threading.Thread(target=simulate, args=(results, i)) for i in range(4)]
for thread in threads:
    thread.start()
for thread in threads:
    thread.join()
# DOCUSAURUS: ParallelWorlds stop
assert all(y is not None for y in results)

for _ in range(10):
    world.step()
    # DOCUSAURUS: IslandManager start
    # Iter on each rigid-bodies that moved (dynamic and kinematic).
    for rigid_body_handle in world.active_bodies():
        rigid_body = world.rigid_bodies[rigid_body_handle]
        print(f"Rigid body {rigid_body_handle} has a new position: {rigid_body.position}")
    # DOCUSAURUS: IslandManager stop

# DOCUSAURUS: DetectCollisions start
# Teleport the ball, then update the contacts and the scene queries right away.
world.rigid_bodies[ball_handle].translation = (0.0, 5.0, 0.0)
world.detect_collisions()

# The ray-cast now hits the ball at its new position.
hit = world.query_pipeline.cast_ray(ray, max_toi=100.0, solid=True)
# DOCUSAURUS: DetectCollisions stop
assert hit is not None and abs(hit[1] - 4.5) < 1.0e-3

# DOCUSAURUS: Measuring start
world.step()
# The counters of the pipeline measure the last timestep (they are enabled by default).
counters = world.physics_pipeline.counters
print(f"Step time: {counters.step_time_ms} ms")
print(f"Collision detection: {counters.stages.collision_detection_time_ms} ms")
print(f"Solver: {counters.stages.solver_time_ms} ms")
# Disable them to save the (small) cost of the measurements.
counters.disable()
# DOCUSAURUS: Measuring stop
assert not world.physics_pipeline.counters.enabled
world.physics_pipeline.counters.enable()

# DOCUSAURUS: QueryPipeline start
# A query pipeline refers to the broad-phase, the narrow-phase, and the sets it reads. This is
# what `PhysicsWorld.query_pipeline` is made of.
query_pipeline = rp.QueryPipeline(
    world.broad_phase, world.narrow_phase, world.rigid_bodies, world.colliders
)
# DOCUSAURUS: QueryPipeline stop
assert query_pipeline.cast_ray(ray, max_toi=100.0, solid=True) is not None

# DOCUSAURUS: basic_sim_manual start
rigid_body_set = rp.RigidBodySet()
collider_set = rp.ColliderSet()

# Create the ground.
collider_set.insert(rp.Collider.cuboid(100.0, 0.1, 100.0))

# Create the bouncing ball.
ball_body_handle = rigid_body_set.insert(rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)))
collider = rp.Collider.ball(0.5).restitution(0.7)
collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set)

# Create other structures necessary for the simulation.
gravity = (0.0, -9.81, 0.0)
integration_parameters = rp.IntegrationParameters()
physics_pipeline = rp.PhysicsPipeline()
island_manager = rp.IslandManager()
broad_phase = rp.BroadPhaseBvh()
narrow_phase = rp.NarrowPhase()
impulse_joint_set = rp.ImpulseJointSet()
multibody_joint_set = rp.MultibodyJointSet()
soft_body_set = rp.SoftBodySet()
ccd_solver = rp.CCDSolver()
physics_hooks = None
event_handler = None

# Run the game loop, stepping the simulation once per frame.
for _ in range(200):
    physics_pipeline.step(
        gravity,
        integration_parameters,
        island_manager,
        broad_phase,
        narrow_phase,
        rigid_body_set,
        collider_set,
        impulse_joint_set,
        multibody_joint_set,
        ccd_solver,
        hooks=physics_hooks,
        events=event_handler,
        soft_bodies=soft_body_set,
    )

    ball_body = rigid_body_set[ball_body_handle]
    print("Ball altitude:", ball_body.translation.y)
# DOCUSAURUS: basic_sim_manual stop
