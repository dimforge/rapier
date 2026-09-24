# DOCUSAURUS: basic_sim start
import rapier3d as rp

# The world owns every structure needed by the simulation.
world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

# Create the ground.
world.colliders.insert(rp.Collider.cuboid(100.0, 0.1, 100.0).build())

# Create the bouncing ball.
ball_body_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
    colliders=[rp.Collider.ball(0.5).restitution(0.7)],
)

# Run the game loop, stepping the simulation once per frame.
for _ in range(200):
    world.step()

    ball_body = world.rigid_bodies[ball_body_handle]
    print("Ball altitude:", ball_body.translation.y)
# DOCUSAURUS: basic_sim stop
