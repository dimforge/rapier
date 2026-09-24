import os
import sys

# Run the testbed headlessly (no window) from its sources in the repository.
os.environ["PANDA_NO_WINDOW"] = "1"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../python/rapier-testbed"))

import rapier3d as rp


# DOCUSAURUS: Scene start
def bouncing_ball(testbed):
    # The scene itself, built like in any other application.
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
    ball_handle = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
        colliders=[rp.Collider.ball(0.5).restitution(0.7)],
    )

    # Hand the world to the testbed, and place the camera.
    testbed.set_world(world)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))

    # A function called after each timestep, e.g., to control the scene.
    def print_altitude(testbed):
        print("Ball altitude:", world.rigid_bodies[ball_handle].translation.y)

    testbed.add_callback(print_altitude)
# DOCUSAURUS: Scene stop


# DOCUSAURUS: Main start
from rapier_testbed import register, run

# The scenes listed by the picker of the testbed, as (category, name) pairs.
register("Demos", "Bouncing ball", bouncing_ball)

if __name__ == "__main__":
    # Open the testbed directly on this scene.
    run(initial="Demos / Bouncing ball")
# DOCUSAURUS: Main stop
