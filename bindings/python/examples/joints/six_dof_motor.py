"""GenericJoint with motors driving multiple axes simultaneously.

A dynamic body is connected to a fixed anchor through a GenericJoint with
no axes locked (all six DoF are free). Velocity motors drive linear-X and
angular-Z at constant targets. We then report the body's pose.

Run::

    python python/examples/joints/six_dof_motor.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, 0, 0))

    anchor = world.add_body(rp.RigidBody.fixed(translation=(0, 0, 0)))
    body = world.add_body(
        rp.RigidBody.dynamic(translation=(0.5, 0, 0)),
        colliders=[rp.Collider.cuboid(0.25, 0.25, 0.25).density(1.0)],
    )

    # GenericJoint with all axes free: motors will drive translation
    # along X and rotation around Z.
    j = (
        rp.GenericJoint.builder(locked_axes=rp.JointAxesMask.empty())
        .motor_velocity(rp.JointAxis.LIN_X, 1.0, 1.0)
        .motor_max_force(rp.JointAxis.LIN_X, 1000.0)
        .motor_velocity(rp.JointAxis.ANG_Z, 0.5, 1.0)
        .motor_max_force(rp.JointAxis.ANG_Z, 1000.0)
        .build()
    )
    world.impulse_joints.insert(anchor, body, j)

    for _ in range(240):
        world.step()

    b = world.rigid_bodies[body]
    print(f"motor: lin.x={b.translation.x:.2f} ang.z={b.angvel.z:.2f}")


if __name__ == "__main__":
    main()
