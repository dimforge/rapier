"""Port of examples2d/debug_intersection2.rs.

The Rust version animates a moving probe and uses the testbed's
``set_body_color`` API to highlight intersected bodies. The Python
mini-testbed has no per-body color hook, so this port just builds the
static grid; intersection queries are exposed via ``QueryPipeline`` but
omitted here.
"""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Intersection"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 1.0
    count = 100

    for x in range(count):
        for y in range(count):
            xx = (x - count / 2.0) * rad * 3.0
            yy = (y - count / 2.0) * rad * 3.0
            handle = bodies.insert(rp.RigidBody.fixed().translation((xx, yy)))
            colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=50.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
