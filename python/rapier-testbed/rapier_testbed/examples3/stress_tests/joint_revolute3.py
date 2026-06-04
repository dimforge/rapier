"""Port of examples3d/stress_tests/joint_revolute3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "ImpulseJoint revolute"

X = (1.0, 0.0, 0.0)
Z = (0.0, 0.0, 1.0)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    num = 10
    shift = 2.0

    for l in range(4):
        y = l * shift * num * 3.0
        for j in range(50):
            x = j * shift * 4.0
            curr_parent = bodies.insert(rp.RigidBody.fixed().translation((x, y, 0.0)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_parent, bodies)
            for i in range(num):
                z = i * shift * 2.0 + shift
                positions = [
                    (x, y, z),
                    (x + shift, y, z),
                    (x + shift, y, z + shift),
                    (x, y, z + shift),
                ]
                handles = []
                for k in range(4):
                    body = rp.RigidBody.dynamic().translation(positions[k])
                    h = bodies.insert(body)
                    handles.append(h)
                    colliders.insert_with_parent(
                        rp.Collider.cuboid(rad, rad, rad).density(1.0), h, bodies
                    )

                revs = [
                    rp.RevoluteJoint.builder(axis=Z).local_anchor2((0.0, 0.0, -shift)).build(),
                    rp.RevoluteJoint.builder(axis=X).local_anchor2((-shift, 0.0, 0.0)).build(),
                    rp.RevoluteJoint.builder(axis=Z).local_anchor2((0.0, 0.0, -shift)).build(),
                    rp.RevoluteJoint.builder(axis=X).local_anchor2((shift, 0.0, 0.0)).build(),
                ]
                impulse_joints.insert(curr_parent, handles[0], revs[0], wake_up=True)
                impulse_joints.insert(handles[0], handles[1], revs[1], wake_up=True)
                impulse_joints.insert(handles[1], handles[2], revs[2], wake_up=True)
                impulse_joints.insert(handles[2], handles[3], revs[3], wake_up=True)
                curr_parent = handles[3]

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((478.0, 83.0, 228.0), (134.0, 83.0, -116.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
