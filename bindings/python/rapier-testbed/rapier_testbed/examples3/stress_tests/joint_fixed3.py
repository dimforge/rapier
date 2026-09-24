"""Port of examples3d/stress_tests/joint_fixed3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "ImpulseJoint fixed"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    num = 5
    shift = 1.0
    body_handles = []

    for m in range(10):
        z0 = m * shift * (num + 2.0)
        for l in range(10):
            y0 = l * shift * 3.0
            for jj in range(5):
                x0 = jj * shift * num * 2.0
                for k in range(num):
                    for i in range(num):
                        fk = float(k)
                        fi = float(i)
                        if i == 0 and ((k % 4 == 0 and k != num - 2) or k == num - 1):
                            bb = rp.RigidBody.fixed()
                        else:
                            bb = rp.RigidBody.dynamic()
                        bb = bb.translation((x0 + fk * shift, y0, z0 + fi * shift))
                        child = bodies.insert(bb)
                        colliders.insert_with_parent(rp.Collider.ball(rad), child, bodies)

                        if i > 0:
                            parent = body_handles[-1]
                            j = rp.FixedJoint.builder().local_anchor2((0.0, 0.0, -shift)).build()
                            impulse_joints.insert(parent, child, j, wake_up=True)
                        if k > 0:
                            parent = body_handles[len(body_handles) - num]
                            j = rp.FixedJoint.builder().local_anchor2((-shift, 0.0, 0.0)).build()
                            impulse_joints.insert(parent, child, j, wake_up=True)
                        body_handles.append(child)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((-38.0, 14.0, 108.0), (46.0, 12.0, 23.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
