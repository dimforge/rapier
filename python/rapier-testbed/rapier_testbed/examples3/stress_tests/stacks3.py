"""Port of examples3d/stress_tests/stacks3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Stacks"


def _create_pyramid(bodies, colliders, offset, stack_height, hext):
    shift = (hext[0] * 2.0, hext[1] * 2.0, hext[2] * 2.0)
    for i in range(stack_height):
        for j in range(i, stack_height):
            for k in range(i, stack_height):
                fi = float(i)
                fj = float(j)
                fk = float(k)
                x = (
                    (fi * shift[0] / 2.0)
                    + (fk - fi) * shift[0]
                    + offset[0]
                    - stack_height * hext[0]
                )
                y = fi * shift[1] + offset[1]
                z = (
                    (fi * shift[2] / 2.0)
                    + (fj - fi) * shift[2]
                    + offset[2]
                    - stack_height * hext[2]
                )
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.cuboid(hext[0], hext[1], hext[2]), h, bodies
                )


def _create_wall(bodies, colliders, offset, stack_height, hext):
    shift = (hext[0] * 2.0, hext[1] * 2.0, hext[2] * 2.0)
    for i in range(stack_height):
        for j in range(i, stack_height):
            fj = float(j)
            fi = float(i)
            x = offset[0]
            y = fi * shift[1] + offset[1]
            z = (fi * shift[2] / 2.0) + (fj - fi) * shift[2] + offset[2] - stack_height * hext[2]
            body = rp.RigidBody.dynamic().translation((x, y, z))
            h = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(hext[0], hext[1], hext[2]), h, bodies
            )


def _create_tower_circle(bodies, colliders, offset, stack_height, nsubdivs, hext):
    ang_step = math.pi * 2.0 / nsubdivs
    radius = 1.3 * nsubdivs * hext[0] / math.pi
    shift = (hext[0] * 2.0, hext[1] * 2.0, hext[2] * 2.0)
    for i in range(stack_height):
        for j in range(nsubdivs):
            fj = float(j)
            fi = float(i)
            y = fi * shift[1]
            angle = (fi / 2.0 + fj) * ang_step
            rot = rp.Rotation3.from_axis_angle((0.0, 1.0, 0.0), angle)
            # Equivalent to: Isometry::translation(offset) * Rotation_y(angle) translated locally
            # by (0, y, radius).
            local_t = rot.transform_vector((0.0, y, radius))
            pos = rp.Isometry3(
                translation=(offset[0] + local_t.x, offset[1] + local_t.y, offset[2] + local_t.z),
                rotation=rot,
            )
            body = rp.RigidBody.dynamic().position(pos)
            h = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(hext[0], hext[1], hext[2]), h, bodies
            )


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 200.0
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    cube_size = 1.0
    hext = (cube_size, cube_size, cube_size)
    bottomy = cube_size * 50.0
    for x in (-110.0, -80.0, -50.0, -20.0):
        _create_pyramid(bodies, colliders, (x, bottomy, 0.0), 12, hext)
    for x in (-2.0, 4.0, 10.0):
        _create_wall(bodies, colliders, (x, bottomy, 0.0), 12, hext)
    _create_tower_circle(bodies, colliders, (25.0, bottomy, 0.0), 8, 24, hext)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
