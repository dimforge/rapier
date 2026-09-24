"""Port of examples3d/keva3.rs (Keva-style tower)."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Keva tower"


def _build_block(bodies, colliders, half_extents, shift, dims):
    """Build a single block of the Keva tower.

    ``half_extents`` is a 3-tuple; ``dims`` is ``(numx, numy, numz)``.
    """
    numx, numy, numz = dims
    dimensions = [
        half_extents,
        (half_extents[2], half_extents[1], half_extents[0]),
    ]
    block_width = 2.0 * half_extents[2] * numx
    block_height = 2.0 * half_extents[1] * numy
    spacing = (half_extents[2] * numx - half_extents[0]) / (numz - 1.0)

    for i in range(numy):
        # mem::swap(&mut numx, &mut numz)
        numx, numz = numz, numx
        dim = dimensions[i % 2]
        y = dim[1] * i * 2.0
        for j in range(numx):
            if i % 2 == 0:
                x = spacing * j * 2.0
            else:
                x = dim[0] * j * 2.0
            for k in range(numz):
                if i % 2 == 0:
                    z = dim[2] * k * 2.0
                else:
                    z = spacing * k * 2.0
                body = rp.RigidBody.dynamic().translation(
                    (x + dim[0] + shift[0], y + dim[1] + shift[1], z + dim[2] + shift[2])
                )
                h = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.cuboid(dim[0], dim[1], dim[2]), h, bodies
                )

    # Top cap.
    dim = (half_extents[2], half_extents[0], half_extents[1])
    for i in range(int(block_width / (dim[0] * 2.0))):
        for j in range(int(block_width / (dim[2] * 2.0))):
            body = rp.RigidBody.dynamic().translation(
                (
                    i * dim[0] * 2.0 + dim[0] + shift[0],
                    dim[1] + shift[1] + block_height,
                    j * dim[2] * 2.0 + dim[2] + shift[2],
                )
            )
            h = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(dim[0], dim[1], dim[2]), h, bodies
            )


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 50.0
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    # `Vector::new(0.02, 0.1, 0.4) / 2.0 * 10.0` → (0.1, 0.5, 2.0).
    half_extents = (0.02 / 2.0 * 10.0, 0.1 / 2.0 * 10.0, 0.4 / 2.0 * 10.0)
    block_height = 0.0
    numy = [0, 9, 13, 17, 21, 41]
    for i in range(5, 0, -1):
        numx = i
        ny = numy[i]
        numz = numx * 3 + 1
        block_width = numx * half_extents[2] * 2.0
        _build_block(
            bodies,
            colliders,
            half_extents,
            (-block_width / 2.0, block_height, -block_width / 2.0),
            (numx, ny, numz),
        )
        block_height += ny * half_extents[1] * 2.0 + half_extents[0] * 2.0

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
