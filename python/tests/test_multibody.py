"""Multibody joint tests (3D)."""

from __future__ import annotations

import math

import pytest

import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


def test_multibody_three_body_chain(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.fixed(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    c = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(2, 0, 0)).build())

    # Three-body chain via Revolute joints around Y.
    j_ab = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    j_bc = w.multibody_joints.insert(
        b, c,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    assert j_ab is not None
    assert j_bc is not None

    mb = w.multibody_joints.multibody(j_bc)
    assert mb is not None
    assert mb.num_links == 3
    # Iteration over links yields three.
    links = list(mb)
    assert len(links) == 3


def test_multibody_link_coords_and_joint_rot(ns):
    """A link exposes its joint's generalized coordinates and rotation."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.fixed(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    h = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    assert h is not None
    mb = w.multibody_joints.multibody(h)
    assert mb is not None
    link = mb.get_link(1)
    assert link is not None
    # `coords` is a flat list (6 entries in 3D); a freshly built revolute
    # joint sits at zero angle.
    coords = link.coords
    assert isinstance(coords, list)
    assert len(coords) == 6
    # The joint rotation is a valid unit quaternion at (near) zero angle.
    q = link.joint_rot
    w, x, y, z = q.quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    assert abs(norm - 1.0) < 1e-5
    assert abs(q.angle) < 1e-5


def test_multibody_link_id(ns):
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.fixed().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    h = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0)).build(),
    )
    assert h is not None
    link_id = w.multibody_joints.rigid_body_link(b)
    assert link_id is not None
    # The link id for body b is 1 (root is 0).
    assert link_id.id == 1


def test_multibody_step_preserves_connectivity(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    handles = [
        w.rigid_bodies.insert(ns.RigidBody.fixed().build())
    ]
    for i in range(3):
        h = w.rigid_bodies.insert(
            ns.RigidBody.dynamic(translation=(float(i + 1), 0, 0)).build()
        )
        # Provide mass via a collider.
        w.colliders.insert_with_parent(
            ns.Collider.cuboid(0.4, 0.4, 0.4).density(1.0).build(),
            h, w.rigid_bodies,
        )
        handles.append(h)

    # Chain them.
    for parent, child in zip(handles[:-1], handles[1:]):
        w.multibody_joints.insert(
            parent, child,
            ns.RevoluteJoint.builder(axis=(0, 0, 1))
                .local_anchor1((0.5, 0, 0))
                .local_anchor2((-0.5, 0, 0))
                .build(),
        )

    for _ in range(30):
        w.step()

    # Still alive in the multibody set.
    mb = w.multibody_joints.multibody(
        ns.MultibodyJointHandle.from_raw_parts(*handles[-1].into_raw_parts() if False else (handles[-1].index, handles[-1].generation))
    )
    # The above gymnastics: MultibodyJointHandle == RigidBodyHandle of the
    # last link by construction in the rapier engine. Skip directly.
    h_last = ns.MultibodyJointHandle.from_raw_parts(
        handles[-1].index, handles[-1].generation
    )
    mb = w.multibody_joints.multibody(h_last)
    assert mb is not None
    assert mb.num_links == 4
