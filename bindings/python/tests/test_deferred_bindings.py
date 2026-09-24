"""Regression tests for the second batch of newly-exposed Rust API.

Covers the items deferred from the first coverage pass:

* ``RevoluteJoint.angle`` (3D).
* extra shape constructors: ``round_triangle``, ``round_convex_hull``,
  ``voxels`` / ``voxels_from_points``, ``convex_mesh`` (3D),
  ``converted_trimesh``, and ``MeshConverter.CONVEX_DECOMPOSITION``.
* URDF / MJCF ``collider_blueprint`` / ``rigid_body_blueprint`` and the
  MJCF ``contact_filter_mode`` (``ContactFilterMode``).
* multibody accessors: ``damping``, ``generalized_velocity`` (+ setters),
  ``generalized_acceleration``, ``joint_velocity``, ``body_jacobian``,
  ``kinematic_branch``, ``forward_kinematics``, ``update_rigid_bodies``,
  and ``MultibodyJointSet.joint_between``.
"""

from __future__ import annotations

import math

import pytest

import rapier3d as dim3
from rapier3d.loaders import mjcf as mjcf_loader
from rapier3d.loaders import urdf as urdf_loader


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


# --------------------------------------------------------------------------
# RevoluteJoint.angle
# --------------------------------------------------------------------------


def test_revolute_angle_3d(ns):
    j = ns.RevoluteJoint.builder((0, 0, 1)).build()
    rot1 = ns.Rotation3.identity()
    rot2 = ns.Rotation3.from_axis_angle((0, 0, 1), math.radians(30))
    assert j.angle(rot1, rot2) == pytest.approx(math.radians(30), abs=1e-4)


# --------------------------------------------------------------------------
# shape constructors
# --------------------------------------------------------------------------

_TET = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
_TET_IDX = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]


def test_round_triangle(ns):
    s = ns.SharedShape.round_triangle((0, 0, 0), (1, 0, 0), (0, 1, 0), 0.1)
    assert s.shape_type == ns.ShapeType.ROUND_TRIANGLE


def test_round_convex_hull_3d(ns):
    s = ns.SharedShape.round_convex_hull(_TET, 0.05)
    assert s.shape_type == ns.ShapeType.ROUND_CONVEX_POLYHEDRON


def test_convex_mesh_3d(ns):
    s = ns.SharedShape.convex_mesh(_TET, _TET_IDX)
    assert s.shape_type == ns.ShapeType.CONVEX_POLYHEDRON
    s2 = ns.SharedShape.round_convex_mesh(_TET, _TET_IDX, 0.05)
    assert s2.shape_type == ns.ShapeType.ROUND_CONVEX_POLYHEDRON


def test_voxels_3d(ns):
    s = ns.SharedShape.voxels((0.5, 0.5, 0.5), [(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    assert s.shape_type == ns.ShapeType.VOXELS
    s2 = ns.SharedShape.voxels_from_points((0.5, 0.5, 0.5), [(0, 0, 0), (0.9, 0, 0)])
    assert s2.shape_type == ns.ShapeType.VOXELS


def test_collider_shape_ctors_3d(ns):
    ns.Collider.round_triangle((0, 0, 0), (1, 0, 0), (0, 1, 0), 0.1).build()
    ns.Collider.round_convex_hull(_TET, 0.05).build()
    ns.Collider.convex_mesh(_TET, _TET_IDX).build()
    ns.Collider.round_convex_mesh(_TET, _TET_IDX, 0.05).build()
    ns.Collider.voxels((0.5, 0.5, 0.5), [(0, 0, 0), (1, 0, 0)]).build()
    ns.Collider.voxels_from_points((0.5, 0.5, 0.5), [(0, 0, 0), (0.9, 0, 0)]).build()


def test_converted_trimesh(ns):
    coll = ns.Collider.converted_trimesh(
        _TET, _TET_IDX, ns.MeshConverter.CONVEX_HULL
    ).build()
    assert coll.shape.shape_type == ns.ShapeType.CONVEX_POLYHEDRON
    # OBB conversion yields a cuboid.
    coll2 = ns.Collider.converted_trimesh(_TET, _TET_IDX, ns.MeshConverter.OBB).build()
    assert coll2.shape.shape_type == ns.ShapeType.CUBOID


def test_mesh_converter_convex_decomposition_is_3d_only():
    assert hasattr(dim3.MeshConverter, "CONVEX_DECOMPOSITION")


# --------------------------------------------------------------------------
# loader blueprints
# --------------------------------------------------------------------------


def test_urdf_loader_blueprints():
    opts = urdf_loader.UrdfLoaderOptions(
        collider_blueprint=dim3.Collider.ball(0.5),
        rigid_body_blueprint=dim3.RigidBody.fixed(),
    )
    assert opts.collider_blueprint is not None
    assert opts.rigid_body_blueprint is not None
    # default is None
    assert urdf_loader.UrdfLoaderOptions().collider_blueprint is None


def test_mjcf_loader_blueprints_and_contact_filter():
    cfm = mjcf_loader.ContactFilterMode
    opts = mjcf_loader.MjcfLoaderOptions(
        contact_filter_mode=cfm.Asymmetric,
        collider_blueprint=dim3.Collider.ball(1.0),
        rigid_body_blueprint=dim3.RigidBody.fixed(),
    )
    assert opts.contact_filter_mode == cfm.Asymmetric
    assert opts.collider_blueprint is not None
    assert opts.rigid_body_blueprint is not None
    # default is Symmetric
    assert mjcf_loader.MjcfLoaderOptions().contact_filter_mode == cfm.Symmetric
    # setter
    opts.contact_filter_mode = cfm.Symmetric
    assert opts.contact_filter_mode == cfm.Symmetric


# --------------------------------------------------------------------------
# multibody accessors
# --------------------------------------------------------------------------


def _two_link_world(ns):
    w = ns.PhysicsWorld()
    root = w.rigid_bodies.insert(ns.RigidBody.fixed().build())
    child = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    handle = w.multibody_joints.insert(
        root, child, ns.RevoluteJoint.builder((0, 0, 1)).build(), True
    )
    return w, root, child, handle


def test_multibody_state_accessors(ns):
    w, _root, _child, handle = _two_link_world(ns)
    mb = w.multibody_joints.multibody(handle)
    n = mb.ndofs
    assert len(mb.damping()) == n
    assert len(mb.generalized_velocity()) == n
    assert len(mb.generalized_acceleration()) == n

    mb.set_generalized_velocity([0.25] * n)
    assert mb.generalized_velocity() == pytest.approx([0.25] * n)
    mb.set_damping([0.0] * n)
    assert mb.damping() == pytest.approx([0.0] * n)
    with pytest.raises(ValueError):
        mb.set_generalized_velocity([0.0] * (n + 1))

    assert mb.kinematic_branch(1) == [0, 1]
    link = mb.get_link(1)
    assert link is not None
    assert isinstance(mb.joint_velocity(link), list)


def test_multibody_body_jacobian_shape(ns):
    w, _root, _child, handle = _two_link_world(ns)
    w.step()  # populate jacobians
    mb = w.multibody_joints.multibody(handle)
    jac = mb.body_jacobian(1)
    rows = 6
    assert len(jac) == rows
    assert all(len(r) == mb.ndofs for r in jac)


def test_multibody_forward_kinematics_and_update(ns):
    w, _root, _child, handle = _two_link_world(ns)
    mb = w.multibody_joints.multibody(handle)
    # A full FK -> writeback sequence runs without error on the retrieved copy.
    mb.forward_kinematics(w.rigid_bodies, True)
    mb.update_rigid_bodies(w.rigid_bodies, False)


def test_multibody_joint_between(ns):
    w, root, child, handle = _two_link_world(ns)
    found = w.multibody_joints.joint_between(root, child)
    assert found is not None
    fh, mb, link = found
    assert fh == handle
    assert mb.ndofs >= 1
    # unrelated bodies → None
    other = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    assert w.multibody_joints.joint_between(root, other) is None
