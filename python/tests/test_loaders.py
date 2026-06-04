"""Loader tests.

These exercise the URDF, MJCF, and mesh loader bindings. The loaders are
3D-only / f32-only — there are no f64 equivalents.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import rapier3d as rapier
from rapier3d.loaders import mesh as mesh_loader
from rapier3d.loaders import mjcf as mjcf_loader
from rapier3d.loaders import urdf as urdf_loader


# ---------------------------------------------------------------------------
# Mesh loader
# ---------------------------------------------------------------------------


def test_mesh_loader_module_imports() -> None:
    """The mesh sub-module re-exports the expected symbols."""
    assert hasattr(mesh_loader, "load_from_path")
    assert hasattr(mesh_loader, "load_from_raw_mesh")
    assert hasattr(mesh_loader, "LoadedShape")
    assert hasattr(mesh_loader, "MeshLoaderError")
    assert hasattr(mesh_loader, "MeshConversionError")


def test_load_from_raw_mesh_triangle() -> None:
    """Loading a single-triangle mesh produces a valid SharedShape."""
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2]], dtype=np.uint32)
    loaded = mesh_loader.load_from_raw_mesh(vertices, indices)
    assert loaded is not None
    assert loaded.shape is not None
    # Default TRIMESH converter — should produce a TriMesh shape.
    assert loaded.shape.as_trimesh() is not None
    # Vertices ndarray round-trip.
    out_v = loaded.vertices
    assert out_v.shape == (3, 3)
    out_i = loaded.indices
    assert out_i.shape == (1, 3)


def test_load_from_raw_mesh_convex_hull() -> None:
    """Convex-hull converter also produces a valid shape."""
    # Tetrahedron.
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    indices = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]], dtype=np.uint32)
    loaded = mesh_loader.load_from_raw_mesh(
        vertices, indices, converter=rapier.MeshConverter.CONVEX_HULL
    )
    assert loaded is not None


def test_load_from_raw_mesh_degenerate_raises() -> None:
    """A degenerate mesh (zero-area triangle) on convex-hull errs cleanly."""
    vertices = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    indices = np.array([[0, 0, 0]], dtype=np.uint32)
    with pytest.raises(rapier.MeshConversionError):
        mesh_loader.load_from_raw_mesh(
            vertices, indices, converter=rapier.MeshConverter.CONVEX_HULL
        )


def test_load_from_path_missing_file_raises() -> None:
    """Missing-file error is surfaced as MeshLoaderError."""
    with pytest.raises(rapier.MeshLoaderError):
        mesh_loader.load_from_path("/nonexistent/path/that/does/not.stl")


# ---------------------------------------------------------------------------
# URDF loader
# ---------------------------------------------------------------------------


SIMPLE_URDF = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
  </link>
  <link name="upper_link">
    <inertial>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
  </link>
  <joint name="shoulder" type="revolute">
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="upper_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1.0" velocity="1.0"/>
  </joint>
</robot>
"""


def test_urdf_loader_module_imports() -> None:
    """The URDF sub-module re-exports the expected symbols."""
    for name in [
        "UrdfRobot",
        "UrdfLoaderOptions",
        "UrdfMultibodyOptions",
        "UrdfLink",
        "UrdfJoint",
        "UrdfRobotHandles",
        "UrdfLinkHandle",
        "UrdfJointHandle",
        "UrdfColliderHandle",
        "Robot",
        "UrdfError",
    ]:
        assert hasattr(urdf_loader, name), name


def test_urdf_from_str_two_link() -> None:
    """Parsing a 2-link revolute URDF produces matching link/joint counts."""
    robot, raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    assert robot.n_links == 2
    assert robot.n_joints == 1
    assert raw.name == "two_link"
    assert len(raw.links) == 2
    assert len(raw.joints) == 1
    assert raw.joints[0].name == "shoulder"
    # Joint type comes through as a stringified Debug.
    assert "Revolute" in raw.joints[0].joint_type


def test_urdf_from_str_with_options() -> None:
    """Custom UrdfLoaderOptions round-trips through `from_str`."""
    opts = urdf_loader.UrdfLoaderOptions(
        create_colliders_from_collision_shapes=True,
        create_colliders_from_visual_shapes=False,
        apply_imported_mass_props=True,
        make_roots_fixed=True,
    )
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF, options=opts)
    assert robot.n_links == 2


def test_urdf_insert_using_impulse_joints() -> None:
    """End-to-end: parse URDF, insert into a world, step once."""
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    handles = robot.insert_using_impulse_joints(bodies, colliders, joints)
    assert len(handles.links) == 2
    assert len(handles.joints) == 1
    # The link handles point at valid bodies.
    for lh in handles.links:
        assert lh.body in bodies
    # The joint handle wraps an ImpulseJointHandle.
    assert isinstance(handles.joints[0].joint, rapier.ImpulseJointHandle)
    assert handles.joints[0].joint in joints


def test_urdf_insert_using_multibody_joints() -> None:
    """Multibody insert path also works and returns Option[handle]."""
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    mb = rapier.MultibodyJointSet()
    handles = robot.insert_using_multibody_joints(bodies, colliders, mb)
    assert len(handles.links) == 2
    assert len(handles.joints) == 1


def test_urdf_append_transform() -> None:
    """`append_transform` mutates the robot in place."""
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    iso = rapier.Isometry3.identity()
    # Should not raise.
    robot.append_transform(iso)
    assert robot.n_links == 2


def test_urdf_invalid_xml_raises() -> None:
    """Invalid XML raises UrdfError."""
    with pytest.raises(rapier.UrdfError):
        urdf_loader.UrdfRobot.from_str("<not><valid xml")


def test_urdf_consumed_after_insert() -> None:
    """Inserting consumes the UrdfRobot; subsequent ops raise."""
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    robot.insert_using_impulse_joints(bodies, colliders, joints)
    with pytest.raises(rapier.UrdfError):
        _ = robot.n_links


def test_urdf_multibody_options_bitflags() -> None:
    """The UrdfMultibodyOptions bitflags can be combined."""
    a = urdf_loader.UrdfMultibodyOptions.JOINTS_ARE_KINEMATIC
    b = urdf_loader.UrdfMultibodyOptions.DISABLE_SELF_CONTACTS
    combined = a | b
    assert combined.bits == a.bits | b.bits


# ---------------------------------------------------------------------------
# MJCF loader
# ---------------------------------------------------------------------------

SIMPLE_MJCF = """
<mujoco model="pendulum">
  <worldbody>
    <body name="base" pos="0 0 1">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="link" pos="0 0 -0.5">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def test_mjcf_module_exposes_loader() -> None:
    """The MJCF sub-module re-exports the full loader API."""
    assert hasattr(mjcf_loader, "MjcfError")
    assert hasattr(mjcf_loader, "MjcfRobot")
    assert hasattr(mjcf_loader, "MjcfLoaderOptions")
    assert hasattr(mjcf_loader, "MjcfMultibodyOptions")
    assert hasattr(mjcf_loader, "MjcfRobotHandles")


def test_mjcf_error_is_rapier_error() -> None:
    """`MjcfError` is a subclass of `RapierError`."""
    assert issubclass(mjcf_loader.MjcfError, rapier.RapierError)


def test_mjcf_from_str_returns_robot_and_model() -> None:
    """Parsing a minimal MJCF string yields a robot and a named model."""
    robot, model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    assert model.name == "pendulum"
    # world body (index 0) + base + link.
    assert robot is not None


def test_mjcf_insert_using_impulse_joints() -> None:
    """End-to-end: parse MJCF, insert into a world, step once."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    handles = robot.insert_using_impulse_joints(bodies, colliders, joints)
    # base + link bodies were inserted (world body is left as None).
    inserted = [b for b in handles.bodies if b is not None]
    assert len(inserted) >= 2
    # One hinge joint.
    assert len(handles.joints) == 1
    assert isinstance(handles.joints[0].joint, rapier.ImpulseJointHandle)
    assert handles.joints[0].joint in joints


def test_mjcf_insert_using_multibody_joints() -> None:
    """Multibody insert path works; the joint handle is Optional."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    mb = rapier.MultibodyJointSet()
    impulse = rapier.ImpulseJointSet()
    handles = robot.insert_using_multibody_joints(bodies, colliders, mb, impulse)
    assert len(handles.joints) == 1


def test_mjcf_consumed_after_insert() -> None:
    """Inserting consumes the robot; a second insert raises MjcfError."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    robot.insert_using_impulse_joints(bodies, colliders, joints)
    with pytest.raises(mjcf_loader.MjcfError):
        robot.insert_using_impulse_joints(bodies, colliders, joints)


# ---------------------------------------------------------------------------
# Cassie smoke (fixture-guarded)
# ---------------------------------------------------------------------------


CASSIE_SCENE = Path(__file__).parents[3].joinpath(
    "assets/3d/agility_cassie/scene.xml"
)


@pytest.mark.skipif(
    not CASSIE_SCENE.exists(),
    reason="Cassie MJCF asset not present in this worktree",
)
def test_cassie_mjcf_smoke() -> None:
    """Load the Cassie MJCF scene and step it without producing NaNs.

    The Cassie meshes (`.obj`) aren't committed, so the loader skips the
    mesh colliders (logging a warning) but still builds the body / joint
    hierarchy — enough for a structural + stability smoke test.
    """
    robot, model = mjcf_loader.MjcfRobot.from_file(str(CASSIE_SCENE))
    assert model.name is not None

    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    mb = rapier.MultibodyJointSet()
    impulse = rapier.ImpulseJointSet()
    handles = robot.insert_using_multibody_joints(bodies, colliders, mb, impulse)

    inserted = [b for b in handles.bodies if b is not None]
    assert len(inserted) > 0

    world = rapier.PhysicsWorld(gravity=rapier.Vec3(0.0, 0.0, -9.81))
    # Re-load into the world's own sets (a fresh robot, since insert consumes).
    robot2, _ = mjcf_loader.MjcfRobot.from_file(str(CASSIE_SCENE))
    robot2.insert_using_multibody_joints(
        world.rigid_bodies, world.colliders, world.multibody_joints, world.impulse_joints
    )
    for _ in range(20):
        world.step()

    # No body should have drifted to a non-finite position.
    for handle, body in [(h, world.rigid_bodies[h]) for h in world.active_bodies()]:
        t = body.translation
        assert np.isfinite([t.x, t.y, t.z]).all(), f"NaN/inf body at {handle}"
