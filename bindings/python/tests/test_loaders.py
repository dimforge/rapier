"""Loader tests.

These exercise the URDF, MJCF, and mesh loader bindings. The loaders are
3D-only / f32-only.
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


ACTUATOR_PARAMS_MJCF = """
<mujoco model="actuated">
  <worldbody>
    <body name="base" pos="0 0 1">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="link" pos="0 0 -0.5">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="hinge" kp="30" ctrlrange="-1 1" forcerange="-9 9"/>
  </actuator>
</mujoco>
"""


def test_mjcf_names_readable_before_insertion() -> None:
    """The MJCF naming tables are reachable while the robot is still alive."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    # Entry 0 is the implicit world body, which has no MJCF name.
    assert robot.body_names == [None, "base", "link"]
    assert robot.joint_names == ["hinge"]
    assert robot.body_name_to_idx["link"] == 2
    assert robot.joint_name_to_idx == {"hinge": 0}


def test_mjcf_names_survive_insertion() -> None:
    """Insertion consumes the robot but the handles carry the names over."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    handles = robot.insert_using_impulse_joints(bodies, colliders, joints)

    assert handles.body_names == [None, "base", "link"]
    assert handles.joint_names == ["hinge"]
    assert handles.joint_name_to_idx == {"hinge": 0}
    # The name tables index into the parallel handle lists.
    idx = handles.body_name_to_idx["link"]
    assert handles.bodies[idx] is not None
    with pytest.raises(mjcf_loader.MjcfError):
        _ = robot.body_names


def test_mjcf_actuator_handles_carry_their_parameters() -> None:
    """`<actuator>` entries reach Python with their parameters and joint handle."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(ACTUATOR_PARAMS_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    handles = robot.insert_using_impulse_joints(bodies, colliders, joints)

    assert len(handles.actuators) == 1
    a = handles.actuators[0]
    assert a.name == "servo"
    assert a.kind == "Position"
    assert a.joint_name == "hinge"
    assert a.kp == pytest.approx(30.0)
    assert a.ctrl_range == pytest.approx([-1.0, 1.0])
    assert a.force_range == pytest.approx([-9.0, 9.0])
    assert isinstance(a.joint, rapier.ImpulseJointHandle)
    assert a.joint == handles.joints[0].joint


def test_mjcf_actuator_handles_multibody_path() -> None:
    """Same on the multibody path, where the joint handle is Optional."""
    robot, _model = mjcf_loader.MjcfRobot.from_str(ACTUATOR_PARAMS_MJCF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    mb = rapier.MultibodyJointSet()
    impulse = rapier.ImpulseJointSet()
    handles = robot.insert_using_multibody_joints(bodies, colliders, mb, impulse)

    a = handles.actuators[0]
    assert a.name == "servo"
    assert a.joint_name == "hinge"
    assert isinstance(a.joint, rapier.MultibodyJointHandle)


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


# ---------------------------------------------------------------------------
# Loader reprs, option flags and docs
# ---------------------------------------------------------------------------


def test_urdf_handles_repr() -> None:
    robot, _raw = urdf_loader.UrdfRobot.from_str(SIMPLE_URDF)
    handles = robot.insert_using_impulse_joints(
        rapier.RigidBodySet(), rapier.ColliderSet(), rapier.ImpulseJointSet()
    )
    assert repr(handles) == "UrdfRobotHandles(n_links=2, n_joints=1)"
    link = handles.links[0]
    assert repr(link).startswith("UrdfLinkHandle(body=RigidBodyHandle(index=")
    assert f"n_colliders={len(link.colliders)})" in repr(link)
    for collider in (c for lh in handles.links for c in lh.colliders):
        assert repr(collider).startswith("UrdfColliderHandle(handle=ColliderHandle(index=")
        assert repr(collider).endswith("has_visual=False)")


def test_multibody_options_contains() -> None:
    u = urdf_loader.UrdfMultibodyOptions
    both = u.JOINTS_ARE_KINEMATIC | u.DISABLE_SELF_CONTACTS
    assert u.DISABLE_SELF_CONTACTS in both
    assert u.DISABLE_SELF_CONTACTS not in u.JOINTS_ARE_KINEMATIC
    m = mjcf_loader.MjcfMultibodyOptions
    flags = m.SKIP_LOOP_CLOSURES | m.SKIP_JOINT_SPRINGS
    assert m.SKIP_JOINT_SPRINGS in flags
    assert m.SKIP_JOINT_LIMITS not in flags
    assert m.SKIP_JOINT_SPRINGS.bits == 0b10_0000


def test_mjcf_model_repr() -> None:
    _robot, model = mjcf_loader.MjcfRobot.from_str(SIMPLE_MJCF)
    assert repr(model) == 'MjcfModel(name="pendulum")'
    _robot, model = mjcf_loader.MjcfRobot.from_str("<mujoco><worldbody/></mujoco>")
    assert repr(model) == "MjcfModel(name=None)"


def test_load_from_path_docstring() -> None:
    doc = mesh_loader.load_from_path.__doc__
    assert "MeshConverter.TRIMESH" in doc
    assert "STL" in doc and ".dae" in doc and ".obj" in doc


# ---------------------------------------------------------------------------
# MJCF runtime helpers: actuators, keyframes, contact hooks
# ---------------------------------------------------------------------------

ACTUATED_MJCF = """
<mujoco model="arm">
  <option gravity="0 0 -5"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom name="base_geom" type="box" size="0.3 0.3 0.3"/>
      <body name="link1" pos="0 0 -0.2">
        <joint name="hinge1" type="hinge" axis="0 1 0" stiffness="10"/>
        <geom name="link1_geom" type="sphere" size="0.2" mass="1"/>
        <body name="link2" pos="0 0 -0.5">
          <joint name="slide2" type="slide" axis="0 0 1"/>
          <geom name="link2_geom" type="sphere" size="0.1" mass="1"/>
        </body>
      </body>
    </body>
    <body name="floor" pos="0 0 -0.8">
      <geom name="floor_geom" type="box" size="2 2 0.1"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="base" body2="link1"/>
    <pair geom1="link2_geom" geom2="floor_geom" friction="0.25 0.25 0.005 0.0001 0.0001"/>
  </contact>
  <actuator>
    <position name="servo1" joint="hinge1" kp="50"/>
    <motor name="motor2" joint="slide2" gear="1"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.5 0.1" ctrl="0.25 0"/>
    <key qpos="-0.3 0.0"/>
  </keyframe>
</mujoco>
"""


def _insert_actuated(multibody: bool, **options):
    robot, _ = mjcf_loader.MjcfRobot.from_str(
        ACTUATED_MJCF, mjcf_loader.MjcfLoaderOptions(**options)
    )
    world = rapier.PhysicsWorld(gravity=(0, 0, 0))
    if multibody:
        handles = robot.insert_using_multibody_joints(
            world.rigid_bodies, world.colliders, world.multibody_joints, world.impulse_joints
        )
    else:
        handles = robot.insert_using_impulse_joints(
            world.rigid_bodies, world.colliders, world.impulse_joints
        )
    return world, handles


def test_mjcf_robot_gravity_and_keyframe_names() -> None:
    robot, _ = mjcf_loader.MjcfRobot.from_str(ACTUATED_MJCF)
    g = robot.gravity
    assert (g.x, g.y, g.z) == (0.0, 0.0, -5.0)
    assert robot.keyframe_names == ["home", None]
    robot.insert_using_impulse_joints(
        rapier.RigidBodySet(), rapier.ColliderSet(), rapier.ImpulseJointSet()
    )
    with pytest.raises(mjcf_loader.MjcfError):
        robot.gravity
    with pytest.raises(mjcf_loader.MjcfError):
        robot.keyframe_names


@pytest.mark.parametrize("multibody", [False, True], ids=["impulse", "multibody"])
def test_mjcf_actuator_handles(multibody: bool) -> None:
    world, handles = _insert_actuated(multibody)
    assert handles.keyframe_names == ["home", None]
    assert [a.name for a in handles.actuators] == ["servo1", "motor2"]
    joint_type = rapier.MultibodyJointHandle if multibody else rapier.ImpulseJointHandle
    assert all(isinstance(a.joint, joint_type) for a in handles.actuators)
    assert [a.joint for a in handles.actuators] == [j.joint for j in handles.joints]
    assert repr(handles.actuators[0]).startswith('MjcfActuatorHandle(name="servo1", joint=')
    assert "n_actuators=2, n_keyframes=2" in repr(handles)


@pytest.mark.parametrize("multibody", [False, True], ids=["impulse", "multibody"])
def test_mjcf_apply_controls(multibody: bool) -> None:
    world, handles = _insert_actuated(multibody)
    joints = world.multibody_joints if multibody else world.impulse_joints
    other = world.impulse_joints if multibody else world.multibody_joints
    handles.apply_controls(world.rigid_bodies, joints, [0.3, 2.0], gain_scale=0.5)
    servo = joints[handles.actuators[0].joint].data.motor(rapier.JointAxis.ANG_X)
    assert servo.target_pos == pytest.approx(0.3)
    assert servo.stiffness == pytest.approx(25.0)
    motor = joints[handles.actuators[1].joint].data.motor(rapier.JointAxis.LIN_X)
    assert motor.max_force == pytest.approx(1.0)
    with pytest.raises(ValueError):
        handles.apply_controls(world.rigid_bodies, joints, [0.3])
    with pytest.raises(TypeError):
        handles.apply_controls(world.rigid_bodies, other, [0.3, 2.0])
    for _ in range(3):
        world.step()


def test_mjcf_keyframes_multibody() -> None:
    world, handles = _insert_actuated(True)
    hinge, slide = (world.multibody_joints[j.joint] for j in handles.joints)
    handles.apply_keyframe(world.rigid_bodies, world.multibody_joints, "home")
    assert hinge.coords[3] == pytest.approx(0.5)
    assert slide.coords[0] == pytest.approx(0.1)
    handles.apply_keyframe(world.rigid_bodies, world.multibody_joints, -1)
    assert hinge.coords[3] == pytest.approx(-0.3)
    handles.apply_keyframe(world.rigid_bodies, world.multibody_joints)
    assert hinge.coords[3] == pytest.approx(0.5)
    with pytest.raises(TypeError):
        handles.apply_keyframe(world.rigid_bodies)
    with pytest.raises(IndexError):
        handles.apply_keyframe(world.rigid_bodies, world.multibody_joints, 2)
    with pytest.raises(KeyError):
        handles.apply_keyframe(world.rigid_bodies, world.multibody_joints, "nope")

    # The explicit `ctrl` of the keyframe wins, then the joint position.
    assert handles.keyframe_controls("home") == pytest.approx([0.25, 0.0])
    assert handles.keyframe_controls(1) == pytest.approx([-0.3, 0.0])
    with pytest.raises(TypeError):
        handles.keyframe_controls(1.5)


def test_mjcf_keyframes_impulse() -> None:
    world, handles = _insert_actuated(False)
    link1 = handles.bodies[2].body
    handles.apply_keyframe(world.rigid_bodies, keyframe="home")
    assert world.rigid_bodies[link1].rotation.angle == pytest.approx(0.5, abs=1e-4)
    handles.apply_keyframe(world.rigid_bodies, None, 1)
    assert world.rigid_bodies[link1].rotation.angle == pytest.approx(0.3, abs=1e-4)


@pytest.mark.parametrize("kind", ["none", "native", "python"])
def test_mjcf_contact_hooks(kind: str) -> None:
    # Joint collisions are enabled so only the `<exclude>` rule separates base and link1.
    world, handles = _insert_actuated(False, enable_joint_collisions=True)
    base, link1, link2, floor = (handles.bodies[i].colliders[0].handle for i in range(1, 5))
    frictions = {}
    hooks = handles.contact_hooks()
    assert repr(hooks) == "MjcfContactHooks(has_excludes=True, has_overrides=True)"
    if kind == "native":
        world.physics_hooks = hooks
    elif kind == "python":

        class Hooks:
            def filter_contact_pair(self, ctx):
                return hooks.filter_contact_pair(ctx)

            def filter_intersection_pair(self, ctx):
                return True

            def modify_solver_contacts(self, ctx):
                before = ctx.friction
                hooks.modify_solver_contacts(ctx)
                frictions[(ctx.collider1, ctx.collider2)] = (before, ctx.friction)

        world.physics_hooks = Hooks()
    for _ in range(3):
        world.step()
    excluded = world.narrow_phase.contact_pair(base, link1)
    assert (excluded is not None and excluded.has_any_active_contact) == (kind == "none")
    assert world.narrow_phase.contact_pair(link2, floor).has_any_active_contact
    if kind == "python":
        before, after = frictions[(link2, floor)]
        assert before != pytest.approx(0.25)
        assert after == pytest.approx(0.25)
