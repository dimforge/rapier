"""Load a 2-link revolute URDF from an inline string.

Demonstrates `rapier.loaders.urdf.UrdfRobot.from_str` + `insert_using_impulse_joints`.
URDF loading is 3D / f32 only.

Run::

    python python/examples/urdf/load_simple.py
"""

from __future__ import annotations

import rapier3d as rapier
from rapier3d.loaders import urdf as urdf_loader


URDF = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <collision>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
    </collision>
  </link>
  <link name="upper_link">
    <inertial>
      <origin xyz="0 0 0.25"/>
      <mass value="0.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry><box size="0.2 0.2 0.5"/></geometry>
    </collision>
  </link>
  <joint name="shoulder" type="revolute">
    <origin xyz="0 0 0.25"/>
    <parent link="base_link"/>
    <child link="upper_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1.0" velocity="1.0"/>
  </joint>
</robot>
"""


def main() -> None:
    robot, raw = urdf_loader.UrdfRobot.from_str(URDF)
    bodies = rapier.RigidBodySet()
    colliders = rapier.ColliderSet()
    joints = rapier.ImpulseJointSet()
    handles = robot.insert_using_impulse_joints(bodies, colliders, joints)
    print(
        f"urdf: name={raw.name} links={len(handles.links)} joints={len(handles.joints)}"
    )


if __name__ == "__main__":
    main()
