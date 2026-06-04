"""MJCF loader (`rapier.loaders.mjcf`).

Wraps `rapier3d-mjcf`, the MuJoCo MJCF (XML) loader. 3D-only, f32-precision.
There is no `rapier.loaders.mjcf.f64` because no `-f64` MJCF crate exists
upstream.

Example::

    from rapier3d.loaders import mjcf

    robot, model = mjcf.MjcfRobot.from_file("scene.xml")
    handles = robot.insert_using_impulse_joints(bodies, colliders, impulse_joints)
"""

from __future__ import annotations

from .._rapier3d import (  # noqa: F401
    ContactFilterMode,
    MjcfBodyHandle,
    MjcfColliderHandle,
    MjcfJointHandle,
    MjcfLoaderOptions,
    MjcfModel,
    MjcfMultibodyOptions,
    MjcfRobot,
    MjcfRobotHandles,
)
from .._rapier3d import MjcfError  # noqa: F401  (error type)

__all__ = [
    "ContactFilterMode",
    "MjcfError",
    "MjcfBodyHandle",
    "MjcfColliderHandle",
    "MjcfJointHandle",
    "MjcfLoaderOptions",
    "MjcfModel",
    "MjcfMultibodyOptions",
    "MjcfRobot",
    "MjcfRobotHandles",
]
