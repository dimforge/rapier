"""URDF loader (`rapier.loaders.urdf`).

Wraps `rapier3d-urdf`. 3D-only, f32-precision. There is no `rapier.loaders.urdf.f64`
because no `-f64` URDF crate exists upstream.
"""

from __future__ import annotations

from .._rapier3d import (  # noqa: F401
    Robot,
    UrdfColliderHandle,
    UrdfJoint,
    UrdfJointHandle,
    UrdfLink,
    UrdfLinkHandle,
    UrdfLoaderOptions,
    UrdfMultibodyOptions,
    UrdfRobot,
    UrdfRobotHandles,
)
from .._rapier3d import UrdfError  # noqa: F401  (error type)

__all__ = [
    "Robot",
    "UrdfError",
    "UrdfColliderHandle",
    "UrdfJoint",
    "UrdfJointHandle",
    "UrdfLink",
    "UrdfLinkHandle",
    "UrdfLoaderOptions",
    "UrdfMultibodyOptions",
    "UrdfRobot",
    "UrdfRobotHandles",
]
