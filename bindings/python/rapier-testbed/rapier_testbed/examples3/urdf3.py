"""Port of examples3d/urdf3.rs.

Loads ``assets/3d/T12/urdf/T12.URDF`` twice — once with impulse joints
and once with multibody joints. Resolves the URDF path relative to the
worktree root via ``RAPIER_REPO_ROOT`` (if set) or by walking up from this
file.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import rapier3d as rp
from rapier3d.loaders.urdf import (
    UrdfLoaderOptions,
    UrdfMultibodyOptions,
    UrdfRobot,
)
from .._registry import register

CATEGORY = "Robotics"
NAME = "URDF"


def _find_urdf() -> Path | None:
    env = os.environ.get("RAPIER_REPO_ROOT")
    candidates = []
    if env:
        candidates.append(Path(env) / "assets/3d/T12/urdf/T12.URDF")
    # Walk up from this file looking for the asset.
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        candidates.append(parent / "assets/3d/T12/urdf/T12.URDF")
    for c in candidates:
        if c.is_file():
            return c
    return None


def init_world(testbed) -> None:
    urdf_path = _find_urdf()
    if urdf_path is None:
        raise NotImplementedError(
            "binding gap: assets/3d/T12/urdf/T12.URDF not located. "
            "Set RAPIER_REPO_ROOT to the repository root."
        )

    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    options = UrdfLoaderOptions()
    options.create_colliders_from_visual_shapes = True
    options.create_colliders_from_collision_shapes = False
    options.make_roots_fixed = True
    # Z-up → Y-up.
    options.shift = rp.Isometry3(
        rotation=rp.Rotation3.from_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)
    )

    robot, _ = UrdfRobot.from_file(str(urdf_path), options)
    robot.insert_using_impulse_joints(bodies, colliders, impulse_joints)

    # Second copy via multibody joints, shifted by +10 along X.
    robot2, _ = UrdfRobot.from_file(str(urdf_path), options)
    robot2.append_transform(rp.Isometry3.from_translation(10.0, 0.0, 0.0))
    robot2.insert_using_multibody_joints(
        bodies, colliders, multibody_joints, UrdfMultibodyOptions.DISABLE_SELF_CONTACTS
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((20.0, 20.0, 20.0), (5.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
