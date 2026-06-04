"""Port of examples3d/mjcf3.rs.

Loads ``assets/3d/agility_cassie/scene.xml`` twice — once with impulse joints
and once (offset) with multibody joints. Resolves the asset path relative to
the worktree root via ``RAPIER_REPO_ROOT`` (if set) or by walking up from this
file.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import rapier3d as rp
from rapier3d.loaders.mjcf import (
    MjcfLoaderOptions,
    MjcfMultibodyOptions,
    MjcfRobot,
)

from .._registry import register

CATEGORY = "Robotics"
NAME = "MJCF"

_ASSET = "assets/3d/agility_cassie/scene.xml"


def _find_scene() -> Path | None:
    candidates = []
    env = os.environ.get("RAPIER_REPO_ROOT")
    if env:
        candidates.append(Path(env) / _ASSET)
    # Walk up from this file looking for the asset.
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        candidates.append(parent / _ASSET)
    for c in candidates:
        if c.is_file():
            return c
    return None


def init_world(testbed) -> None:
    scene = _find_scene()
    if scene is None:
        raise NotImplementedError(
            f"asset not found: {_ASSET}. Set RAPIER_REPO_ROOT to the "
            "repository root."
        )

    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # MJCF is Z-up; the testbed is Y-up. Rotate -pi/2 about X to stand it up.
    z_to_y = rp.Rotation3.from_axis_angle((1.0, 0.0, 0.0), -math.pi / 2.0)

    opts = MjcfLoaderOptions()
    opts.make_roots_fixed = True
    opts.shift = rp.Isometry3(rotation=z_to_y)

    # First copy: inserted with impulse joints.
    robot, _ = MjcfRobot.from_file(str(scene), opts)
    robot.insert_using_impulse_joints(bodies, colliders, impulse_joints)

    # Second copy: offset by +1 along Z, inserted with multibody joints. The
    # inserts consume the robot, so we load a second time rather than cloning
    # (mjcf3.rs clones); the offset matches mjcf3.rs's append_transform.
    robot2, _ = MjcfRobot.from_file(str(scene), opts)
    robot2.append_transform(rp.Isometry3.from_translation(0.0, 0.0, 1.0))
    robot2.insert_using_multibody_joints(
        bodies,
        colliders,
        multibody_joints,
        impulse_joints,
        MjcfMultibodyOptions.SKIP_LOOP_CLOSURES
        | MjcfMultibodyOptions.DISABLE_SELF_CONTACTS,
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((2.0, 2.0, 2.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
