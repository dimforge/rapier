"""Port of ``examples3d/utils/character.rs``.

The Python testbed has no egui UI panel and no keyboard polling hook
exposed yet, so the rich interactive controller from the Rust version
collapses into a pair of headless-friendly stubs:

* :class:`CharacterControlMode` — same enum-ish struct as the Rust code so
  examples can keep their ``Kinematic(speed) / Pid(speed)`` switch.
* :func:`update_character` — no-op when no input source is wired. Examples
  that want a moving character can pass their own ``desired_movement_fn``.

Once a keyboard/egui shim lands on the Python ``Testbed`` this module can
grow back the slider UI without touching downstream examples.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import rapier3d as rp


@dataclass
class CharacterControlMode:
    """``Kinematic(speed)`` or ``Pid(speed)`` — selects how the character moves."""

    kind: str  # "kinematic" or "pid"
    speed: float = 0.1

    @classmethod
    def kinematic(cls, speed: float = 0.1) -> "CharacterControlMode":
        return cls("kinematic", speed)

    @classmethod
    def pid(cls, speed: float = 0.1) -> "CharacterControlMode":
        return cls("pid", speed)


def update_character(
    testbed,
    control_mode: CharacterControlMode,
    controller: rp.KinematicCharacterController,
    pid,
    character_handle: rp.RigidBodyHandle,
    desired_movement: Optional[Tuple[float, float, float]] = None,
    desired_movement_fn: Optional[Callable[[object], Tuple[float, float, float]]] = None,
) -> None:
    """Per-step character update.

    Headless-friendly: with no ``desired_movement`` and no ``desired_movement_fn``
    the character simply stands still. Examples that want their character
    to move can pass a callable returning ``(dx, dy, dz)`` per step.
    """
    if desired_movement is None and desired_movement_fn is not None:
        desired_movement = desired_movement_fn(testbed)
    if desired_movement is None:
        return

    bodies = testbed.bodies
    colliders = testbed.colliders
    body = bodies.get(character_handle)
    if body is None:
        return

    dt = testbed._integration_parameters.dt

    if control_mode.kind == "kinematic":
        # Mirrors update_kinematic_controller in Rust: query the colliders
        # owned by the character body, drive ``controller.move_shape``.
        coll_handles = body.colliders
        if not coll_handles:
            return
        char_collider = colliders.get(coll_handles[0])
        if char_collider is None:
            return
        # The Python binding exposes ``move_shape`` on KinematicCharacterController
        # but our minimal testbed has no QueryPipeline plumbed. Best-effort:
        # if either piece is missing, just snap the kinematic body forward.
        new_translation = body.translation + rp.Vec3(*desired_movement)
        # ``set_next_kinematic_translation`` is not exposed in pyi yet; fall
        # back to writing the translation directly. Either keeps the body
        # moving without the dynamic collision response of the Rust path.
        body.translation = new_translation
    else:  # "pid"
        # No PidController in Python yet — leave the body alone.
        pass


__all__ = ["CharacterControlMode", "update_character"]
