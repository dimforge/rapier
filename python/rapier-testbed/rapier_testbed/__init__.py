"""Panda3D-based mini-testbed for the Rapier Python bindings.

This package mirrors the Rust ``src_testbed`` crate in spirit: each example
implements ``init_world(testbed: Testbed) -> None``, populates a fresh set
of ``RigidBodySet`` / ``ColliderSet`` / ``ImpulseJointSet`` /
``MultibodyJointSet``, and hands them to the testbed via
:meth:`Testbed.set_world`. The testbed owns the solver state and steps it
each frame, feeding the :class:`DebugRenderPipeline` output into Panda3D
:class:`LineSegs` geometry.

Quick start::

    from rapier_testbed import Testbed, register

    def init_world(testbed: Testbed) -> None:
        import rapier as rp
        bodies = rp.RigidBodySet()
        colliders = rp.ColliderSet()
        impulse_joints = rp.ImpulseJointSet()
        multibody_joints = rp.MultibodyJointSet()
        # ... build the scene ...
        testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
        testbed.look_at((20, -40, 25), (0, 0, 0))

    register("Demo", "Hello", init_world)

    # Then either:
    #   python -m rapier_testbed                  # picker menu
    #   python -m rapier_testbed.examples3.<name> # direct launch

Keyboard controls (when the window is open):

* ``space`` — pause / resume
* ``arrow_right`` — single-step
* ``r`` — reset (re-run the current example's ``init_world``)
* ``tab`` — cycle to the next registered example
* digits ``1``-``9`` — jump to example #N
* ``escape`` — quit

CI / headless mode: set ``PANDA_NO_WINDOW=1`` to run a fixed number of
physics steps without opening a window. Useful for regression tests.
"""

from __future__ import annotations

from ._picker import run
from ._registry import EXAMPLES, Example, register, find, by_label
from ._testbed import Testbed, CallbackFn


def _autoload_bundled_examples() -> None:
    """Import every bundled example module so ``register(...)`` runs.

    Done lazily (not at top-level) so that ``python -m rapier_testbed.<...>``
    doesn't double-import the target module via the package's eager imports.
    """
    import importlib

    # New examples ported from ``examples3d/`` / ``examples2d/`` should
    # extend this list.
    _bundled = (
        "rapier_testbed.examples3.primitives3",
        # ── 3D examples (first half) ────────────────────────────────
        "rapier_testbed.examples3.ccd3",
        "rapier_testbed.examples3.character_controller3",
        "rapier_testbed.examples3.collision_groups3",
        "rapier_testbed.examples3.compound3",
        "rapier_testbed.examples3.convex_decomposition3",
        "rapier_testbed.examples3.convex_polyhedron3",
        "rapier_testbed.examples3.damping3",
        # ── 3D examples (second half) ───────────────────────────────
        "rapier_testbed.examples3.domino3",
        "rapier_testbed.examples3.dynamic_trimesh3",
        "rapier_testbed.examples3.fountain3",
        "rapier_testbed.examples3.gyroscopic3",
        "rapier_testbed.examples3.harness_capsules3",
        "rapier_testbed.examples3.heightfield3",
        "rapier_testbed.examples3.inverse_kinematics3",
        "rapier_testbed.examples3.joint_motor_position3",
        "rapier_testbed.examples3.joints3",
        "rapier_testbed.examples3.keva3",
        "rapier_testbed.examples3.locked_rotations3",
        "rapier_testbed.examples3.newton_cradle3",
        "rapier_testbed.examples3.one_way_platforms3",
        "rapier_testbed.examples3.platform3",
        "rapier_testbed.examples3.restitution3",
        "rapier_testbed.examples3.rope_joints3",
        "rapier_testbed.examples3.sensor3",
        "rapier_testbed.examples3.mjcf3",
        "rapier_testbed.examples3.spring_joints3",
        "rapier_testbed.examples3.trimesh3",
        "rapier_testbed.examples3.urdf3",
        "rapier_testbed.examples3.vehicle_controller3",
        "rapier_testbed.examples3.vehicle_joints3",
        "rapier_testbed.examples3.voxels3",
        # ── 3D stress tests ─────────────────────────────────────────
        "rapier_testbed.examples3.stress_tests.balls3",
        "rapier_testbed.examples3.stress_tests.boxes3",
        "rapier_testbed.examples3.stress_tests.capsules3",
        "rapier_testbed.examples3.stress_tests.ccd3",
        "rapier_testbed.examples3.stress_tests.compound3",
        "rapier_testbed.examples3.stress_tests.convex_polyhedron3",
        "rapier_testbed.examples3.stress_tests.heightfield3",
        "rapier_testbed.examples3.stress_tests.joint_ball3",
        "rapier_testbed.examples3.stress_tests.joint_fixed3",
        "rapier_testbed.examples3.stress_tests.joint_prismatic3",
        "rapier_testbed.examples3.stress_tests.joint_revolute3",
        "rapier_testbed.examples3.stress_tests.keva3",
        "rapier_testbed.examples3.stress_tests.many_kinematics3",
        "rapier_testbed.examples3.stress_tests.many_pyramids3",
        "rapier_testbed.examples3.stress_tests.many_sleep3",
        "rapier_testbed.examples3.stress_tests.many_static3",
        "rapier_testbed.examples3.stress_tests.pyramid3",
        "rapier_testbed.examples3.stress_tests.ray_cast3",
        "rapier_testbed.examples3.stress_tests.stacks3",
        "rapier_testbed.examples3.stress_tests.trimesh3",
        # ── 2D examples ─────────────────────────────────────────────
        "rapier_testbed.examples2.add_remove2",
        "rapier_testbed.examples2.drum2",
        "rapier_testbed.examples2.inv_pyramid2",
        "rapier_testbed.examples2.platform2",
        "rapier_testbed.examples2.pyramid2",
        "rapier_testbed.examples2.sensor2",
        "rapier_testbed.examples2.convex_polygons2",
        "rapier_testbed.examples2.heightfield2",
        "rapier_testbed.examples2.polyline2",
        "rapier_testbed.examples2.trimesh2",
        "rapier_testbed.examples2.voxels2",
        "rapier_testbed.examples2.collision_groups2",
        "rapier_testbed.examples2.one_way_platforms2",
        "rapier_testbed.examples2.locked_rotations2",
        "rapier_testbed.examples2.restitution2",
        "rapier_testbed.examples2.damping2",
        "rapier_testbed.examples2.ccd2",
        "rapier_testbed.examples2.joints2",
        "rapier_testbed.examples2.rope_joints2",
        "rapier_testbed.examples2.pin_slot_joint2",
        "rapier_testbed.examples2.joint_motor_position2",
        "rapier_testbed.examples2.inverse_kinematics2",
        "rapier_testbed.examples2.character_controller2",
        "rapier_testbed.examples2.s2d_high_mass_ratio_1",
        "rapier_testbed.examples2.s2d_high_mass_ratio_2",
        "rapier_testbed.examples2.s2d_high_mass_ratio_3",
        "rapier_testbed.examples2.s2d_confined",
        "rapier_testbed.examples2.s2d_pyramid",
        "rapier_testbed.examples2.s2d_card_house",
        "rapier_testbed.examples2.s2d_arch",
        "rapier_testbed.examples2.s2d_bridge",
        "rapier_testbed.examples2.s2d_ball_and_chain",
        "rapier_testbed.examples2.s2d_joint_grid",
        "rapier_testbed.examples2.s2d_far_pyramid",
        "rapier_testbed.examples2.stress_tests.balls2",
        "rapier_testbed.examples2.stress_tests.boxes2",
        "rapier_testbed.examples2.stress_tests.capsules2",
        "rapier_testbed.examples2.stress_tests.convex_polygons2",
        "rapier_testbed.examples2.stress_tests.heightfield2",
        "rapier_testbed.examples2.stress_tests.pyramid2",
        "rapier_testbed.examples2.stress_tests.vertical_stacks2",
        "rapier_testbed.examples2.stress_tests.joint_ball2",
        "rapier_testbed.examples2.stress_tests.joint_fixed2",
        "rapier_testbed.examples2.stress_tests.joint_prismatic2",
    )
    for mod in _bundled:
        importlib.import_module(mod)

__all__ = [
    "Testbed",
    "Example",
    "EXAMPLES",
    "CallbackFn",
    "register",
    "find",
    "by_label",
    "run",
]
