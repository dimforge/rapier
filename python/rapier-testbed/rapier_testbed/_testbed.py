"""Panda3D-backed mini testbed for the Rapier Python bindings.

Mirrors the structure of the Rust ``src_testbed`` crate: each example
implements ``init_world(testbed: Testbed) -> None`` which builds a
:class:`RigidBodySet`, :class:`ColliderSet`, :class:`ImpulseJointSet`, and
:class:`MultibodyJointSet`, then hands them to the testbed via
:meth:`Testbed.set_world`. The testbed owns the persistent solver state
(broad phase, narrow phase, island manager, CCD solver, integration
parameters, physics pipeline) and steps it each frame.

Visualization uses :class:`DebugRenderPipeline` + :class:`DebugLineCollector`
to extract a ``(N, 2, D)`` array of line segments and an ``(N, 4)`` array of
RGBA colors each frame; these are uploaded into a Panda3D :class:`LineSegs`
geometry attached to the scene root (``render`` in 3D, ``aspect2d`` in 2D).

Headless mode is selected by setting ``PANDA_NO_WINDOW=1`` in the
environment — the testbed then runs ``N`` (default 60) physics steps
without opening a window, then returns. This is the mode used by the
``tests/test_testbed_smoke.py`` regression test.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from typing import Any, Callable, Deque, List, Optional, Tuple, TYPE_CHECKING, Union

import numpy as np

import rapier3d as _rp3
import rapier2d as _rp2

if TYPE_CHECKING:  # pragma: no cover
    # Imported lazily inside __init__; this is for type-checkers only.
    from panda3d.core import LineSegs, NodePath  # noqa: F401


_DEFAULT_GRAVITY_3D: Tuple[float, float, float] = (0.0, -9.81, 0.0)
_DEFAULT_GRAVITY_2D: Tuple[float, float] = (0.0, -9.81)
_HEADLESS_ENV_VAR: str = "PANDA_NO_WINDOW"
_HEADLESS_STEPS_DEFAULT: int = 60


CallbackFn = Callable[["Testbed"], None]
Vec3Like = Union[Tuple[float, float, float], Any]
Vec2Like = Union[Tuple[float, float], Any]


class _Arcball:
    """Mouse-driven arcball camera.

    The camera orbits a focal point on a sphere of radius ``distance``.

    Controls (matching the Rust testbed / kiss3d ``OrbitCamera3d``):
        * left-drag      : rotate (yaw / pitch around focal point)
        * right-drag     : pan (move focal point in screen plane)
        * mouse-wheel    : zoom
    """

    def __init__(
        self,
        base: Any,
        eye: Tuple[float, float, float],
        at: Tuple[float, float, float],
    ) -> None:
        import math

        from panda3d.core import LPoint3, LVector3  # noqa: F401

        self._base = base
        self._target = LPoint3(*at)
        offset = (eye[0] - at[0], eye[1] - at[1], eye[2] - at[2])
        self._distance = max(1e-3, math.sqrt(sum(v * v for v in offset)))
        # Spherical coords with Y as the up axis.
        self._yaw = math.atan2(offset[0], offset[2])  # around +Y
        self._pitch = math.asin(max(-1.0, min(1.0, offset[1] / self._distance)))
        self._last_mouse: Optional[Tuple[float, float]] = None
        self._dragging: Optional[str] = None

        # Disable Panda's default mouse so we can drive the camera ourselves.
        try:
            base.disableMouse()
        except Exception:
            pass

        # Mouse buttons. Bindings match the Rust testbed (kiss3d
        # OrbitCamera3d): left=rotate, right=pan, wheel=zoom.
        # Panda3D button numbering: mouse1=left, mouse2=middle, mouse3=right.
        base.accept("mouse1", self._on_press, ["rotate"])
        base.accept("mouse1-up", self._on_release)
        base.accept("mouse3", self._on_press, ["pan"])
        base.accept("mouse3-up", self._on_release)
        base.accept("wheel_up", self._on_wheel, [-1.0])
        base.accept("wheel_down", self._on_wheel, [1.0])

        base.taskMgr.add(self._tick, "rapier-arcball-tick")
        self._apply()

    def look_at(self, eye: Tuple[float, float, float], at: Tuple[float, float, float]) -> None:
        """Re-aim the arcball at ``at`` with the camera at ``eye``."""
        import math

        from panda3d.core import LPoint3

        self._target = LPoint3(*at)
        offset = (eye[0] - at[0], eye[1] - at[1], eye[2] - at[2])
        self._distance = max(1e-3, math.sqrt(sum(v * v for v in offset)))
        self._yaw = math.atan2(offset[0], offset[2])
        self._pitch = math.asin(max(-1.0, min(1.0, offset[1] / self._distance)))
        self._apply()

    def _apply(self) -> None:
        import math

        from panda3d.core import LPoint3, LVector3  # noqa: F401

        cp = math.cos(self._pitch)
        sp = math.sin(self._pitch)
        cy = math.cos(self._yaw)
        sy = math.sin(self._yaw)
        x = self._distance * cp * sy
        y = self._distance * sp
        z = self._distance * cp * cy
        eye = self._target + LVector3(x, y, z)
        self._base.camera.setPos(eye)
        self._base.camera.lookAt(self._target, LVector3(0, 1, 0))

    def _on_press(self, mode: str) -> None:
        self._dragging = mode
        if self._base.mouseWatcherNode.hasMouse():
            m = self._base.mouseWatcherNode.getMouse()
            self._last_mouse = (m.getX(), m.getY())

    def _on_release(self) -> None:
        self._dragging = None
        self._last_mouse = None

    def _on_wheel(self, sign: float) -> None:
        self._distance = max(0.05, self._distance * (1.15 ** sign))
        self._apply()

    def _tick(self, task: Any) -> Any:
        import math

        from panda3d.core import LVector3

        if self._dragging is None or self._last_mouse is None:
            return task.cont
        if not self._base.mouseWatcherNode.hasMouse():
            return task.cont
        m = self._base.mouseWatcherNode.getMouse()
        cur = (m.getX(), m.getY())
        dx = cur[0] - self._last_mouse[0]
        dy = cur[1] - self._last_mouse[1]
        self._last_mouse = cur
        if self._dragging == "rotate":
            # full-screen drag ≈ 2π yaw / π pitch
            self._yaw -= dx * math.pi
            self._pitch += dy * math.pi
            # Clamp away from the poles to avoid flips.
            limit = math.pi / 2.0 - 0.02
            self._pitch = max(-limit, min(limit, self._pitch))
            self._apply()
        elif self._dragging == "pan":
            quat = self._base.camera.getQuat()
            right = quat.xform(LVector3(1, 0, 0))
            up = quat.xform(LVector3(0, 1, 0))
            scale = self._distance
            self._target -= right * (dx * scale)
            self._target -= up * (dy * scale)
            self._apply()
        return task.cont


_panda_config_done: bool = False


def _ensure_panda_config() -> None:
    """Apply global Panda3D configuration.

    Must run **before** any ``ShowBase`` is constructed. We force
    ``coordinate-system yup-right`` so Rapier's Y-up world maps
    directly onto Panda3D's render graph (default Panda3D is Z-up,
    which would put gravity into the screen).

    Idempotent — safe to call from multiple entry points (picker,
    testbed direct launch).
    """
    global _panda_config_done
    if _panda_config_done:
        return
    try:
        from panda3d.core import loadPrcFileData

        loadPrcFileData("", "coordinate-system yup-right")
        loadPrcFileData("", "window-title Rapier Python Testbed")
        # Smooth lines look much nicer for debug-render output.
        loadPrcFileData("", "framebuffer-multisample 1")
        loadPrcFileData("", "multisamples 4")
    except Exception:  # pragma: no cover - panda3d not installed
        pass
    _panda_config_done = True


def _is_headless() -> bool:
    """``True`` when the testbed should skip window creation.

    Triggered by setting ``PANDA_NO_WINDOW=1`` (any non-empty value other
    than ``"0"`` or ``"false"``). Used by CI / regression tests.
    """
    val = os.environ.get(_HEADLESS_ENV_VAR, "")
    return val not in ("", "0", "false", "False")


class Testbed:
    """Per-example testbed.

    Parameters
    ----------
    dim:
        ``2`` for a 2D scene (orthographic camera, XY plane) or ``3`` for a
        3D scene (perspective camera + arcball mouse). Defaults to 3.
    headless:
        Force headless mode. When ``None`` (the default), the
        ``PANDA_NO_WINDOW`` environment variable is consulted.
    headless_steps:
        Number of physics steps to run in headless mode before
        :meth:`run` returns. Default 60.
    """

    # ---- per-frame populated by the active example's init_world ----------
    bodies: Any
    colliders: Any
    impulse_joints: Any
    multibody_joints: Any
    gravity: Any

    def __init__(
        self,
        dim: int = 3,
        *,
        headless: Optional[bool] = None,
        headless_steps: int = _HEADLESS_STEPS_DEFAULT,
        base: Optional[Any] = None,
        on_escape: Optional[Callable[[], None]] = None,
    ) -> None:
        """Construct a testbed.

        :param dim: 2 or 3.
        :param headless: skip window creation if True.
        :param headless_steps: number of frames to run in headless mode.
        :param base: existing Panda3D ``ShowBase`` to reuse. **Required
            when the testbed is constructed by the picker on macOS** —
            Cocoa cannot host more than one ``ShowBase`` per process, so
            the picker creates the only one and passes it here. Pass
            ``None`` to let the testbed create its own (the common path
            for ``python -m rapier_testbed.examples3.<name>`` direct
            launches and headless runs).
        """
        if dim not in (2, 3):
            raise ValueError(f"Testbed dim must be 2 or 3, got {dim!r}")
        self.dim: int = dim
        self._ns = _rp2 if dim == 2 else _rp3
        self._headless: bool = _is_headless() if headless is None else bool(headless)
        self._headless_steps: int = int(headless_steps)

        # ---- solver state (persistent across example switches) ----------
        self._reset_solver_state()

        # ---- world sub-sets (populated by set_world) ---------------------
        ns = self._ns
        self.bodies = ns.RigidBodySet()
        self.colliders = ns.ColliderSet()
        self.impulse_joints = ns.ImpulseJointSet()
        self.multibody_joints = ns.MultibodyJointSet()
        self.gravity = _DEFAULT_GRAVITY_3D if dim == 3 else _DEFAULT_GRAVITY_2D
        self._hooks: Optional[Any] = None
        self._event_handler: Optional[Any] = None

        # ---- example metadata --------------------------------------------
        self._init_fn: Optional[CallbackFn] = None
        self._callbacks: List[CallbackFn] = []
        self._example_label: str = "(no example)"

        # ---- runtime state -----------------------------------------------
        self._paused: bool = False
        self._single_step: bool = False
        self._step_count: int = 0
        self._line_collector = self._ns.DebugLineCollector()
        self._debug_pipeline = self._ns.DebugRenderPipeline()

        # ---- timing rings (rolling 60-frame averages) --------------------
        self._physics_times: Deque[float] = deque(maxlen=60)
        self._render_times: Deque[float] = deque(maxlen=60)
        self._frame_times: Deque[float] = deque(maxlen=60)
        self._last_frame_perf: Optional[float] = None

        # ---- Panda3D scaffold (None when headless) -----------------------
        # ``base`` may be pre-supplied by the picker; otherwise we create
        # one in ``_init_panda3d``.
        self._base: Any = base
        self._arcball: Optional[_Arcball] = None
        self._line_node: Any = None  # NodePath to the LineSegs geom (overlay)
        self._line_segs: Any = None  # LineSegs accumulator (overlay)
        self._status_text: Any = None
        self._label_text: Any = None
        self._initial_camera_eye: Optional[Tuple[float, float, float]] = None
        self._initial_camera_at: Optional[Tuple[float, float, float]] = None
        self._initial_2d_center: Tuple[float, float] = (0.0, 0.0)
        self._initial_2d_zoom: float = 1.0

        # ---- mesh-rendering state ----------------------------------------
        # One NodePath per collider, parented to ``_mesh_root``. Each
        # collider group then has one or more child Geom NodePaths (a
        # Compound shape contributes one Geom per sub-shape with its
        # own local pose).
        self._mesh_nodes: dict = {}              # ColliderHandle.index → NodePath group
        self._mesh_root: Any = None              # parent NodePath under render
        # Cleanup hooks: ``_cleanup_scene()`` runs through these at the end
        # of ``run()`` so the same ``ShowBase`` can be reused across picks.
        self._tracked_nodes: List[Any] = []      # NodePaths to ``removeNode()``
        self._tracked_lights: List[Any] = []     # NodePaths of lights → clearLight()
        self._tracked_widgets: List[Any] = []    # OnscreenText / DirectGui → destroy()
        self._tracked_tasks: List[str] = []      # taskMgr task names to remove
        # Callback invoked when the user presses Esc. When set (CLI mode)
        # the testbed asks the caller to unload it instead of stopping
        # the main loop. ``None`` falls back to ``base.taskMgr.stop()``.
        self._on_escape: Optional[Callable[[], None]] = on_escape
        self._known_colliders: set = set()       # ColliderHandle.index seen
        self._show_wireframe: bool = False       # debug-line overlay toggle

        if not self._headless:
            _ensure_panda_config()
            self._init_panda3d()

    # =====================================================================
    # Solver state
    # =====================================================================

    def _reset_solver_state(self) -> None:
        ns = self._ns
        self._broad_phase = ns.BroadPhaseBvh()
        self._narrow_phase = ns.NarrowPhase()
        self._islands = ns.IslandManager()
        self._ccd_solver = ns.CCDSolver()
        self._integration_parameters = ns.IntegrationParameters()
        self._physics_pipeline = ns.PhysicsPipeline()

    # =====================================================================
    # Public API mirrored from Rust Testbed
    # =====================================================================

    def set_world(
        self,
        bodies: Any,
        colliders: Any,
        impulse_joints: Any,
        multibody_joints: Any,
    ) -> None:
        """Bind freshly-built sets as the active simulation state.

        Mirrors :meth:`rapier_testbed::Testbed::set_world`. Uses the
        testbed's current :attr:`gravity` (defaulting to ``(0, -9.81, 0)``
        / ``(0, -9.81)`` if the example doesn't override it).
        """
        self.set_world_with_params(
            bodies, colliders, impulse_joints, multibody_joints, self.gravity
        )

    def set_world_with_params(
        self,
        bodies: Any,
        colliders: Any,
        impulse_joints: Any,
        multibody_joints: Any,
        gravity: Any,
        hooks: Optional[Any] = None,
    ) -> None:
        """Bind sets + override gravity and (optionally) physics hooks."""
        self.bodies = bodies
        self.colliders = colliders
        self.impulse_joints = impulse_joints
        self.multibody_joints = multibody_joints
        self.gravity = gravity
        self._hooks = hooks
        self._step_count = 0
        # Fresh solver state on world swap (matches Rust testbed semantics).
        self._reset_solver_state()
        # Drop per-collider mesh nodes so the next render frame rebuilds.
        self._clear_mesh_cache()

    def set_gravity(self, gravity: Any) -> None:
        """Replace the gravity vector for the active world."""
        self.gravity = gravity

    def set_hooks(self, hooks: Optional[Any]) -> None:
        """Install or detach a physics hooks object."""
        self._hooks = hooks

    def set_event_handler(self, events: Optional[Any]) -> None:
        """Install or detach an event handler (e.g. ``ChannelEventCollector``)."""
        self._event_handler = events

    def add_callback(self, fn: CallbackFn) -> None:
        """Register a per-frame callback invoked after each successful step.

        The callback receives the testbed itself. Use for vehicle
        controllers, custom forces, statistics — anything you need to run
        between physics steps.
        """
        self._callbacks.append(fn)

    # ---- thin world-mutation helpers (delegate to the active sets) ------

    def add_body(self, builder: Any) -> Any:
        """Insert a rigid body via the active :class:`RigidBodySet`."""
        return self.bodies.insert(builder)

    def add_collider(self, builder: Any, parent: Optional[Any] = None) -> Any:
        """Insert a collider, optionally with a parent body.

        Mirrors ``ColliderSet::insert`` / ``insert_with_parent``.
        """
        if parent is None:
            return self.colliders.insert(builder)
        return self.colliders.insert_with_parent(builder, parent, self.bodies)

    # ---- camera ---------------------------------------------------------

    def look_at(self, eye: Vec3Like, at: Vec3Like) -> None:
        """Position the 3D camera at ``eye`` looking at ``at``."""
        if self.dim != 3:
            raise RuntimeError("look_at requires a 3D testbed")
        e = _as_tuple3(eye)
        a = _as_tuple3(at)
        self._initial_camera_eye = e
        self._initial_camera_at = a
        if self._base is not None:
            self._apply_camera_3d(e, a)

    def set_camera_2d(self, center: Vec2Like = (0.0, 0.0), zoom: float = 1.0) -> None:
        """Position the 2D orthographic camera."""
        if self.dim != 2:
            raise RuntimeError("set_camera_2d requires a 2D testbed")
        c = _as_tuple2(center)
        self._initial_2d_center = c
        self._initial_2d_zoom = float(zoom)
        if self._base is not None:
            self._apply_camera_2d(c, float(zoom))

    # =====================================================================
    # Example lifecycle
    # =====================================================================

    def load_example(self, init_fn: CallbackFn, label: str = "") -> None:
        """Reset the world and call ``init_fn(self)`` to repopulate it."""
        self._init_fn = init_fn
        self._example_label = label or init_fn.__qualname__
        ns = self._ns
        # Hand the user a fresh empty quadruple; init_fn either populates
        # these (uncommon) or builds its own and calls set_world(...).
        self.bodies = ns.RigidBodySet()
        self.colliders = ns.ColliderSet()
        self.impulse_joints = ns.ImpulseJointSet()
        self.multibody_joints = ns.MultibodyJointSet()
        self.gravity = _DEFAULT_GRAVITY_3D if self.dim == 3 else _DEFAULT_GRAVITY_2D
        self._callbacks = []
        self._step_count = 0
        self._paused = False
        self._reset_solver_state()
        # Drop per-collider mesh nodes — example switch swaps the world.
        self._clear_mesh_cache()
        init_fn(self)
        self._refresh_status_text()

    def reset(self) -> None:
        """Re-run the active example's ``init_world`` from scratch."""
        if self._init_fn is None:
            return
        self.load_example(self._init_fn, self._example_label)

    # =====================================================================
    # Per-frame step + render
    # =====================================================================

    def step_once(self) -> None:
        """Run a single physics step and invoke per-frame callbacks."""
        self._physics_pipeline.step(
            self.gravity,
            self._integration_parameters,
            self._islands,
            self._broad_phase,
            self._narrow_phase,
            self.bodies,
            self.colliders,
            self.impulse_joints,
            self.multibody_joints,
            self._ccd_solver,
            self._hooks,
            self._event_handler,
        )
        self._step_count += 1
        for cb in self._callbacks:
            cb(self)

    def _render_frame(self) -> None:
        """Build/update per-collider mesh nodes and (optionally) overlay lines.

        Two passes:

        1. ``_sync_mesh_nodes``: detach removed colliders, build Geom
           nodes for newly-inserted colliders (one-time mesh upload).
        2. ``_update_transforms``: cheap per-frame `setPosQuat` on each
           collider group; transforms come from the parent body's pose
           (or the collider's own ``position`` for body-less colliders).

        When ``_show_wireframe`` is set, the debug-line pipeline is also
        run and uploaded as a translucent overlay.
        """
        if self._base is None:
            return
        self._sync_mesh_nodes()
        self._update_transforms()
        if self._show_wireframe:
            lines, colors, _objects = self._debug_pipeline.render_to_arrays(
                self.bodies,
                self.colliders,
                self.impulse_joints,
                self.multibody_joints,
                self._narrow_phase,
            )
            self._upload_lines(lines, colors)
        elif self._line_node is not None:
            self._line_node.removeNode()
            self._line_node = None
            self._line_segs = None

    def _toggle_wireframe(self) -> None:
        """Flip the debug-line overlay on/off."""
        self._show_wireframe = not self._show_wireframe

    def _clear_mesh_cache(self) -> None:
        """Drop every per-collider node. Called on world swap / reset."""
        if self._mesh_root is not None:
            for np_group in list(self._mesh_nodes.values()):
                try:
                    np_group.removeNode()
                except Exception:
                    pass
        self._mesh_nodes.clear()
        self._known_colliders.clear()

    def _sync_mesh_nodes(self) -> None:
        """Insert nodes for new colliders; drop nodes for removed ones."""
        if self._mesh_root is None:
            return
        from panda3d.core import GeomNode

        # Local imports keep headless paths from touching panda3d here.
        from . import _meshes

        current = {}  # handle.index → collider
        for handle, col in self.colliders:
            current[int(handle.index)] = (handle, col)

        # 1) Remove nodes for colliders that have disappeared.
        for idx in list(self._known_colliders):
            if idx not in current:
                np_group = self._mesh_nodes.pop(idx, None)
                if np_group is not None:
                    try:
                        np_group.removeNode()
                    except Exception:
                        pass
                self._known_colliders.discard(idx)

        # 2) Build nodes for new colliders.
        for idx, (handle, col) in current.items():
            if idx in self._known_colliders:
                continue
            color = self._color_for_collider(col)
            try:
                geoms = _meshes.shape_to_geoms(col.shape, dim=self.dim, color_rgba=color)
            except Exception as exc:
                # Unsupported shape: skip but emit a one-off warning so
                # silent fallbacks are at least visible in the console.
                import sys
                print(
                    f"[rapier_testbed] mesh build failed for collider {idx} "
                    f"(shape={col.shape.shape_type}): {exc!r}",
                    file=sys.stderr,
                )
                self._known_colliders.add(idx)
                continue
            group_np = self._mesh_root.attachNewNode(f"collider-{idx}")
            # Render both sides — guards against winding surprises across
            # the various shape converters (sphere/cylinder/heightfield
            # use different conventions) and helps when the camera ends up
            # inside an open shape like a heightfield's underside.
            group_np.setTwoSided(True)
            for j, (local_pose, geom) in enumerate(geoms):
                node = GeomNode(f"collider-{idx}-{j}")
                node.addGeom(geom)
                child = group_np.attachNewNode(node)
                self._apply_local_pose(child, local_pose)
                # Use vertex colors; turn off any inherited material.
                child.setMaterialOff(1)
            self._mesh_nodes[idx] = group_np
            self._known_colliders.add(idx)

    def _update_transforms(self) -> None:
        """Per-frame: push each collider's world pose into its NodePath."""
        if self._mesh_root is None:
            return
        from panda3d.core import LQuaternion

        for handle, col in self.colliders:
            idx = int(handle.index)
            np_group = self._mesh_nodes.get(idx)
            if np_group is None:
                continue
            # World pose comes from the parent body's transform when the
            # collider is attached; otherwise the collider's own position
            # (collider.position is already in world space for body-less
            # colliders, and equal to parent_pose * collider_offset
            # otherwise — but Rapier exposes the world-space position
            # directly so we can use it unconditionally).
            iso = col.position
            t = iso.translation
            if self.dim == 3:
                np_group.setPos(float(t.x), float(t.y), float(t.z))
                r = iso.rotation
                # Panda3D LQuaternion takes (w, x, y, z) — matches Rapier.
                np_group.setQuat(LQuaternion(float(r.w), float(r.i), float(r.j), float(r.k)))
            else:
                # Embed 2D world (x, y) into 3D as (x, y, 0). The 2D
                # rotation is around the Z axis (out of the screen, with
                # gravity pointing down on screen).
                np_group.setPos(float(t.x), float(t.y), 0.0)
                angle = float(iso.rotation.angle)
                half = angle * 0.5
                np_group.setQuat(LQuaternion(math.cos(half), 0.0, 0.0, math.sin(half)))

    def _apply_local_pose(self, np_node: Any, iso: Any) -> None:
        """Apply a sub-shape's local pose to a Compound-child NodePath."""
        from panda3d.core import LQuaternion

        t = iso.translation
        if self.dim == 3:
            np_node.setPos(float(t.x), float(t.y), float(t.z))
            r = iso.rotation
            np_node.setQuat(LQuaternion(float(r.w), float(r.i), float(r.j), float(r.k)))
        else:
            # 2D world (x, y) → 3D (x, y, 0); rotation around +Z.
            np_node.setPos(float(t.x), float(t.y), 0.0)
            angle = float(iso.rotation.angle)
            half = angle * 0.5
            np_node.setQuat(LQuaternion(math.cos(half), 0.0, 0.0, math.sin(half)))

    def _color_for_collider(self, col: Any) -> Tuple[float, float, float, float]:
        """Pick a per-collider color: parent-body palette when attached,
        deterministic fallback when free-standing."""
        from . import _meshes

        if col.parent is not None:
            body = self.bodies.get(col.parent)
            if body is not None:
                return _meshes.color_for_body(int(col.parent.index), body.body_type)
        return _meshes.color_for_free_collider(0)

    # =====================================================================
    # Run loop
    # =====================================================================

    def run(self) -> None:
        """Open the window (or run headlessly) and step the simulation.

        When ``PANDA_NO_WINDOW`` is set the testbed runs a fixed number of
        physics steps then returns, which is what CI uses. Otherwise the
        Panda3D :class:`ShowBase` event loop takes over until the user
        presses ``escape``.
        """
        if self._headless:
            for _ in range(self._headless_steps):
                self.step_once()
            return

        # Default path: install the step task on the existing ShowBase and
        # drive ``base.run()`` ourselves. Suitable for direct-launch
        # (``python -m rapier_testbed.examples3.<name>``).
        self.install_step_task()
        assert self._base is not None
        try:
            self._base.run()
        finally:
            # Esc / window-close lands here. Tear down every scene-graph
            # node, light, UI widget, task and keyboard binding this
            # testbed installed, so the caller (the CLI picker) can launch
            # another scenario in the same ``ShowBase``.
            self._cleanup_scene()

    def _stop_loop(self) -> None:
        """Exit the Panda3D task loop without killing the process."""
        if self._on_escape is not None:
            try:
                self._on_escape()
            except Exception:  # pragma: no cover
                pass
            return
        if self._base is not None:
            try:
                self._base.taskMgr.stop()
            except Exception:  # pragma: no cover
                pass

    def install_step_task(self) -> None:
        """Add the per-frame physics-step + render task to ``base.taskMgr``.

        Used by both ``Testbed.run()`` (the direct-launch path) and the
        CLI picker (which drives ``base.run()`` itself across many
        scenarios). Idempotent: calling twice is a no-op via the unique
        task name.
        """
        if self._headless or self._base is None:
            return
        from direct.task import Task

        # Track tasks so ``_cleanup_scene`` removes them. Task names that
        # weren't actually installed (no arcball / no 2D cam) just fail
        # silently when removed.
        self._tracked_tasks.append("rapier-testbed-step")
        self._tracked_tasks.append("rapier-arcball-tick")
        self._tracked_tasks.append("rapier-cam2d-tick")

        def _task(task: Any) -> Any:
            now = time.perf_counter()
            if self._last_frame_perf is not None:
                self._frame_times.append(now - self._last_frame_perf)
            self._last_frame_perf = now

            if not self._paused or self._single_step:
                t0 = time.perf_counter()
                self.step_once()
                self._physics_times.append(time.perf_counter() - t0)
                self._single_step = False

            t1 = time.perf_counter()
            self._render_frame()
            self._render_times.append(time.perf_counter() - t1)

            self._refresh_status_text()
            return Task.cont

        self._base.taskMgr.add(_task, "rapier-testbed-step")

    def _cleanup_scene(self) -> None:
        """Detach everything this testbed added to the shared ``ShowBase``.

        Called from the ``finally`` clause of :meth:`run` so the picker
        can re-use the same ``ShowBase`` for the next scenario.
        """
        if self._base is None:
            return
        # Remove per-frame tasks.
        for name in self._tracked_tasks:
            try:
                self._base.taskMgr.remove(name)
            except Exception:  # pragma: no cover
                pass
        self._tracked_tasks.clear()
        # Drop keyboard handlers we installed.
        try:
            self._base.ignoreAll()
        except Exception:  # pragma: no cover
            pass
        # Destroy on-screen text + DirectGui widgets.
        for w in self._tracked_widgets:
            try:
                w.destroy()
            except Exception:  # pragma: no cover
                pass
        self._tracked_widgets.clear()
        # Drop lights from the scene graph.
        try:
            scene_root = self._base.render
            for lnp in self._tracked_lights:
                try:
                    scene_root.clearLight(lnp)
                except Exception:
                    pass
                try:
                    lnp.removeNode()
                except Exception:
                    pass
        except Exception:  # pragma: no cover
            pass
        self._tracked_lights.clear()
        # Detach tracked scene nodes (mesh root, etc.) and the line node.
        for n in list(self._tracked_nodes):
            try:
                n.removeNode()
            except Exception:  # pragma: no cover
                pass
        self._tracked_nodes.clear()
        if self._line_node is not None:
            try:
                self._line_node.removeNode()
            except Exception:  # pragma: no cover
                pass
            self._line_node = None
        # Drop per-collider node caches so the next scenario rebuilds.
        self._mesh_nodes.clear()
        self._known_colliders.clear()
        self._mesh_root = None
        # Reset arcball / 2D-camera bindings.
        self._arcball = None
        setattr(self, "_cam2d_bound", False)
        # Status / label text already destroyed via _tracked_widgets.
        self._label_text = None
        self._status_text = None

    # =====================================================================
    # Panda3D scaffold
    # =====================================================================

    def _init_panda3d(self) -> None:
        # Imports are local so headless usage doesn't pull in Panda3D.
        from direct.showbase.ShowBase import ShowBase
        from direct.gui.OnscreenText import OnscreenText
        from panda3d.core import (
            AmbientLight,
            DirectionalLight,
            LVector3,
            OrthographicLens,
            TextNode,
            WindowProperties,
        )

        if self._base is None:
            self._base = ShowBase()
        else:
            # The picker pre-created the ShowBase and handed it to us.
            # Clear any handlers it registered so they don't collide
            # with ours (e.g., the picker's digit→pick callbacks).
            self._base.ignoreAll()
        # ``ignoreAll`` (above or in a previous ``_cleanup_scene``) wipes
        # ShowBase's own ``window-event`` handler, which is what updates
        # the camera lens's aspect ratio when the user resizes the
        # window. Without it the scene stretches with the window. Re-
        # register the bound method so resize keeps working.
        try:
            self._base.accept("window-event", self._base.windowEvent)
        except Exception:  # pragma: no cover - cosmetic only
            pass
        # Background matches the Rust testbed: (245, 245, 236) / 255 cream.
        self._base.setBackgroundColor(245.0 / 255.0, 245.0 / 255.0, 236.0 / 255.0, 1.0)

        # ---- lighting: ambient + one directional key light ----
        # Both 2D and 3D scenes live in `render`. 2D is embedded in the
        # z=0 plane and viewed with an orthographic camera (set up later).
        scene_root = self._base.render
        ambient = AmbientLight("rapier-ambient")
        ambient.setColor((0.50, 0.50, 0.50, 1.0))
        ambient_np = scene_root.attachNewNode(ambient)
        scene_root.setLight(ambient_np)
        self._tracked_lights.append(ambient_np)
        directional = DirectionalLight("rapier-key")
        directional.setColor((0.85, 0.85, 0.85, 1.0))
        dir_np = scene_root.attachNewNode(directional)
        if self.dim == 3:
            # Aim the light down-and-forward (Y-up world).
            dir_np.setHpr(-20, -45, 0)
        else:
            # 2D shapes all face +Z; aim the directional straight along -Z
            # so face-normals get fully lit.
            dir_np.setHpr(0, 0, 0)
        scene_root.setLight(dir_np)
        self._tracked_lights.append(dir_np)

        # Root NodePath for all collider meshes — letting us flush them
        # all at once on world swap.
        self._mesh_root = scene_root.attachNewNode("rapier-meshes")
        self._tracked_nodes.append(self._mesh_root)
        try:
            props = WindowProperties()
            props.setTitle("Rapier Python Testbed")
            self._base.win.requestProperties(props)
        except Exception:  # pragma: no cover - cosmetic only
            pass

        # 2D uses an orthographic main camera looking along -Z at the
        # z=0 plane, with screen-up = +Y. Mouse-driven pan + wheel-zoom
        # controls are wired in `_apply_camera_2d`. Lens + camera placement
        # happen below, after the lens is fully configured.

        # Empty placeholder; replaced on the first `_upload_lines` call.
        self._line_node = None
        self._line_segs = None

        # On-screen text overlays. ``aspect2d``'s X range is
        # ``[-aspect_ratio, +aspect_ratio]`` and its Y range is
        # ``[-1, +1]`` (top = +1). We pin the text just inside the
        # top-left corner by re-computing X from the current aspect
        # ratio whenever the window resizes (see
        # ``_layout_overlay_text``). ``base.a2dTopLeft`` doesn't work
        # for us — its position is hard-coded for Z-up and ends up at
        # the middle-left under our Y-up coord system.
        self._label_text = OnscreenText(
            text=self._example_label,
            pos=(0, 0.92),
            scale=0.05,
            fg=(0.10, 0.10, 0.10, 1),
            bg=(1, 1, 1, 0.55),
            align=TextNode.ALeft,
            mayChange=True,
        )
        self._status_text = OnscreenText(
            text="running  step 0",
            pos=(0, 0.85),
            scale=0.04,
            fg=(0.05, 0.30, 0.10, 1),
            bg=(1, 1, 1, 0.55),
            align=TextNode.ALeft,
            mayChange=True,
        )
        self._layout_overlay_text()
        # Re-pin the text whenever the window's aspect ratio changes.
        try:
            self._base.accept(
                "aspectRatioChanged", self._layout_overlay_text
            )
        except Exception:  # pragma: no cover - cosmetic only
            pass
        self._tracked_widgets.extend([self._label_text, self._status_text])

        # Camera positioning.
        if self.dim == 3:
            if self._initial_camera_eye is None:
                # Default 3D view (Y-up convention): elevated, slightly
                # off-axis, looking down at the origin.
                self._initial_camera_eye = (20.0, 25.0, 40.0)
                self._initial_camera_at = (0.0, 0.0, 0.0)
            self._apply_camera_3d(
                self._initial_camera_eye,
                self._initial_camera_at or (0.0, 0.0, 0.0),
            )
        else:
            self._apply_camera_2d(self._initial_2d_center, self._initial_2d_zoom)

        # Keyboard handlers. Each digit binds three events:
        # - ``"1"`` (character event) — works on QWERTY / AZERTY-shifted etc.
        # - ``"raw-1"`` (layout-independent physical key) — works on layouts
        #   whose unshifted top row produces typographic characters instead
        #   of digits (BÉPO, Dvorak, programmer's Dvorak, …).
        # - ``"kp_1"`` (numpad) — universal fallback.
        # Letter shortcuts (``r``, ``space``, ``tab``) stay on the character
        # event since users think in terms of letters, not physical position.
        # Esc just stops the task loop — control returns to whoever called
        # ``Testbed.run()`` (the CLI picker) so they can launch another
        # scenario without restarting the process.
        self._base.accept("escape", self._stop_loop)
        self._base.accept("space", self._toggle_pause)
        self._base.accept("arrow_right", self._do_single_step)
        self._base.accept("r", self.reset)
        self._base.accept("tab", self._cycle_example)
        self._base.accept("w", self._toggle_wireframe)
        for d in range(0, 10):
            self._base.accept(str(d), self._goto_example_index, [d])
            self._base.accept(f"raw-{d}", self._goto_example_index, [d])
            self._base.accept(f"kp_{d}", self._goto_example_index, [d])

    def _apply_camera_3d(self, eye: Tuple[float, float, float], at: Tuple[float, float, float]) -> None:
        """Aim the 3D camera. Creates / re-targets the arcball controller.

        Mouse controls: left=rotate, middle=pan, right=zoom, wheel=zoom step.
        See :class:`_Arcball`.
        """
        if self._base is None:
            return
        if self._arcball is None:
            self._arcball = _Arcball(self._base, eye, at)
        else:
            self._arcball.look_at(eye, at)

    def _apply_camera_2d(self, center: Tuple[float, float], zoom: float) -> None:
        """Place the main camera so 2D worlds render in the z=0 plane.

        Uses an orthographic projection: the world-X axis maps to screen-X,
        world-Y maps to screen-Y, world-Z is the depth axis. The camera
        sits at ``(cx, cy, +D)`` and looks toward ``(cx, cy, 0)`` with
        screen-up = +Y. Wires up mouse pan / wheel zoom controls.
        """
        if self._base is None:
            return
        from panda3d.core import LPoint3, LVector3, OrthographicLens

        cx, cy = float(center[0]), float(center[1])
        z = float(zoom) if zoom > 1e-6 else 1.0

        try:
            self._base.disableMouse()
        except Exception:
            pass

        # Orthographic lens. Film size = visible world width/height in
        # world units. zoom > 1 ⇒ closer-in (smaller film), zoom < 1
        # ⇒ wider view.
        lens = OrthographicLens()
        film = 40.0 / z
        try:
            aspect = self._base.getAspectRatio()
        except Exception:
            aspect = 16.0 / 9.0
        lens.setFilmSize(film * max(aspect, 1.0), film)
        lens.setNearFar(-1000, 1000)
        try:
            self._base.cam.node().setLens(lens)
        except Exception:  # pragma: no cover - fallback
            return

        # Position the camera in front of the z=0 plane, looking along -Z.
        # The explicit `up=+Y` form requires an LPoint3 target — passing
        # three floats doesn't accept a 4th up-vector arg.
        cam = self._base.camera
        cam.setPos(cx, cy, 100.0)
        cam.lookAt(LPoint3(cx, cy, 0.0), LVector3(0, 1, 0))

        # Stash for the mouse pan/zoom handlers.
        self._cam2d_center = [cx, cy]
        self._cam2d_zoom = z
        self._cam2d_lens = lens
        self._cam2d_film_height = film
        self._cam2d_aspect = max(aspect, 1.0)

        # Bind mouse pan + wheel zoom once.
        if not getattr(self, "_cam2d_bound", False):
            self._base.accept("mouse1", self._cam2d_press)
            self._base.accept("mouse1-up", self._cam2d_release)
            self._base.accept("mouse3", self._cam2d_press)
            self._base.accept("mouse3-up", self._cam2d_release)
            self._base.accept("wheel_up", self._cam2d_zoom_step, [-1.0])
            self._base.accept("wheel_down", self._cam2d_zoom_step, [1.0])
            self._base.taskMgr.add(self._cam2d_tick, "rapier-cam2d-tick")
            self._cam2d_bound = True
            self._cam2d_last_mouse: Optional[Tuple[float, float]] = None
            self._cam2d_dragging: bool = False

    def _cam2d_press(self) -> None:
        if self._base is None or not self._base.mouseWatcherNode.hasMouse():
            return
        m = self._base.mouseWatcherNode.getMouse()
        self._cam2d_last_mouse = (m.getX(), m.getY())
        self._cam2d_dragging = True

    def _cam2d_release(self) -> None:
        self._cam2d_dragging = False
        self._cam2d_last_mouse = None

    def _cam2d_zoom_step(self, sign: float) -> None:
        self._cam2d_zoom = max(0.05, self._cam2d_zoom * (1.15 ** -sign))
        self._apply_camera_2d(tuple(self._cam2d_center), self._cam2d_zoom)

    def _cam2d_tick(self, task: Any) -> Any:
        from direct.task import Task
        from panda3d.core import LPoint3, LVector3

        if not self._cam2d_dragging or self._cam2d_last_mouse is None:
            return Task.cont
        if self._base is None or not self._base.mouseWatcherNode.hasMouse():
            return Task.cont
        m = self._base.mouseWatcherNode.getMouse()
        cur = (m.getX(), m.getY())
        dx = cur[0] - self._cam2d_last_mouse[0]
        dy = cur[1] - self._cam2d_last_mouse[1]
        self._cam2d_last_mouse = cur
        # Convert mouse-NDC delta into world units. Mouse coords run [-1, 1]
        # so half the film size in world units per unit NDC.
        film_h = self._cam2d_film_height
        film_w = film_h * self._cam2d_aspect
        self._cam2d_center[0] -= dx * 0.5 * film_w
        self._cam2d_center[1] -= dy * 0.5 * film_h
        cx, cy = self._cam2d_center
        try:
            self._base.camera.setPos(cx, cy, 100.0)
            self._base.camera.lookAt(LPoint3(cx, cy, 0.0), LVector3(0, 1, 0))
        except Exception:
            pass
        return Task.cont

    # ---- keyboard handlers ----------------------------------------------

    def _toggle_pause(self) -> None:
        self._paused = not self._paused

    def _do_single_step(self) -> None:
        self._single_step = True

    def _cycle_example(self) -> None:
        from ._registry import EXAMPLES

        if not EXAMPLES or self._init_fn is None:
            return
        try:
            idx = next(
                i for i, ex in enumerate(EXAMPLES) if ex.init_fn is self._init_fn
            )
        except StopIteration:
            idx = -1
        next_idx = (idx + 1) % len(EXAMPLES)
        ex = EXAMPLES[next_idx]
        self.load_example(ex.init_fn, f"{ex.category} / {ex.name}")

    def _goto_example_index(self, idx: int) -> None:
        from ._registry import EXAMPLES

        if 0 <= idx < len(EXAMPLES):
            ex = EXAMPLES[idx]
            self.load_example(ex.init_fn, f"{ex.category} / {ex.name}")

    def _layout_overlay_text(self) -> None:
        """Re-pin the on-screen label + status to the window's top-left.

        ``aspect2d``'s X coords run from ``-aspect`` to ``+aspect``, so
        the left edge of the window is at ``X = -aspect`` and depends
        on the current window dimensions. We re-compute that on each
        ``aspectRatioChanged`` event (fired by ShowBase's
        ``windowEvent`` when the user resizes the window) so the text
        stays glued to the top-left corner regardless of shape.
        """
        if self._base is None or self._label_text is None:
            return
        try:
            aspect = self._base.getAspectRatio()
        except Exception:  # pragma: no cover - cosmetic only
            aspect = 16.0 / 9.0
        x = -aspect + 0.05  # 0.05 inset from the left edge
        try:
            self._label_text.setPos(x, 0.92)
            if self._status_text is not None:
                self._status_text.setPos(x, 0.85)
        except Exception:  # pragma: no cover - cosmetic only
            pass

    def _refresh_status_text(self) -> None:
        if self._status_text is None:
            return
        state = "paused" if self._paused else "running"

        def _avg_ms(d: Deque[float]) -> float:
            return (sum(d) / len(d) * 1000.0) if d else 0.0

        def _avg_fps(d: Deque[float]) -> float:
            avg = (sum(d) / len(d)) if d else 0.0
            return (1.0 / avg) if avg > 0 else 0.0

        physics_ms = _avg_ms(self._physics_times)
        render_ms = _avg_ms(self._render_times)
        fps = _avg_fps(self._frame_times)
        wf = " [wireframe]" if self._show_wireframe else ""
        self._status_text.setText(
            f"{state}  step {self._step_count}{wf}\n"
            f"physics {physics_ms:5.2f} ms  render {render_ms:5.2f} ms  {fps:5.1f} fps"
        )
        if self._label_text is not None:
            self._label_text.setText(self._example_label)

    # ---- bulk line upload ------------------------------------------------

    def _upload_lines(self, lines: np.ndarray, colors: np.ndarray) -> None:
        """Push per-frame debug lines onto the GPU in a single transfer.

        Uses a pre-allocated ``GeomVertexData`` whose buffer is filled by
        a single ``setData(bytes)`` call sourced from a structured NumPy
        array. The previous implementation called ``LineSegs.setColor/
        moveTo/drawTo`` once per segment — 3 Python→C++ calls per line —
        which dominates frame time at >1k segments. This path performs
        one upload regardless of segment count.
        """
        from panda3d.core import (
            Geom,
            GeomLines,
            GeomNode,
            GeomVertexData,
            GeomVertexFormat,
        )

        n = int(lines.shape[0]) if lines.size else 0

        # Project (N, 2, D) → (2N, 3) float32 vertex positions.
        if n == 0:
            verts = np.empty((0, 3), dtype=np.float32)
        elif lines.shape[2] == 2:
            # 2D world embedded in (x, y, 0); flatten endpoints AB,AB,AB,…
            flat = np.asarray(lines, dtype=np.float32).reshape(2 * n, 2)
            verts = np.empty((2 * n, 3), dtype=np.float32)
            verts[:, 0] = flat[:, 0]
            verts[:, 1] = flat[:, 1]
            verts[:, 2] = 0.0
        else:
            verts = np.ascontiguousarray(
                np.asarray(lines, dtype=np.float32).reshape(2 * n, 3)
            )

        # Per-vertex colors as uint8 RGBA. Each segment's two endpoints
        # share the same color; expand (N, 4) → (2N, 4).
        if n == 0:
            cols_u8 = np.empty((0, 4), dtype=np.uint8)
        else:
            col_f = np.clip(np.asarray(colors, dtype=np.float32), 0.0, 1.0)
            cols_u8 = (col_f * 255.0).astype(np.uint8, copy=False)
            cols_u8 = np.repeat(cols_u8, 2, axis=0)

        # Allocate/reuse GeomVertexData. We rebuild the Geom each frame
        # because the vertex count varies frame-to-frame; the heavy lift
        # — vertex data — is one bulk byte upload via setData().
        fmt = GeomVertexFormat.getV3c4()
        vdata = GeomVertexData("rapier-debug-lines", fmt, Geom.UHStream)
        vdata.unclean_set_num_rows(2 * n)

        if n > 0:
            # Interleave into the V3C4 stride: 3*float32 (xyz) + 4*uint8 (RGBA) = 16 bytes.
            dtype = np.dtype(
                [("pos", np.float32, 3), ("color", np.uint8, 4)]
            )
            buf = np.empty(2 * n, dtype=dtype)
            buf["pos"] = verts
            buf["color"] = cols_u8
            arr_handle = vdata.modifyArrayHandle(0)
            arr_handle.copyDataFrom(buf.tobytes())

        prim = GeomLines(Geom.UHStream)
        if n > 0:
            prim.addConsecutiveVertices(0, 2 * n)
            prim.closePrimitive()

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode("rapier-debug-lines")
        node.addGeom(geom)

        if self._line_node is not None:
            self._line_node.removeNode()
        # Both 2D and 3D scenes live under `render`; 2D just uses an
        # orthographic camera looking along -Z at the z=0 plane.
        self._line_node = self._base.render.attachNewNode(node)
        # Line thickness via RenderModeAttrib (LineSegs equivalent of setThickness).
        self._line_node.setRenderModeThickness(1.5)
        # Lines carry their own per-vertex colors; disable lighting so they
        # render at full intensity even with no light in the scene.
        self._line_node.setLightOff(1)
        # Cache marker (kept for backwards compat with any external code
        # poking at _line_segs — the old LineSegs builder is gone).
        self._line_segs = node


# ---------------------------------------------------------------------------
# Tiny coercion helpers
# ---------------------------------------------------------------------------


def _as_tuple3(v: Any) -> Tuple[float, float, float]:
    if hasattr(v, "x") and hasattr(v, "y") and hasattr(v, "z"):
        return (float(v.x), float(v.y), float(v.z))
    seq = tuple(v)  # type: ignore[arg-type]
    if len(seq) != 3:
        raise ValueError(f"expected a 3-vector, got {v!r}")
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def _as_tuple2(v: Any) -> Tuple[float, float]:
    if hasattr(v, "x") and hasattr(v, "y") and not hasattr(v, "z"):
        return (float(v.x), float(v.y))
    seq = tuple(v)  # type: ignore[arg-type]
    if len(seq) != 2:
        raise ValueError(f"expected a 2-vector, got {v!r}")
    return (float(seq[0]), float(seq[1]))


__all__ = ["Testbed", "CallbackFn"]
