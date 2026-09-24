# Rapier C testbed

The viewer uses raylib for graphics and Dear ImGui through cimgui for its C UI.
CMake downloads pinned sources and Fira Sans Regular during the first configuration.
A C/C++ compiler, Cargo, CMake 3.25+, and an internet connection for that initial
download are required. Subsequent builds reuse the downloaded sources.
macOS uses system frameworks. Linux requires the usual X11/OpenGL development
packages needed to compile raylib's bundled GLFW. See [dependencies.md](dependencies.md) for versions, offline builds, and licenses.

From the repository root:

```sh
cmake -S bindings/c -B build/c3 -DRAPIER_BUILD_TESTBED=ON -DRAPIER_DIMENSION=3 \
  -DRAPIER_PROFILE=release -DCMAKE_BUILD_TYPE=Release
cmake --build build/c3 --config Release --parallel
build/c3/testbed/rapier_testbed --example restitution3
```

With a multi-configuration generator, the executable is under `testbed/Release/`.
Use a separate build directory and `-DRAPIER_DIMENSION=2` for 2D; use
`-DRAPIER_PRECISION=64` for double precision. The visible **Physics: Release/Debug**
label is read from the loaded Rapier library's `r3BuildProfile` function.
`RAPIER_PROFILE` selects Cargo's profile; `CMAKE_BUILD_TYPE` / `--config` selects
the C/C++ build mode. They are independent. The workspace development profile
currently has `opt-level=1`; the label still correctly identifies it as Debug.

The header also reports **SIMD lanes**, **parallel support**, and the actual worker
count from the loaded library. Build options are `-DRAPIER_ENABLE_PARALLEL=ON|OFF`
(default ON for the testbed) and `-DRAPIER_SIMD_LANES=4|8` (default 4). This branch
always uses SIMD; eight lanes require f32 and cannot be combined with
`enhanced-determinism`. See [binding build options](../README.md#build-the-library).

Choose workers under **Settings > Execution**, then Apply, or pass `--threads N`
to either executable. `0` selects automatic sizing, `1` uses one worker, and the
viewer accepts explicit counts through 256. The UI shows the resolved worker
count. Changes preserve the running scene and persist through restart, scene
switches, and snapshot restore. Builds without parallel support run serially and
explain how to enable the control.

## Controls

- Examples: grouped selection and search. Prev/Next cycle through matching available
  demos, wrapping at the ends. Scenes requiring disabled build features are hidden.
- Settings: worker count, timestep, gravity, solver iterations, CCD substeps, and scene settings.
  Scene-specific settings and the sleeping option apply on Restart.
- Performance: native physics time per step (the same counter as the Rust UI),
  simulation time per rendered frame with its step count, and draw CPU time.
  The viewer advances one step per rendered frame, like the Rust viewer.
  Simulation includes the example's per-frame logic; compare the **ms/step** value with Rust. Draw CPU excludes the UI and GPU execution.
- Debug: surfaces, wireframes, body axes, contacts, joints, soft constraints/stress.
- T: play/pause. S: one step. R: restart. F: frame the simulation.
- Left drag: pull a dynamic object with a spring joint, including articulated links
  and individual soft-body particles. Release removes the temporary joint and anchor.
  Dragging needs the simulation running (or single-stepping) to move the object.
- Right drag: arc-ball orbit about the camera target in 3D, or pan in 2D.
  Shift + right drag or middle drag: pan in 3D. Mouse wheel: zoom.
- Deformable polylines use three-pixel ribbons. Sensor colliders are translucent,
  sorted behind-to-front with depth writes disabled during their render pass.
- Arrows and Enter: example controls. Space/Right Ctrl: character up/down; Shift: slow movement.
  Hold C: cut in the cutting demo. Space adds a voxel in the 3D voxel demo; Left Shift + Space removes it. Text entry and focused UI widgets capture
  keyboard input, and UI interactions do not control the scene camera.
- Save/Restore: in-memory physics snapshots. Disabled for scenes with local animation state
  because physics snapshots do not serialize those C variables.

The UI uses automatic layout and scrollable panels, keyboard navigation, and a
TrueType font rasterized at the display's framebuffer density. `--ui-tab Settings`
(or `Examples`, `Performance`, `Debug`) chooses the initial tab.

## Headless and smoke runs

```sh
build/c3/testbed/rapier_testbed_headless --list
build/c3/testbed/rapier_testbed_headless --example restitution3 --steps 120 --no-sleep --threads 1
build/c3/testbed/rapier_testbed --example restitution3 --frames 90 --screenshot preview.png
ctest --test-dir build/c3 -C Release --output-on-failure
```

The `testbed_ui_input` CTest case exercises real ImGui text-input capture and
simulation shortcut routing, example navigation, and orbit-camera geometry using a
null renderer; it needs no display or GPU. `testbed_mouse_grab` checks rigid and soft
spring dragging, fixed/sensor exclusion, temporary-cluster cleanup, and deletion of
a grabbed object.
The C ABI test checks the library-reported profile and execution features against
CMake's options. `testbed_threading` verifies live worker changes and persistence
across restart, scene switches, and snapshot restore, including serial builds.
[Manual C-versus-Rust timing checks](tools/README.md) replay identical no-sleep
snapshots with both APIs and compare the final poses and velocities exactly.

Configure `-DRAPIER_TESTBED_GRAPHICS=OFF` to build only the headless runner without
raylib, ImGui, or display requirements. Viewer and headless runner use the same C
scene sources. `--assets PATH` overrides the repository asset directory.

## Example source

Each scene uses the Rapier C API directly, following its matching Rust example.
Each example owns its render loop, physics stepping, events, and local animation
state. The viewer renders one frame and processes input when the example calls
`tbRenderFrame`; it does not wrap physics construction or stepping.

## Port coverage

All **202 Rust catalog entries** have matching C ports: **88 in 2D and 114 in 3D**.
`coverage.json` records the correspondence; `update_catalog.py` regenerates the
registries from the Rust catalog and C sources. Compile-time feature requirements
remain explicit in the example picker.
`--all` reports passed, failed, and unavailable counts separately; unavailable
scenes are not validation passes. The UI covers the controls described above;
advanced Rust profiler and internal physics-counter panels are not yet ported.


## Optional examples

Enable the native FEM solver with `-DRAPIER_FEATURES=fem`. For 3D/f32, use
`-DRAPIER_FEATURES=fem,robotics` to include the URDF, MJCF, and MuJoCo Menagerie
examples too. Robotics is limited to 3D/f32 because the Rust loader crates have
that same restriction. These features add Rust dependencies through Cargo, not
extra C dependency installation steps.

The URDF and Cassie MJCF examples use the repository's `assets/3d` files.
Menagerie discovers `<robot>/scene*.xml` in `../mujoco_menagerie`; set
`RAPIER_MENAGERIE_DIR` to use another checkout. It exposes model and keyframe
pickers, both joint representations, collision and spring switches, and live
actuator strength. Render meshes retain their colors, UVs, smooth normals,
textures, and material parameters; the lightweight raylib lighting differs
from the Rust viewer's rendering.

The deserialization debug example requires `snapshotN.bincode` files in
`RAPIER_SNAPSHOT_DIR` (the Rust example's original directory is the fallback).
These are the legacy rigid-state files from the identical Rust build, not the
C binding's tagged whole-world snapshots. Missing or incompatible files report
an error; they do not count as a validated scene.

OBJ models are read by a small example utility. The 2D logo's checked-in mesh
uses the Rust example's SVG tessellator, so there is no SVG dependency at runtime.
To regenerate it after editing the source logo:

```sh
cargo run -p rapier-c-logo-mesh > bindings/c/testbed/examples2d/utils/logo_mesh.h
```

`testbed_interactions` drives the actual C example loops with synthetic input:
IK target convergence, switching from kinematic to PID control, and 3D voxel
addition/removal. It runs without a graphics context.

The `testbed_soft_render` regression test traverses and draws the soft-mesh
geometry with recorded draw calls, without a window or GPU. It covers the
cluster/skin demos, bounds cache allocations, and checks that back-face culling
stays disabled until raylib flushes queued mesh triangles. All mesh surfaces are
rendered two-sided.
