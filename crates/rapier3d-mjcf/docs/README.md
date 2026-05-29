# `rapier3d-mjcf` — Planning Documents

This directory contains the design and roadmap for `rapier3d-mjcf`, a loader for the
MuJoCo XML (MJCF) format that mirrors the role `rapier3d-urdf` plays for URDF.

The work is split across two crates:

| Crate            | Role                                                                |
| ---------------- | ------------------------------------------------------------------- |
| `mjcf-rs`        | Pure-Rust MJCF parser. Reads XML, resolves `<include>` and          |
|                  | `<default>` inheritance, produces a typed AST. Equivalent of        |
|                  | `urdf-rs`.                                                          |
| `rapier3d-mjcf`  | Converts an `mjcf-rs::Model` into rapier rigid-bodies, colliders,   |
|                  | and joints. Equivalent of `rapier3d-urdf`.                          |

No C bindings — neither crate depends on `libmujoco` or any FFI shim.

## How to read these docs

| File                                     | Audience               |
| ---------------------------------------- | ---------------------- |
| [`01-overview.md`](01-overview.md)       | Anyone — start here.   |
| [`02-coverage-and-mapping.md`](02-coverage-and-mapping.md) | Spec coverage table — full enumeration of MJCF features and their rapier mapping. |
| [`03-mjcf-rs-design.md`](03-mjcf-rs-design.md) | Parser-crate design.   |
| [`04-out-of-scope.md`](04-out-of-scope.md) | Items intentionally not supported. |
| [`phase-1-core-kinematics.md`](phase-1-core-kinematics.md) | First implementation milestone. |
| [`phase-2-defaults-and-meshes.md`](phase-2-defaults-and-meshes.md) | Class inheritance, `<include>`, mesh assets. |
| [`phase-3-contact-filtering-and-equality.md`](phase-3-contact-filtering-and-equality.md) | `<contact>`, `<equality connect/weld>`. |
| [`phase-4-springs-dampers-armature.md`](phase-4-springs-dampers-armature.md) | Joint dynamics: spring, damping, armature, ref. |
| [`phase-5-actuators-sensors-keyframes.md`](phase-5-actuators-sensors-keyframes.md) | Actuator/sensor pass-through, keyframes. |

## Implementation order

The phases are ordered so each one delivers a working, testable subset:

1. **Phase 1** — Core kinematics: nested bodies, joints (hinge/slide/ball/free), primitive
   geoms (box/sphere/capsule/cylinder/ellipsoid/plane), inertial frames, basic
   `<compiler>` (`angle`, `eulerseq`).
2. **Phase 2** — Default-class inheritance (`<default>`), `<include>`, mesh assets
   (STL/OBJ via `rapier3d-meshloader`), `compiler/inertiafromgeom`,
   `compiler/meshdir`, `compiler/autolimits`.
3. **Phase 3** — Contact filtering (`<contact><pair>` / `<exclude>`,
   `contype`/`conaffinity`), `<equality connect>` and `<equality weld>`.
4. **Phase 4** — Joint dynamics: `stiffness`, `damping`, `armature`, `ref`,
   `frictionloss`, `springref`, `springdamper`, joint `range`/`limited` edge cases.
5. **Phase 5** — `<keyframe>` initial state, optional pass-through of
   `<actuator>` and `<sensor>` metadata for downstream code (rapier doesn't simulate
   them, but consumers may).

Stretch / out-of-scope items are listed in [`04-out-of-scope.md`](04-out-of-scope.md).
