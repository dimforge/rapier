# 04 — Out of scope

These features are intentionally not on the roadmap. They are listed here so
contributors don't waste time scoping them, and so users have an authoritative
list of what `rapier3d-mjcf` won't model.

## Hard non-goals

| Feature                                  | Why                                                                  |
| ---------------------------------------- | -------------------------------------------------------------------- |
| `<extension>` / MuJoCo plugins            | Plugins are MuJoCo-runtime concepts; replicating the plugin ABI is out of scope. |
| `<flexcomp>` / `<flex>` / `<skin>`        | Soft body / FEM / skinned-mesh; rapier doesn't simulate continuum mechanics. |
| `<composite>`                             | Macro that expands to clusters of bodies (rope, cloth, particles). Outside the kinematic-rigid-body scope. |
| `<tendon>` (`spatial` and `fixed`)        | Path-routed cables and linear joint-coordinate combinations have no first-class rapier object. A user can model approximations manually. |
| `<equality joint>`                        | Polynomial joint coupling — not natively supported by rapier. |
| `<equality tendon>`                       | Tendon-length coupling — depends on tendons. |
| `<actuator cylinder>` / `muscle` / `adhesion` | Specialty actuators with non-trivial dynamics; users can wire them up themselves on top of the metadata pass-through. |
| `<sensor>` types that need contact-area integration (`touch`, `rangefinder`, `geomdist`, `camprojection`) | Each of these would require new rapier APIs. The other sensors are derivable from rapier state and *will* be supported by helpers. |
| `<geom type="sdf">`                       | Rapier has no signed-distance-field shape. |
| `<visual>`, `<asset texture>`, `<asset material>` actually rendered | We expose them as metadata; consumers do their own rendering. |
| Round-tripping (load → edit → write MJCF) | The parser is read-only on the v1 roadmap. |

## Soft / "later, maybe" items

| Feature                              | Notes                                                                  |
| ------------------------------------ | ---------------------------------------------------------------------- |
| `<attach>` cross-file body grafting   | Replaceable with `<include>` in practice. Could be added if real models need it. |
| MJCF write-back from `mjcf-rs`        | Would unlock model editing tools. Mechanically straightforward but a lot of attribute serialization. |
| Tendon-length equality via custom rapier constraint | Possible if the rapier constraint API ever exposes user-written linear constraints. |
| Closed-loop `<equality joint>` via penalty constraints | Same as above. |
| Faithful `condim` 1/4/6 contact dims  | Rapier always uses 3D contacts; condim 1 (frictionless) and 6 (full torsion) would need contact-modification hooks. |

## What we approximate (lossy mappings)

These *are* on the roadmap, but the user should be aware the mapping is not
exact. The loader logs a warning the first time each approximation fires.

| MJCF feature                | Rapier mapping                                            | Loss / discrepancy |
| --------------------------- | --------------------------------------------------------- | ------------------ |
| `<geom type="ellipsoid">`   | Convex hull of an icosphere scaled by `size`             | Approximate; degenerates to a ball when sizes are equal. |
| Joint `frictionloss`        | Velocity-based motor with capped force                    | Not exactly Coulomb friction but a close practical match. |
| Joint `armature`            | Diagonal addition to the link's principal inertia along the joint axis | Equivalent for hinge/slide axes aligned with body principal axes; lossy otherwise. |
| `gravcomp`                  | `RigidBody::gravity_scale` clamp / per-step gravity force | Exact when `gravcomp ∈ {0, 1}`; in-between values use a per-body external force. |
| `<contact pair>` `solref` / `solimp` | Ignored                                       | Friction/restitution are mapped; the soft-contact spring/damper is not. |
| MJCF `<option integrator>` choice | Ignored — rapier has its own integrator         | Behavior may diverge for stiff systems. |
