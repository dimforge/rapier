# Phase 4 — Joint Dynamics: Springs, Dampers, Armature, Friction

**Goal:** Real robot models in MJCF rely heavily on `damping`, `armature`,
`stiffness`, `springref`, and `frictionloss` for stable simulation. Without
them, the model is qualitatively wrong: arms flop, knees bounce, fingers
tremble. This phase translates these into rapier-native counterparts.

## Scope

### `<joint>` dynamics attributes

| Attribute       | Rapier mapping                                                 |
| --------------- | -------------------------------------------------------------- |
| `damping`       | `JointMotor` with target velocity 0 and `damping = damping`. Uses MotorModel::ForceBased. |
| `stiffness`     | `JointMotor` with target position = `springref` and `stiffness = stiffness`. Uses ForceBased model so the motor acts as a PD spring. |
| `springref`     | Reference position used by the stiffness motor. Default = 0. |
| `springdamper`  | `(timeconst, dampratio)` — convert to `(stiffness, damping)` via the standard MuJoCo formula and apply same as above. |
| `armature`      | Add `armature` to the diagonal of the link's inertia along the joint axis (in the joint's local frame). For axes aligned with body principal axes, this is exact; otherwise we approximate by adding `armature * outer(axis, axis)` to the inertia tensor. |
| `frictionloss`  | Velocity-based motor with target 0 and capped force = `frictionloss`. Approximates dry Coulomb friction; documented as lossy. |
| `ref`           | "Joint zero" reference. Bake into the joint's local frames so that
                    rapier reports `joint_position = 0` exactly when MJCF would. |
| `actuatorfrcrange` / `actuatorfrclimited` | Pass-through metadata; surfaces on `MjcfJoint` so user-driven motors can clamp themselves. |

### `<inertial>` defaulting / clamps

- `compiler/boundmass` and `compiler/boundinertia` clamps applied here too
  (already wired in Phase 2 for derived inertia; in Phase 4 we also apply
  them to user-supplied `<inertial>` values so behavior matches MuJoCo).
- `compiler/balanceinertia="true"` adjusts the inertia diagonal so it
  satisfies the triangle inequality. Implement the same algorithm
  (`I_xx = max(I_xx, |I_yy - I_zz|)`, etc.).

### `<body>` `gravcomp`

- `gravcomp ∈ {0, 1}` → use `RigidBody::set_gravity_scale(1.0 - gravcomp)`.
- Fractional values: emit a per-step external force equal to
  `mass * gravity * gravcomp`. Surface a helper:

```rust
impl MjcfRobotHandles<...> {
    pub fn apply_gravity_compensation(&self, bodies: &mut RigidBodySet, gravity: Vector);
}
```

The user calls it once per step before `physics_pipeline.step()`.

## Non-goals (deferred)

- `solref`/`solimp` softness for joint limits and contacts — still recorded
  only. Translating MuJoCo's two-parameter spring model into rapier's
  per-axis limit softness is possible but loss-y; we don't promise it.
- `condim 6` (torsional + rolling friction) translation — would need
  contact-mod hooks, possibly Phase 4.5.
- Faithful MuJoCo "smooth" friction-loss model — we use a capped motor.

## Tests

- A pendulum with `damping=0.5`. Compare the decay envelope to a
  hand-tuned rapier scene.
- A revolute joint with `springref=π/4 stiffness=10 damping=1`. Verify
  the resting angle equals `π/4` and the natural frequency is correct.
- A robot finger with `armature=0.001` per joint. Verify simulation is
  stable at a 5 ms timestep where it diverges without armature.
- A box on a slope with a hinge joint at its top, `frictionloss=0.5`.
  Verify the joint angle stays within ±5° of upright while the box stays
  put under gravity.

## Acceptance criteria

- The `mujoco_menagerie` models that previously loaded but simulated badly
  (typically humanoids and dexterous hands) now produce stable, physically
  reasonable trajectories at the same timestep MuJoCo uses.
- `damping`, `stiffness`, `springref`, `springdamper`, `armature`,
  `frictionloss`, `ref` all have unit-test coverage with assertions on the
  resulting rapier state.
