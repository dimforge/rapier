# Phase 5 — Actuators, Sensors, and Keyframes

**Goal:** Make MJCF models *useful* for control / RL applications. Rapier
won't simulate actuators or sensors itself, but the loader should give
users everything they need to wire them up to rapier in a few lines.

## Scope

### `<keyframe>` — initial state

```xml
<keyframe>
  <key name="home" qpos="0 0 0.5 1 0 0 0 ..." qvel="..." mpos="..." mquat="..."/>
</keyframe>
```

- Parse all keyframes onto the AST.
- Add an opt-in API:

```rust
impl MjcfRobotHandles<...> {
    pub fn apply_keyframe(
        &self,
        bodies: &mut RigidBodySet,
        joints: &mut MultibodyJointSet,
        impulse_joints: &mut ImpulseJointSet,
        key: &mjcf_rs::Keyframe,
    );
}
```

For multibody chains, this calls `Multibody::apply_displacements` /
`set_link_vel_mut` on the appropriate generalized coordinates. For
impulse-joint chains, we set body positions and velocities one by one.

`mpos`/`mquat` are applied to bodies marked `mocap="true"`.

### `<actuator>` — pass-through with helpers

For each actuator we record:

- `name`, `class`
- `joint=` / `tendon=` / `body=` reference (we honor `joint=` only;
  `tendon=` is unsupported, `body=` is rare and post-processed)
- `gear`, `ctrlrange`, `forcerange`, `ctrllimited`, `forcelimited`
- `dyntype` (`"none"` / `"integrator"` / `"filter"` / `"filterexact"`),
  `dynprm`, `gainprm`, `biasprm`

API surface:

```rust
pub struct MjcfActuator {
    pub name: Option<String>,
    pub kind: MjcfActuatorKind,
    pub gear: Real,
    pub ctrl_range: Option<[Real; 2]>,
    pub force_range: Option<[Real; 2]>,
    pub joint_handle: Option<MultibodyJointHandle>, // resolved at insert time
    pub gain_prm: [Real; 10],
    pub bias_prm: [Real; 10],
    pub dyn_type: MjcfDynType,
    pub dyn_prm: [Real; 10],
}

pub enum MjcfActuatorKind {
    Motor,
    Position,    // setpoint controller (kp = gainprm[0])
    Velocity,    // velocity controller (kv = gainprm[0])
    IntVelocity,
    Damper,
    General,
    Other(String),
}

impl MjcfRobotHandles<...> {
    /// Drive every actuator the loader can recognise, given a control vector
    /// in actuator order, by writing into the corresponding rapier joint motors.
    pub fn apply_controls(
        &self,
        joints: &mut MultibodyJointSet,
        ctrl: &[Real],
    );
}
```

Built-in support:

- `motor`: write `set_motor_velocity(0.0, ctrl * gear, force_range_max)` (force-controlled motor).
- `position`: target position controller — set `set_motor_position(ctrl, kp, kd, _)`.
- `velocity`: target velocity controller — `set_motor_velocity(ctrl, kv, _)`.
- `damper`: zero target, damping = `gainprm[0] * ctrl`.
- `general`: caller-driven. We record the parameters but don't apply.

`<actuator><cylinder>`, `<muscle>`, `<adhesion>` are recorded but not
auto-driven (out of scope per `04-out-of-scope.md`).

### `<sensor>` — readers

Many MuJoCo sensors are simple state queries. We provide derived readings
without storing them on the robot:

```rust
impl MjcfRobotHandles<...> {
    pub fn read_sensor(
        &self,
        sensor: &mjcf_rs::Sensor,
        bodies: &RigidBodySet,
        joints: &MultibodyJointSet,
        gravity: Vector,
    ) -> Option<MjcfSensorValue>;
}

pub enum MjcfSensorValue {
    Scalar(Real),
    Vector3(Vector),
    Quat(Rotation),
}
```

Implementable in Phase 5:

| Sensor          | Reading                                                       |
| --------------- | ------------------------------------------------------------- |
| `accelerometer` | `linear_acceleration_at_point(site)` minus gravity            |
| `velocimeter`   | linear velocity at the site                                   |
| `gyro`          | angular velocity in the site frame                            |
| `force`         | accumulated joint constraint force                            |
| `torque`        | joint constraint torque                                       |
| `jointpos`      | `Multibody::generalized_position(idx)`                        |
| `jointvel`      | `Multibody::generalized_velocity(idx)`                        |
| `framepos`      | site/body world position                                      |
| `framequat`     | site/body world orientation                                   |
| `framelinvel`   | site/body world linear velocity                               |
| `frameangvel`   | site/body world angular velocity                              |
| `subtreecom`    | mass-weighted CoM over the rigid subtree rooted at body       |
| `subtreemass`   | summed mass over the rigid subtree                            |
| `subtreelinvel` | summed `mass * vel` over the rigid subtree, divided by mass   |
| `subtreeangmom` | summed angular momentum about the subtree CoM                  |
| `clock`         | accumulated step time (caller passes it in)                   |

Out of scope (sensors that need contact-area integration or rendering):

- `touch` (would need accumulated normal-impulse area)
- `rangefinder`
- `geomdist` / `geomnormal` / `geomfromto`
- `camprojection`

## Non-goals

- Implicit driving of `<actuator>`s during `physics_pipeline.step()`. Users
  call `apply_controls()` before each step — same model as PyMjcf or
  `mujoco-py`.
- Composite sensors (e.g. averaging multiple body states); the user composes
  these from primitives.

## Tests

- Cartpole with a `<actuator><motor joint="cart_slide">` — feed a constant
  control signal and verify the cart accelerates as expected.
- Humanoid with a `<keyframe name="home">` — apply it, verify all joints
  match the expected angles within 1e-6.
- A revolute joint with a `<sensor jointpos>` — read it back after
  manually setting the joint angle and verify equality.
- A free body with an `<sensor framepos>` and `<sensor framequat>` —
  verify the readings match the rigid-body's position / orientation.

## Acceptance criteria

- A demo example: humanoid model loaded from MJCF, keyframe applied,
  actuator controls driven from a hand-written PD controller using
  `apply_controls()`, simulation runs stably for 10 seconds.
- All sensors listed above have a unit test.
- Documentation includes a "wiring control loops" example end-to-end.
