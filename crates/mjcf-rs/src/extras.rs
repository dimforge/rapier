//! Pass-through types: `<actuator>`, `<sensor>`, `<keyframe>`.

/// `<actuator>` element. Stores enough information for the loader to wire
/// it up to a rapier joint motor; the simulation itself is up to the user.
#[derive(Clone, Debug, Default)]
pub struct Actuator {
    /// `name` attribute.
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// Subtype (`motor`, `position`, `velocity`, …).
    pub kind: ActuatorKind,
    /// Joint name this actuator drives, if any.
    pub joint: Option<String>,
    /// Tendon name. Tendons are out of scope; recorded only.
    pub tendon: Option<String>,
    /// Body name (for `<actuator><adhesion>`-like actuators). Recorded only.
    pub body: Option<String>,
    /// Site (for `<actuator><site>`-like actuators). Recorded only.
    pub site: Option<String>,
    /// Gear ratio (default 1.0). Length 6 for spatial gears; we keep all 6.
    pub gear: [f64; 6],
    /// `ctrlrange` if specified.
    pub ctrl_range: Option<[f64; 2]>,
    /// `forcerange` if specified.
    pub force_range: Option<[f64; 2]>,
    /// `ctrllimited`.
    pub ctrl_limited: crate::types::Tristate,
    /// `forcelimited`.
    pub force_limited: crate::types::Tristate,
    /// `gainprm` (up to 10 entries).
    pub gainprm: Vec<f64>,
    /// `biasprm` (up to 10 entries).
    pub biasprm: Vec<f64>,
    /// `gaintype` (`fixed`, `affine`, `muscle`, …). `None` ⇒ MJCF default
    /// (`fixed`). Used to interpret `<general>` actuators.
    pub gain_type: Option<String>,
    /// `biastype` (`none`, `affine`, `muscle`, …). `None` ⇒ MJCF default
    /// (`none`). An `affine` bias turns a `<general>` actuator into a
    /// position/velocity servo: `bias = biasprm0 + biasprm1·q + biasprm2·q̇`.
    pub bias_type: Option<String>,
    /// `dyntype`.
    pub dyn_type: Option<String>,
    /// `dynprm`.
    pub dynprm: Vec<f64>,
    /// `kp` for `<position>` actuators.
    pub kp: Option<f64>,
    /// `kv` for `<velocity>` / `<damper>` actuators.
    pub kv: Option<f64>,
}

/// `<actuator>` subtype.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum ActuatorKind {
    /// `<motor>` — direct force/torque actuator.
    #[default]
    Motor,
    /// `<position>`.
    Position,
    /// `<velocity>`.
    Velocity,
    /// `<intvelocity>`.
    IntVelocity,
    /// `<damper>`.
    Damper,
    /// `<general>` (most general form).
    General,
    /// Anything else (cylinder/muscle/adhesion/plugin) — recorded but not driven.
    Other,
}

/// `<sensor>` element. Recorded for downstream code.
#[derive(Clone, Debug, Default)]
pub struct Sensor {
    /// `name` attribute.
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// MJCF subtype name (`accelerometer`, `gyro`, `jointpos`, …).
    pub kind: String,
    /// Object reference type (`body`, `geom`, `site`, `joint`, …).
    pub objtype: Option<String>,
    /// Object name reference.
    pub objname: Option<String>,
    /// Reference body (for relative sensors).
    pub reftype: Option<String>,
    /// Reference body name.
    pub refname: Option<String>,
    /// `cutoff` clamp.
    pub cutoff: Option<f64>,
    /// `noise` standard deviation.
    pub noise: Option<f64>,
}

/// One `<keyframe><key>` entry.
#[derive(Clone, Debug, Default)]
pub struct Keyframe {
    /// `name` attribute.
    pub name: Option<String>,
    /// `time` attribute.
    pub time: f64,
    /// `qpos` array (one entry per generalized coordinate).
    pub qpos: Vec<f64>,
    /// `qvel` array.
    pub qvel: Vec<f64>,
    /// `act` actuator activation array.
    pub act: Vec<f64>,
    /// `ctrl` controller setpoint array.
    pub ctrl: Vec<f64>,
    /// `mpos` mocap-body positions (3 per mocap body).
    pub mpos: Vec<f64>,
    /// `mquat` mocap-body orientations (4 per mocap body).
    pub mquat: Vec<f64>,
}
