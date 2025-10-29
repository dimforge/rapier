## 0.4.0

### Modified

- Pre-parsing of urdf files is now done through the more recent `urdf_rs` crate.

### Added

- Add optional support for Collada and Wavefront files through new feature flags `collada` and `wavefront`.
- Add support for capsule urdf geometry

## 0.3.0

This is the initial release of the `rapier3d-urdf` crate.

### Added

- Add `UrdfRobot` which is a collection of colliders, rigid-bodies and joints representing a robot loaded from an URDF
  file.
- Add `UrdfRobot::from_file` to load an `UrdfRobot` from an URDF file.
- Add `UrdfRobot::from_str` to load an `UrdfRobot` from a string in URDF format.
- Add `UrdfRobot::from_robot` to load an `UrdfRobot` from an already loaded URDF
  robot (pre-parsed with the `xurdf` crate).
- Add `UrdfRobot::insert_using_impulse_joints` to insert the robot to the rapier sets. Joints are represented as
  **impulse** joints.
- Add `UrdfRobot::insert_using_impulse_joints` to insert the robot to the rapier sets. Joints are represented as
  **multibody** joints.
