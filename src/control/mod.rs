//! Utilities for controlling the trajectories of objects in a non-physical way.

#[cfg(feature = "alloc")]
pub use self::character_controller::{
    CharacterAutostep, CharacterCollision, CharacterLength, EffectiveCharacterMovement,
    KinematicCharacterController,
};
#[cfg(feature = "alloc")]
pub use self::pid_controller::{PdController, PdErrors, PidController};

#[cfg(all(feature = "dim3", feature = "alloc"))]
pub use self::ray_cast_vehicle_controller::{DynamicRayCastVehicleController, Wheel, WheelTuning};

#[cfg(feature = "alloc")]
mod character_controller;

#[cfg(feature = "alloc")]
mod pid_controller;
#[cfg(all(feature = "dim3", feature = "alloc"))]
mod ray_cast_vehicle_controller;
