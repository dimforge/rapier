//! Utilities for controlling the trajectories of objects in a non-physical way.

pub use self::character_controller::{
    CharacterAutostep, CharacterCollision, CharacterLength, EffectiveCharacterMovement,
    KinematicCharacterController,
};
pub use self::ray_cast_vehicle_controller::{DynamicRayCastVehicleController, Wheel, WheelTuning};

mod character_controller;
mod ray_cast_vehicle_controller;
