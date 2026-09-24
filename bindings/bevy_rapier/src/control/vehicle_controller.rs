use crate::control::ControllerFilterPredicate;
use crate::geometry::CollisionGroups;
use crate::math::{Real, Vect};
use crate::reflect::WheelTuningWrapper;
use bevy::ecs::entity::EntityHashSet;
use bevy::prelude::*;
use rapier::control::DynamicRayCastVehicleController;
use rapier::prelude::QueryFilterFlags;

pub use rapier::control::WheelTuning;

/// The state of a [`VehicleWheel`] computed by the plugin at the last vehicle update.
///
/// World-space quantities are expressed relative to the chassis pose before the last
/// simulation step.
#[derive(Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Default, PartialEq, Debug)]
pub struct VehicleWheelState {
    /// The wheel's current rotation angle (in radians) around its axle.
    ///
    /// This increases when the vehicle rolls along the wheel's forward direction
    /// (`contact_normal × axle`).
    pub rotation: Real,
    /// The current length of the wheel's suspension.
    pub suspension_length: Real,
    /// Is the wheel in contact with the ground?
    pub is_in_contact: bool,
    /// The world-space point hit by the wheel's ray-cast.
    pub contact_point: Vect,
    /// The world-space contact normal between the wheel and the ground.
    pub contact_normal: Vect,
    /// The world-space starting point of the wheel's ray-cast.
    pub hard_point: Vect,
    /// The world-space center of the wheel.
    pub center: Vect,
    /// The world-space direction of the wheel's suspension.
    pub suspension_direction: Vect,
    /// The world-space direction of the wheel's axle, including steering.
    pub axle_direction: Vect,
    /// The collider entity hit by the wheel's ray-cast, if any.
    pub ground_entity: Option<Entity>,
    /// The force applied by the suspension.
    pub suspension_force: Real,
    /// The forward impulse applied by the wheel on the chassis.
    pub forward_impulse: Real,
    /// The side impulse applied by the wheel on the chassis.
    pub side_impulse: Real,
}

/// A wheel of a [`RayCastVehicleController`].
///
/// The description fields (connection point, directions, lengths, tuning) and the controls
/// (engine force, brake, steering) can be modified at any time. The [`Self::state`] is
/// written by the plugin at each vehicle update.
#[derive(Copy, Clone, Debug, PartialEq, Reflect)]
#[reflect(Default, PartialEq, Debug)]
pub struct VehicleWheel {
    /// The position of the wheel's suspension attachment, in the chassis' local space.
    pub chassis_connection_point: Vect,
    /// The direction of the wheel's suspension, in the chassis' local space.
    ///
    /// The ray-cast detecting the ground follows this direction.
    pub direction: Vect,
    /// The wheel's axle axis, in the chassis' local space.
    pub axle: Vect,
    /// The rest length of the wheel's suspension spring.
    pub suspension_rest_length: Real,
    /// The wheel's radius.
    pub radius: Real,
    /// Parameters affecting the physical behavior of the wheel.
    #[reflect(remote = WheelTuningWrapper)]
    pub tuning: WheelTuning,
    /// The forward force applied by this wheel on the chassis.
    pub engine_force: Real,
    /// The maximum braking impulse applied by this wheel to slow down the vehicle.
    pub brake: Real,
    /// The steering angle (in radians) of this wheel, around the suspension direction.
    pub steering: Real,
    /// The state of the wheel, written back by the plugin.
    pub state: VehicleWheelState,
}

impl Default for VehicleWheel {
    fn default() -> Self {
        Self {
            chassis_connection_point: Vect::ZERO,
            direction: -Vect::Y,
            axle: Vect::Z,
            suspension_rest_length: 0.5,
            radius: 0.5,
            tuning: WheelTuning::default(),
            engine_force: 0.0,
            brake: 0.0,
            steering: 0.0,
            state: VehicleWheelState::default(),
        }
    }
}

impl VehicleWheel {
    /// Creates a wheel with the given geometry, the given tuning, and no engine force,
    /// brake, or steering.
    pub fn new(
        chassis_connection_point: Vect,
        direction: Vect,
        axle: Vect,
        suspension_rest_length: Real,
        radius: Real,
        tuning: WheelTuning,
    ) -> Self {
        Self {
            chassis_connection_point,
            direction,
            axle,
            suspension_rest_length,
            radius,
            tuning,
            ..Default::default()
        }
    }

    /// The wheel's transform relative to the chassis, for rendering.
    ///
    /// The translation is the wheel center at the current suspension length, and the rotation
    /// combines steering (around the suspension direction) and spin (around the axle).
    pub fn local_transform(&self) -> Transform {
        let translation =
            self.chassis_connection_point + self.direction * self.state.suspension_length;
        let steering = Quat::from_axis_angle(-self.direction.normalize_or_zero(), self.steering);
        // Rolling forward along `normal × axle` is a negative rotation around the axle.
        let spin = Quat::from_axis_angle(self.axle.normalize_or_zero(), -self.state.rotation);
        Transform::from_translation(translation).with_rotation(steering * spin)
    }

    pub(crate) fn sync_to_raw(&self, raw: &mut rapier::control::Wheel) {
        raw.chassis_connection_point_cs = self.chassis_connection_point;
        raw.direction_cs = self.direction;
        raw.axle_cs = self.axle;
        raw.suspension_rest_length = self.suspension_rest_length;
        raw.radius = self.radius;
        raw.suspension_stiffness = self.tuning.suspension_stiffness;
        raw.damping_compression = self.tuning.suspension_compression;
        raw.damping_relaxation = self.tuning.suspension_damping;
        raw.max_suspension_travel = self.tuning.max_suspension_travel;
        raw.side_friction_stiffness = self.tuning.side_friction_stiffness;
        raw.friction_slip = self.tuning.friction_slip;
        raw.max_suspension_force = self.tuning.max_suspension_force;
        raw.engine_force = self.engine_force;
        raw.brake = self.brake;
        raw.steering = self.steering;
    }
}

/// A vehicle controller simulating wheels with ray-casts, attached to the chassis rigid-body.
///
/// Before each simulation step, the plugin casts one ray per wheel along its suspension
/// direction, then applies suspension, engine, braking and friction impulses to the chassis,
/// which must be a dynamic rigid-body on the same entity. Colliders attached to the chassis
/// rigid-body are always ignored by the ray-casts. The results are written back into each
/// wheel's [`VehicleWheel::state`] and into [`Self::current_vehicle_speed`].
///
/// The impulses are computed once per plugin update using the last simulation timestep, so
/// this is best used with a fixed timestep.
#[derive(Clone, Debug, Component, Reflect)]
#[reflect(Component, Default, Debug)]
pub struct RayCastVehicleController {
    /// The wheels of this vehicle.
    ///
    /// Wheels can be added, removed or modified at any time.
    pub wheels: Vec<VehicleWheel>,
    /// The chassis' local up axis (`0 = x, 1 = y, 2 = z`).
    pub index_up_axis: usize,
    /// The chassis' local forward axis (`0 = x, 1 = y, 2 = z`).
    ///
    /// This is only used to determine the sign of [`Self::current_vehicle_speed`].
    pub index_forward_axis: usize,
    /// Flags for excluding some categories of colliders from the wheels' ray-casts.
    #[reflect(ignore)]
    pub filter_flags: QueryFilterFlags,
    /// Groups for excluding some colliders from the wheels' ray-casts.
    pub filter_groups: Option<CollisionGroups>,
    /// Colliders (identified by their entity) ignored by the wheels' ray-casts.
    pub exclude_colliders: EntityHashSet,
    /// Rigid-bodies (identified by their entity) whose colliders are all ignored by the
    /// wheels' ray-casts.
    pub exclude_rigid_bodies: EntityHashSet,
    /// If set, any collider for which this predicate returns `false` is ignored by the wheels'
    /// ray-casts. The colliders with a [`ControllerIgnored`](crate::control::ControllerIgnored)
    /// component (or attached to a rigid-body with one) are always ignored.
    #[reflect(ignore)]
    pub filter_predicate: Option<ControllerFilterPredicate>,
    /// The current forward speed of the vehicle, written back by the plugin.
    ///
    /// It is negative if the vehicle moves backward along its forward axis.
    pub current_vehicle_speed: Real,
    /// The underlying Rapier controller, created and synchronized automatically by the plugin.
    ///
    /// Its wheels are overwritten from [`Self::wheels`] before each update.
    #[reflect(ignore)]
    pub raw: Option<DynamicRayCastVehicleController>,
}

impl Default for RayCastVehicleController {
    fn default() -> Self {
        Self {
            wheels: vec![],
            index_up_axis: 1,
            index_forward_axis: 0,
            filter_flags: QueryFilterFlags::EXCLUDE_SENSORS,
            filter_groups: None,
            exclude_colliders: EntityHashSet::default(),
            exclude_rigid_bodies: EntityHashSet::default(),
            filter_predicate: None,
            current_vehicle_speed: 0.0,
            raw: None,
        }
    }
}

impl RayCastVehicleController {
    /// Creates a vehicle controller with the given wheels.
    pub fn new(wheels: Vec<VehicleWheel>) -> Self {
        Self {
            wheels,
            ..Default::default()
        }
    }

    /// Adds a wheel to this vehicle.
    pub fn add_wheel(&mut self, wheel: VehicleWheel) -> &mut Self {
        self.wheels.push(wheel);
        self
    }

    /// Is any wheel of this vehicle in contact with the ground?
    pub fn any_wheel_in_contact(&self) -> bool {
        self.wheels.iter().any(|w| w.state.is_in_contact)
    }
}
