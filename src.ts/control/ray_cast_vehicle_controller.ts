import {RawDynamicRayCastVehicleController} from "../raw";
import {scratchBuffer, Vector, VectorOps} from "../math";
import {
    BroadPhase,
    Collider,
    ColliderSet,
    InteractionGroups,
    NarrowPhase,
} from "../geometry";
import {QueryFilterFlags} from "../pipeline";
import {RigidBody, RigidBodyHandle, RigidBodySet} from "../dynamics";

/**
 * A character controller to simulate vehicles using ray-casting for the wheels.
 */
export class DynamicRayCastVehicleController {
    private raw: RawDynamicRayCastVehicleController;
    private broadPhase: BroadPhase;
    private narrowPhase: NarrowPhase;
    private bodies: RigidBodySet;
    private colliders: ColliderSet;
    private _chassis: RigidBody;

    constructor(
        chassis: RigidBody,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
    ) {
        this.raw = new RawDynamicRayCastVehicleController(chassis.handle);
        this.broadPhase = broadPhase;
        this.narrowPhase = narrowPhase;
        this.bodies = bodies;
        this.colliders = colliders;
        this._chassis = chassis;
    }

    /** @internal */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }

        this.raw = undefined;
    }

    /**
     * Updates the vehicle’s velocity based on its suspension, engine force, and brake.
     *
     * This directly updates the velocity of its chassis rigid-body.
     *
     * @param dt - Time increment used to integrate forces.
     * @param filterFlags - Flag to exclude categories of objects from the wheels’ ray-cast.
     * @param filterGroups - Only colliders compatible with these groups will be hit by the wheels’ ray-casts.
     * @param filterPredicate - Callback to filter out which collider will be hit by the wheels’ ray-casts.
     */
    public updateVehicle(
        dt: number,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.raw.update_vehicle(
            dt,
            this.broadPhase.raw,
            this.narrowPhase.raw,
            this.bodies.raw,
            this.colliders.raw,
            filterFlags,
            filterGroups,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * The current forward speed of the vehicle.
     */
    public currentVehicleSpeed(): number {
        return this.raw.current_vehicle_speed();
    }

    /**
     * The rigid-body used as the chassis.
     */
    public chassis(): RigidBody {
        return this._chassis;
    }

    /**
     * The chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`).
     */
    get indexUpAxis(): number {
        return this.raw.index_up_axis();
    }

    /**
     * Sets the chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`).
     */
    set indexUpAxis(axis: number) {
        this.raw.set_index_up_axis(axis);
    }

    /**
     * The chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`).
     */
    get indexForwardAxis(): number {
        return this.raw.index_forward_axis();
    }

    /**
     * Sets the chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`).
     */
    set setIndexForwardAxis(axis: number) {
        this.raw.set_index_forward_axis(axis);
    }

    /**
     * Adds a new wheel attached to this vehicle.
     * @param chassisConnectionCs  - The position of the wheel relative to the chassis.
     * @param directionCs - The direction of the wheel’s suspension, relative to the chassis. The ray-casting will
     *                      happen following this direction to detect the ground.
     * @param axleCs - The wheel’s axle axis, relative to the chassis.
     * @param suspensionRestLength - The rest length of the wheel’s suspension spring.
     * @param radius - The wheel’s radius.
     */
    public addWheel(
        chassisConnectionCs: Vector,
        directionCs: Vector,
        axleCs: Vector,
        suspensionRestLength: number,
        radius: number,
    ) {
        let rawChassisConnectionCs = VectorOps.intoRaw(chassisConnectionCs);
        let rawDirectionCs = VectorOps.intoRaw(directionCs);
        let rawAxleCs = VectorOps.intoRaw(axleCs);

        this.raw.add_wheel(
            rawChassisConnectionCs,
            rawDirectionCs,
            rawAxleCs,
            suspensionRestLength,
            radius,
        );

        rawChassisConnectionCs.free();
        rawDirectionCs.free();
        rawAxleCs.free();
    }

    /**
     * The number of wheels attached to this vehicle.
     */
    public numWheels(): number {
        return this.raw.num_wheels();
    }

    /*
     *
     * Access to wheel properties.
     *
     */
    /*
     * Getters + setters
     */

    /**
     * The position of the i-th wheel, relative to the chassis.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelChassisConnectionPointCs(
        i: number,
        target?: Vector,
    ): Vector | null {
        const exists = this.raw.wheel_chassis_connection_point_cs(
            i,
            scratchBuffer,
        );
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     * Sets the position of the i-th wheel, relative to the chassis.
     */
    public setWheelChassisConnectionPointCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_chassis_connection_point_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * The rest length of the i-th wheel’s suspension spring.
     */
    public wheelSuspensionRestLength(i: number): number | null {
        return this.raw.wheel_suspension_rest_length(i);
    }

    /**
     * Sets the rest length of the i-th wheel’s suspension spring.
     */
    public setWheelSuspensionRestLength(i: number, value: number) {
        this.raw.set_wheel_suspension_rest_length(i, value);
    }

    /**
     * The maximum distance the i-th wheel suspension can travel before and after its resting length.
     */
    public wheelMaxSuspensionTravel(i: number): number | null {
        return this.raw.wheel_max_suspension_travel(i);
    }

    /**
     * Sets the maximum distance the i-th wheel suspension can travel before and after its resting length.
     */
    public setWheelMaxSuspensionTravel(i: number, value: number) {
        this.raw.set_wheel_max_suspension_travel(i, value);
    }

    /**
     * The i-th wheel’s radius.
     */
    public wheelRadius(i: number): number | null {
        return this.raw.wheel_radius(i);
    }

    /**
     * Sets the i-th wheel’s radius.
     */
    public setWheelRadius(i: number, value: number) {
        this.raw.set_wheel_radius(i, value);
    }

    /**
     * The i-th wheel’s suspension stiffness.
     *
     * Increase this value if the suspension appears to not push the vehicle strong enough.
     */
    public wheelSuspensionStiffness(i: number): number | null {
        return this.raw.wheel_suspension_stiffness(i);
    }

    /**
     * Sets the i-th wheel’s suspension stiffness.
     *
     * Increase this value if the suspension appears to not push the vehicle strong enough.
     */
    public setWheelSuspensionStiffness(i: number, value: number) {
        this.raw.set_wheel_suspension_stiffness(i, value);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being compressed.
     */
    public wheelSuspensionCompression(i: number): number | null {
        return this.raw.wheel_suspension_compression(i);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being compressed.
     */
    public setWheelSuspensionCompression(i: number, value: number) {
        this.raw.set_wheel_suspension_compression(i, value);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being released.
     *
     * Increase this value if the suspension appears to overshoot.
     */
    public wheelSuspensionRelaxation(i: number): number | null {
        return this.raw.wheel_suspension_relaxation(i);
    }

    /**
     * Sets the i-th wheel’s suspension’s damping when it is being released.
     *
     * Increase this value if the suspension appears to overshoot.
     */
    public setWheelSuspensionRelaxation(i: number, value: number) {
        this.raw.set_wheel_suspension_relaxation(i, value);
    }

    /**
     * The maximum force applied by the i-th wheel’s suspension.
     */
    public wheelMaxSuspensionForce(i: number): number | null {
        return this.raw.wheel_max_suspension_force(i);
    }

    /**
     * Sets the maximum force applied by the i-th wheel’s suspension.
     */
    public setWheelMaxSuspensionForce(i: number, value: number) {
        this.raw.set_wheel_max_suspension_force(i, value);
    }

    /**
     * The maximum amount of braking impulse applied on the i-th wheel to slow down the vehicle.
     */
    public wheelBrake(i: number): number | null {
        return this.raw.wheel_brake(i);
    }

    /**
     * Set the maximum amount of braking impulse applied on the i-th wheel to slow down the vehicle.
     */
    public setWheelBrake(i: number, value: number) {
        this.raw.set_wheel_brake(i, value);
    }

    /**
     * The steering angle (radians) for the i-th wheel.
     */
    public wheelSteering(i: number): number | null {
        return this.raw.wheel_steering(i);
    }

    /**
     * Sets the steering angle (radians) for the i-th wheel.
     */
    public setWheelSteering(i: number, value: number) {
        this.raw.set_wheel_steering(i, value);
    }

    /**
     * The forward force applied by the i-th wheel on the chassis.
     */
    public wheelEngineForce(i: number): number | null {
        return this.raw.wheel_engine_force(i);
    }

    /**
     * Sets the forward force applied by the i-th wheel on the chassis.
     */
    public setWheelEngineForce(i: number, value: number) {
        this.raw.set_wheel_engine_force(i, value);
    }

    /**
     * The direction of the i-th wheel’s suspension, relative to the chassis.
     *
     * The ray-casting will happen following this direction to detect the ground.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelDirectionCs(i: number, target?: Vector): Vector | null {
        const exists = this.raw.wheel_direction_cs(i, scratchBuffer);
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     * Sets the direction of the i-th wheel’s suspension, relative to the chassis.
     *
     * The ray-casting will happen following this direction to detect the ground.
     */
    public setWheelDirectionCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_direction_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * The i-th wheel’s axle axis, relative to the chassis.
     *
     * The axis index defined as 0 = X, 1 = Y, 2 = Z.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelAxleCs(i: number, target?: Vector): Vector | null {
        const exists = this.raw.wheel_axle_cs(i, scratchBuffer);
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     * Sets the i-th wheel’s axle axis, relative to the chassis.
     *
     * The axis index defined as 0 = X, 1 = Y, 2 = Z.
     */
    public setWheelAxleCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_axle_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * Parameter controlling how much traction the tire has.
     *
     * The larger the value, the more instantaneous braking will happen (with the risk of
     * causing the vehicle to flip if it’s too strong).
     */
    public wheelFrictionSlip(i: number): number | null {
        return this.raw.wheel_friction_slip(i);
    }

    /**
     * Sets the parameter controlling how much traction the tire has.
     *
     * The larger the value, the more instantaneous braking will happen (with the risk of
     * causing the vehicle to flip if it’s too strong).
     */
    public setWheelFrictionSlip(i: number, value: number) {
        this.raw.set_wheel_friction_slip(i, value);
    }

    /**
     * The multiplier of friction between a tire and the collider it’s on top of.
     *
     * The larger the value, the stronger side friction will be.
     */
    public wheelSideFrictionStiffness(i: number): number | null {
        return this.raw.wheel_side_friction_stiffness(i);
    }

    /**
     * The multiplier of friction between a tire and the collider it’s on top of.
     *
     * The larger the value, the stronger side friction will be.
     */
    public setWheelSideFrictionStiffness(i: number, value: number) {
        this.raw.set_wheel_side_friction_stiffness(i, value);
    }

    /*
     * Getters only.
     */

    /**
     *  The i-th wheel’s current rotation angle (radians) on its axle.
     */
    public wheelRotation(i: number): number | null {
        return this.raw.wheel_rotation(i);
    }

    /**
     *  The forward impulses applied by the i-th wheel on the chassis.
     */
    public wheelForwardImpulse(i: number): number | null {
        return this.raw.wheel_forward_impulse(i);
    }

    /**
     *  The side impulses applied by the i-th wheel on the chassis.
     */
    public wheelSideImpulse(i: number): number | null {
        return this.raw.wheel_side_impulse(i);
    }

    /**
     *  The force applied by the i-th wheel suspension.
     */
    public wheelSuspensionForce(i: number): number | null {
        return this.raw.wheel_suspension_force(i);
    }

    /**
     *  The (world-space) contact normal between the i-th wheel and the floor.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelContactNormal(i: number, target?: Vector): Vector | null {
        const exists = this.raw.wheel_contact_normal_ws(i, scratchBuffer);
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     *  The (world-space) point hit by the wheel’s ray-cast for the i-th wheel.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelContactPoint(i: number, target?: Vector): Vector | null {
        const exists = this.raw.wheel_contact_point_ws(i, scratchBuffer);
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     *  The suspension length for the i-th wheel.
     */
    public wheelSuspensionLength(i: number): number | null {
        return this.raw.wheel_suspension_length(i);
    }

    /**
     *  The (world-space) starting point of the ray-cast for the i-th wheel.
     *
     * @param {number} i
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public wheelHardPoint(i: number, target?: Vector): Vector | null {
        const exists = this.raw.wheel_hard_point_ws(i, scratchBuffer);
        return exists ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     *  Is the i-th wheel in contact with the ground?
     */
    public wheelIsInContact(i: number): boolean {
        return this.raw.wheel_is_in_contact(i);
    }

    /**
     *  The collider hit by the ray-cast for the i-th wheel.
     */
    public wheelGroundObject(i: number): Collider | null {
        return this.colliders.get(this.raw.wheel_ground_object(i));
    }
}
