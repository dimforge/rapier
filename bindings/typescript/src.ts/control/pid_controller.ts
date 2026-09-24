import {RawPidController} from "../raw";
import {Rotation, RotationOps, scratchBuffer, Vector, VectorOps} from "../math";
import {Collider, ColliderSet, InteractionGroups, Shape} from "../geometry";
import {QueryFilterFlags, World} from "../pipeline";
import {IntegrationParameters, RigidBody, RigidBodySet} from "../dynamics";

// TODO: unify with the JointAxesMask
/**
 * An enum representing the possible joint axes controlled by a PidController.
 * They can be ORed together, like:
 * PidAxesMask.LinX || PidAxesMask.LinY
 * to get a pid controller that only constraints the translational X and Y axes.
 *
 * Possible axes are:
 *
 * - `X`: X translation axis
 * - `Y`: Y translation axis
 * - `Z`: Z translation axis
 * - `AngX`: X angular rotation axis (3D only)
 * - `AngY`: Y angular rotation axis (3D only)
 * - `AngZ`: Z angular rotation axis
 */
export enum PidAxesMask {
    None = 0,
    LinX = 1 << 0,
    LinY = 1 << 1,
    LinZ = 1 << 2,
    // #if DIM3
    AngX = 1 << 3,
    AngY = 1 << 4,
    // #endif
    AngZ = 1 << 5,
    // #if DIM2
    AllLin = PidAxesMask.LinX | PidAxesMask.LinY,
    AllAng = PidAxesMask.AngZ,
    // #endif
    // #if DIM3
    AllLin = PidAxesMask.LinX | PidAxesMask.LinY | PidAxesMask.LinZ,
    AllAng = PidAxesMask.AngX | PidAxesMask.AngY | PidAxesMask.AngZ,
    // #endif
    All = PidAxesMask.AllLin | PidAxesMask.AllAng,
}

/**
 * A controller for controlling dynamic bodies using the
 * Proportional-Integral-Derivative correction model.
 */
export class PidController {
    private raw: RawPidController;

    private params: IntegrationParameters;
    private bodies: RigidBodySet;

    constructor(
        params: IntegrationParameters,
        bodies: RigidBodySet,
        kp: number,
        ki: number,
        kd: number,
        axes: PidAxesMask,
    ) {
        this.params = params;
        this.bodies = bodies;
        this.raw = new RawPidController(kp, ki, kd, axes);
    }

    /** @internal */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }

        this.raw = undefined;
    }

    public setKp(kp: number, axes: PidAxesMask) {
        this.raw.set_kp(kp, axes);
    }

    public setKi(ki: number, axes: PidAxesMask) {
        this.raw.set_kp(ki, axes);
    }

    public setKd(kd: number, axes: PidAxesMask) {
        this.raw.set_kp(kd, axes);
    }

    public setAxes(axes: PidAxesMask) {
        this.raw.set_axes_mask(axes);
    }

    public resetIntegrals() {
        this.raw.reset_integrals();
    }

    public applyLinearCorrection(
        body: RigidBody,
        targetPosition: Vector,
        targetLinvel: Vector,
    ) {
        let rawPos = VectorOps.intoRaw(targetPosition);
        let rawVel = VectorOps.intoRaw(targetLinvel);
        this.raw.apply_linear_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();
    }

    // #if DIM2
    public applyAngularCorrection(
        body: RigidBody,
        targetRotation: number,
        targetAngVel: number,
    ) {
        this.raw.apply_angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            targetRotation,
            targetAngVel,
        );
    }
    // #endif

    // #if DIM3
    public applyAngularCorrection(
        body: RigidBody,
        targetRotation: Rotation,
        targetAngVel: Vector,
    ) {
        let rawPos = RotationOps.intoRaw(targetRotation);
        let rawVel = VectorOps.intoRaw(targetAngVel);
        this.raw.apply_angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();
    }
    // #endif

    public linearCorrection(
        body: RigidBody,
        targetPosition: Vector,
        targetLinvel: Vector,
        target?: Vector,
    ): Vector {
        let rawPos = VectorOps.intoRaw(targetPosition);
        let rawVel = VectorOps.intoRaw(targetLinvel);
        this.raw.linear_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
            scratchBuffer,
        );
        rawPos.free();
        rawVel.free();

        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    // #if DIM2
    public angularCorrection(
        body: RigidBody,
        targetRotation: number,
        targetAngVel: number,
    ): number {
        return this.raw.angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            targetRotation,
            targetAngVel,
        );
    }
    // #endif

    // #if DIM3
    public angularCorrection(
        body: RigidBody,
        targetRotation: Rotation,
        targetAngVel: Vector,
        target?: Vector,
    ): Vector {
        let rawPos = RotationOps.intoRaw(targetRotation);
        let rawVel = VectorOps.intoRaw(targetAngVel);
        this.raw.angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
            scratchBuffer,
        );
        rawPos.free();
        rawVel.free();

        return VectorOps.fromBuffer(scratchBuffer, target);
    }
    // #endif
}
