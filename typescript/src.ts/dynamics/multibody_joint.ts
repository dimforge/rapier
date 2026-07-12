import {
    RawImpulseJointSet,
    RawJointAxis,
    RawJointType,
    RawMultibodyJointSet,
} from "../raw";
import {
    FixedImpulseJoint,
    ImpulseJointHandle,
    JointType,
    MotorModel,
    PrismaticImpulseJoint,
    RevoluteImpulseJoint,
} from "./impulse_joint";

// #if DIM3
import {Quaternion} from "../math";
import {SphericalImpulseJoint} from "./impulse_joint";
// #endif

/**
 * The integer identifier of a collider added to a `ColliderSet`.
 */
export type MultibodyJointHandle = number;

export class MultibodyJoint {
    protected rawSet: RawMultibodyJointSet; // The MultibodyJoint won't need to free this.
    handle: MultibodyJointHandle;

    constructor(rawSet: RawMultibodyJointSet, handle: MultibodyJointHandle) {
        this.rawSet = rawSet;
        this.handle = handle;
    }

    public static newTyped(
        rawSet: RawMultibodyJointSet,
        handle: MultibodyJointHandle,
    ): MultibodyJoint {
        switch (rawSet.jointType(handle)) {
            case RawJointType.Revolute:
                return new RevoluteMultibodyJoint(rawSet, handle);
            case RawJointType.Prismatic:
                return new PrismaticMultibodyJoint(rawSet, handle);
            case RawJointType.Fixed:
                return new FixedMultibodyJoint(rawSet, handle);
            // #if DIM3
            case RawJointType.Spherical:
                return new SphericalMultibodyJoint(rawSet, handle);
            // #endif
            default:
                return new MultibodyJoint(rawSet, handle);
        }
    }

    /**
     * Checks if this joint is still valid (i.e. that it has
     * not been deleted from the joint set yet).
     */
    public isValid(): boolean {
        return this.rawSet.contains(this.handle);
    }

    // /**
    //  * The unique integer identifier of the first rigid-body this joint it attached to.
    //  */
    // public bodyHandle1(): RigidBodyHandle {
    //     return this.rawSet.jointBodyHandle1(this.handle);
    // }
    //
    // /**
    //  * The unique integer identifier of the second rigid-body this joint is attached to.
    //  */
    // public bodyHandle2(): RigidBodyHandle {
    //     return this.rawSet.jointBodyHandle2(this.handle);
    // }
    //
    // /**
    //  * The type of this joint given as a string.
    //  */
    // public type(): JointType {
    //     return this.rawSet.jointType(this.handle);
    // }
    //
    // // #if DIM3
    // /**
    //  * The rotation quaternion that aligns this joint's first local axis to the `x` axis.
    //  */
    // public frameX1(): Rotation {
    //     return RotationOps.fromRaw(this.rawSet.jointFrameX1(this.handle));
    // }
    //
    // // #endif
    //
    // // #if DIM3
    // /**
    //  * The rotation matrix that aligns this joint's second local axis to the `x` axis.
    //  */
    // public frameX2(): Rotation {
    //     return RotationOps.fromRaw(this.rawSet.jointFrameX2(this.handle));
    // }
    //
    // // #endif
    //
    // /**
    //  * The position of the first anchor of this joint.
    //  *
    //  * The first anchor gives the position of the points application point on the
    //  * local frame of the first rigid-body it is attached to.
    //  */
    // public anchor1(): Vector {
    //     return VectorOps.fromRaw(this.rawSet.jointAnchor1(this.handle));
    // }
    //
    // /**
    //  * The position of the second anchor of this joint.
    //  *
    //  * The second anchor gives the position of the points application point on the
    //  * local frame of the second rigid-body it is attached to.
    //  */
    // public anchor2(): Vector {
    //     return VectorOps.fromRaw(this.rawSet.jointAnchor2(this.handle));
    // }

    /**
     * Controls whether contacts are computed between colliders attached
     * to the rigid-bodies linked by this joint.
     */
    public setContactsEnabled(enabled: boolean) {
        this.rawSet.jointSetContactsEnabled(this.handle, enabled);
    }

    /**
     * Indicates if contacts are enabled between colliders attached
     * to the rigid-bodies linked by this joint.
     */
    public contactsEnabled(): boolean {
        return this.rawSet.jointContactsEnabled(this.handle);
    }
}

export class UnitMultibodyJoint extends MultibodyJoint {
    /**
     * The axis left free by this joint.
     */
    protected rawAxis?(): RawJointAxis;

    // /**
    //  * Are the limits enabled for this joint?
    //  */
    // public limitsEnabled(): boolean {
    //     return this.rawSet.jointLimitsEnabled(this.handle, this.rawAxis());
    // }
    //
    // /**
    //  * The min limit of this joint.
    //  */
    // public limitsMin(): number {
    //     return this.rawSet.jointLimitsMin(this.handle, this.rawAxis());
    // }
    //
    // /**
    //  * The max limit of this joint.
    //  */
    // public limitsMax(): number {
    //     return this.rawSet.jointLimitsMax(this.handle, this.rawAxis());
    // }
    //
    // public configureMotorModel(model: MotorModel) {
    //     this.rawSet.jointConfigureMotorModel(this.handle, this.rawAxis(), model);
    // }
    //
    // public configureMotorVelocity(targetVel: number, factor: number) {
    //     this.rawSet.jointConfigureMotorVelocity(this.handle, this.rawAxis(), targetVel, factor);
    // }
    //
    // public configureMotorPosition(targetPos: number, stiffness: number, damping: number) {
    //     this.rawSet.jointConfigureMotorPosition(this.handle, this.rawAxis(), targetPos, stiffness, damping);
    // }
    //
    // public configureMotor(targetPos: number, targetVel: number, stiffness: number, damping: number) {
    //     this.rawSet.jointConfigureMotor(this.handle, this.rawAxis(), targetPos, targetVel, stiffness, damping);
    // }
}

export class FixedMultibodyJoint extends MultibodyJoint {}

export class PrismaticMultibodyJoint extends UnitMultibodyJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.LinX;
    }
}

export class RevoluteMultibodyJoint extends UnitMultibodyJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.AngX;
    }
}

// #if DIM3
export class SphericalMultibodyJoint extends MultibodyJoint {
    /* Unsupported by this alpha release.
    public configureMotorModel(model: MotorModel) {
        this.rawSet.jointConfigureMotorModel(this.handle, model);
    }

    public configureMotorVelocity(targetVel: Vector, factor: number) {
        this.rawSet.jointConfigureBallMotorVelocity(this.handle, targetVel.x, targetVel.y, targetVel.z, factor);
    }

    public configureMotorPosition(targetPos: Quaternion, stiffness: number, damping: number) {
        this.rawSet.jointConfigureBallMotorPosition(this.handle, targetPos.w, targetPos.x, targetPos.y, targetPos.z, stiffness, damping);
    }

    public configureMotor(targetPos: Quaternion, targetVel: Vector, stiffness: number, damping: number) {
        this.rawSet.jointConfigureBallMotor(this.handle,
            targetPos.w, targetPos.x, targetPos.y, targetPos.z,
            targetVel.x, targetVel.y, targetVel.z,
            stiffness, damping);
    }
     */
}
// #endif
