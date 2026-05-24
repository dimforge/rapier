import {Rotation, Vector, VectorOps, RotationOps} from "../math";
import {
    RawGenericJoint,
    RawImpulseJointSet,
    RawRigidBodySet,
    RawJointAxis,
    RawJointType,
    RawMotorModel,
} from "../raw";
import {RigidBody, RigidBodyHandle} from "./rigid_body";
import {RigidBodySet} from "./rigid_body_set";
// #if DIM3
import {Quaternion} from "../math";
// #endif

/**
 * The integer identifier of a collider added to a `ColliderSet`.
 */
export type ImpulseJointHandle = number;

/**
 * An enum grouping all possible types of joints:
 *
 * - `Revolute`: A revolute joint that removes all degrees of freedom between the affected
 *               bodies except for the rotation along one axis.
 * - `Fixed`: A fixed joint that removes all relative degrees of freedom between the affected bodies.
 * - `Prismatic`: A prismatic joint that removes all degrees of freedom between the affected
 *                bodies except for the translation along one axis.
 * - `Spherical`: (3D only) A spherical joint that removes all relative linear degrees of freedom between the affected bodies.
 * - `Generic`: (3D only) A joint with customizable degrees of freedom, allowing any of the 6 axes to be locked.
 */
export enum JointType {
    Revolute,
    Fixed,
    Prismatic,
    Rope,
    Spring,
    // #if DIM3
    Spherical,
    Generic,
    // #endif
}

export enum MotorModel {
    AccelerationBased,
    ForceBased,
}

/**
 * An enum representing the possible joint axes of a generic joint.
 * They can be ORed together, like:
 * JointAxesMask.LinX || JointAxesMask.LinY
 * to get a joint that is only free in the X and Y translational (positional) axes.
 *
 * Possible free axes are:
 *
 * - `X`: X translation axis
 * - `Y`: Y translation axis
 * - `Z`: Z translation axis
 * - `AngX`: X angular rotation axis
 * - `AngY`: Y angular rotations axis
 * - `AngZ`: Z angular rotation axis
 */
export enum JointAxesMask {
    LinX = 1 << 0,
    LinY = 1 << 1,
    LinZ = 1 << 2,
    AngX = 1 << 3,
    AngY = 1 << 4,
    AngZ = 1 << 5,
}

export class ImpulseJoint {
    protected rawSet: RawImpulseJointSet; // The ImpulseJoint won't need to free this.
    protected bodySet: RigidBodySet; // The ImpulseJoint won’t need to free this.
    handle: ImpulseJointHandle;

    constructor(
        rawSet: RawImpulseJointSet,
        bodySet: RigidBodySet,
        handle: ImpulseJointHandle,
    ) {
        this.rawSet = rawSet;
        this.bodySet = bodySet;
        this.handle = handle;
    }

    public static newTyped(
        rawSet: RawImpulseJointSet,
        bodySet: RigidBodySet,
        handle: ImpulseJointHandle,
    ): ImpulseJoint {
        switch (rawSet.jointType(handle)) {
            case RawJointType.Revolute:
                return new RevoluteImpulseJoint(rawSet, bodySet, handle);
            case RawJointType.Prismatic:
                return new PrismaticImpulseJoint(rawSet, bodySet, handle);
            case RawJointType.Fixed:
                return new FixedImpulseJoint(rawSet, bodySet, handle);
            case RawJointType.Spring:
                return new SpringImpulseJoint(rawSet, bodySet, handle);
            case RawJointType.Rope:
                return new RopeImpulseJoint(rawSet, bodySet, handle);
            // #if DIM3
            case RawJointType.Spherical:
                return new SphericalImpulseJoint(rawSet, bodySet, handle);
            case RawJointType.Generic:
                return new GenericImpulseJoint(rawSet, bodySet, handle);
            // #endif
            default:
                return new ImpulseJoint(rawSet, bodySet, handle);
        }
    }

    /** @internal */
    public finalizeDeserialization(bodySet: RigidBodySet) {
        this.bodySet = bodySet;
    }

    /**
     * Checks if this joint is still valid (i.e. that it has
     * not been deleted from the joint set yet).
     */
    public isValid(): boolean {
        return this.rawSet.contains(this.handle);
    }

    /**
     * The first rigid-body this joint it attached to.
     */
    public body1(): RigidBody {
        return this.bodySet.get(this.rawSet.jointBodyHandle1(this.handle));
    }

    /**
     * The second rigid-body this joint is attached to.
     */
    public body2(): RigidBody {
        return this.bodySet.get(this.rawSet.jointBodyHandle2(this.handle));
    }

    /**
     * The type of this joint given as a string.
     */
    public type(): JointType {
        return this.rawSet.jointType(this.handle) as number as JointType;
    }

    // #if DIM3
    /**
     * The rotation quaternion that aligns this joint's first local axis to the `x` axis.
     */
    public frameX1(): Rotation {
        return RotationOps.fromRaw(this.rawSet.jointFrameX1(this.handle));
    }

    // #endif

    // #if DIM3
    /**
     * The rotation matrix that aligns this joint's second local axis to the `x` axis.
     */
    public frameX2(): Rotation {
        return RotationOps.fromRaw(this.rawSet.jointFrameX2(this.handle));
    }

    // #endif

    /**
     * The position of the first anchor of this joint.
     *
     * The first anchor gives the position of the application point on the
     * local frame of the first rigid-body it is attached to.
     */
    public anchor1(): Vector {
        return VectorOps.fromRaw(this.rawSet.jointAnchor1(this.handle));
    }

    /**
     * The position of the second anchor of this joint.
     *
     * The second anchor gives the position of the application point on the
     * local frame of the second rigid-body it is attached to.
     */
    public anchor2(): Vector {
        return VectorOps.fromRaw(this.rawSet.jointAnchor2(this.handle));
    }

    /**
     * Sets the position of the first anchor of this joint.
     *
     * The first anchor gives the position of the application point on the
     * local frame of the first rigid-body it is attached to.
     */
    public setAnchor1(newPos: Vector) {
        const rawPoint = VectorOps.intoRaw(newPos);
        this.rawSet.jointSetAnchor1(this.handle, rawPoint);
        rawPoint.free();
    }

    /**
     * Sets the position of the second anchor of this joint.
     *
     * The second anchor gives the position of the application point on the
     * local frame of the second rigid-body it is attached to.
     */
    public setAnchor2(newPos: Vector) {
        const rawPoint = VectorOps.intoRaw(newPos);
        this.rawSet.jointSetAnchor2(this.handle, rawPoint);
        rawPoint.free();
    }

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

export class UnitImpulseJoint extends ImpulseJoint {
    /**
     * The axis left free by this joint.
     */
    protected rawAxis?(): RawJointAxis;

    /**
     * Are the limits enabled for this joint?
     */
    public limitsEnabled(): boolean {
        return this.rawSet.jointLimitsEnabled(this.handle, this.rawAxis());
    }

    /**
     * The min limit of this joint.
     */
    public limitsMin(): number {
        return this.rawSet.jointLimitsMin(this.handle, this.rawAxis());
    }

    /**
     * The max limit of this joint.
     */
    public limitsMax(): number {
        return this.rawSet.jointLimitsMax(this.handle, this.rawAxis());
    }

    /**
     * Sets the limits of this joint.
     *
     * @param min - The minimum bound of this joint’s free coordinate.
     * @param max - The maximum bound of this joint’s free coordinate.
     */
    public setLimits(min: number, max: number) {
        this.rawSet.jointSetLimits(this.handle, this.rawAxis(), min, max);
    }

    public configureMotorModel(model: MotorModel) {
        this.rawSet.jointConfigureMotorModel(
            this.handle,
            this.rawAxis(),
            model as number as RawMotorModel,
        );
    }

    public setMotorMaxForce(maxForce: number) {
        this.rawSet.jointSetMotorMaxForce(
            this.handle,
            this.rawAxis(),
            maxForce,
        );
    }

    public configureMotorVelocity(targetVel: number, factor: number) {
        this.rawSet.jointConfigureMotorVelocity(
            this.handle,
            this.rawAxis(),
            targetVel,
            factor,
        );
    }

    public configureMotorPosition(
        targetPos: number,
        stiffness: number,
        damping: number,
    ) {
        this.rawSet.jointConfigureMotorPosition(
            this.handle,
            this.rawAxis(),
            targetPos,
            stiffness,
            damping,
        );
    }

    public configureMotor(
        targetPos: number,
        targetVel: number,
        stiffness: number,
        damping: number,
    ) {
        this.rawSet.jointConfigureMotor(
            this.handle,
            this.rawAxis(),
            targetPos,
            targetVel,
            stiffness,
            damping,
        );
    }
}

export class FixedImpulseJoint extends ImpulseJoint {}

export class RopeImpulseJoint extends ImpulseJoint {}

export class SpringImpulseJoint extends ImpulseJoint {}

export class PrismaticImpulseJoint extends UnitImpulseJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.LinX;
    }
}

export class RevoluteImpulseJoint extends UnitImpulseJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.AngX;
    }
}

// #if DIM3
export class GenericImpulseJoint extends ImpulseJoint {}

export class SphericalImpulseJoint extends ImpulseJoint {
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

export class JointData {
    anchor1: Vector;
    anchor2: Vector;
    axis: Vector;
    // #if DIM3
    axis1: Vector;
    axis2: Vector;
    // #endif
    frame1: Rotation;
    frame2: Rotation;
    jointType: JointType;
    limitsEnabled: boolean;
    limits: Array<number>;
    axesMask: JointAxesMask;
    stiffness: number;
    damping: number;
    length: number;

    private constructor() {}

    /**
     * Creates a new joint descriptor that builds a Fixed joint.
     *
     * A fixed joint removes all the degrees of freedom between the affected bodies, ensuring their
     * anchor and local frames coincide in world-space.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param frame1 - The reference orientation of the joint wrt. the first rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param frame2 - The reference orientation of the joint wrt. the second rigid-body.
     */
    public static fixed(
        anchor1: Vector,
        frame1: Rotation,
        anchor2: Vector,
        frame2: Rotation,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.frame1 = frame1;
        res.frame2 = frame2;
        res.jointType = JointType.Fixed;
        return res;
    }

    public static spring(
        rest_length: number,
        stiffness: number,
        damping: number,
        anchor1: Vector,
        anchor2: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.length = rest_length;
        res.stiffness = stiffness;
        res.damping = damping;
        res.jointType = JointType.Spring;
        return res;
    }

    public static rope(
        length: number,
        anchor1: Vector,
        anchor2: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.length = length;
        res.jointType = JointType.Rope;
        return res;
    }

    // #if DIM2

    /**
     * Create a new joint descriptor that builds revolute joints.
     *
     * A revolute joint allows three relative rotational degrees of freedom
     * by preventing any relative translation between the anchors of the
     * two attached rigid-bodies.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     */
    public static revolute(anchor1: Vector, anchor2: Vector): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.jointType = JointType.Revolute;
        return res;
    }

    /**
     * Creates a new joint descriptor that builds a Prismatic joint.
     *
     * A prismatic joint removes all the degrees of freedom between the
     * affected bodies, except for the translation along one axis.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param axis - Axis of the joint, expressed in the local-space of the rigid-bodies it is attached to.
     */
    public static prismatic(
        anchor1: Vector,
        anchor2: Vector,
        axis: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.axis = axis;
        res.jointType = JointType.Prismatic;
        return res;
    }

    // #endif

    // #if DIM3
    /**
     * Create a new joint descriptor that builds generic joints.
     *
     * A generic joint allows customizing its degrees of freedom
     * by supplying a mask of the joint axes that should remain locked.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param axis - The X axis of the joint, expressed in the local-space of the rigid-bodies it is attached to.
     * @param axesMask - Mask representing the locked axes of the joint. You can use logical OR to select these from
     *                   the JointAxesMask enum. For example, passing (JointAxesMask.AngX || JointAxesMask.AngY) will
     *                   create a joint locked in the X and Y rotational axes.
     */
    public static generic(
        anchor1: Vector,
        anchor2: Vector,
        axis: Vector,
        axesMask: JointAxesMask,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.axis = axis;
        res.axesMask = axesMask;
        res.jointType = JointType.Generic;
        return res;
    }

    /**
     * Create a new joint descriptor that builds spherical joints.
     *
     * A spherical joint allows three relative rotational degrees of freedom
     * by preventing any relative translation between the anchors of the
     * two attached rigid-bodies.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     */
    public static spherical(anchor1: Vector, anchor2: Vector): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.jointType = JointType.Spherical;
        return res;
    }

    /**
     * Creates a new joint descriptor that builds a Prismatic joint.
     *
     * A prismatic joint removes all the degrees of freedom between the
     * affected bodies, except for the translation along one axis.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param axis - Axis of the joint, expressed in the local-space of the rigid-bodies it is attached to.
     */
    public static prismatic(
        anchor1: Vector,
        anchor2: Vector,
        axis: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.axis = axis;
        res.jointType = JointType.Prismatic;
        return res;
    }

    /**
     * Create a new joint descriptor that builds Revolute joints.
     *
     * A revolute joint removes all degrees of freedom between the affected
     * bodies except for the rotation along one axis.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param axis - Axis of the joint, expressed in the local-space of the rigid-bodies it is attached to.
     */
    public static revolute(
        anchor1: Vector,
        anchor2: Vector,
        axis: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.axis = axis;
        res.jointType = JointType.Revolute;
        return res;
    }

    /**
     * Create a new joint descriptor that builds Revolute joints with independent
     * local axes for each attached rigid-body.
     *
     * This is useful when the same world-space hinge axis is represented by
     * different local axes on the two bodies.
     *
     * @param anchor1 - Point where the joint is attached on the first rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param anchor2 - Point where the joint is attached on the second rigid-body affected by this joint. Expressed in the
     *                  local-space of the rigid-body.
     * @param axis1 - Axis of the joint, expressed in the local-space of the first rigid-body.
     * @param axis2 - Axis of the joint, expressed in the local-space of the second rigid-body.
     */
    public static revoluteWithAxes(
        anchor1: Vector,
        anchor2: Vector,
        axis1: Vector,
        axis2: Vector,
    ): JointData {
        let res = new JointData();
        res.anchor1 = anchor1;
        res.anchor2 = anchor2;
        res.axis = axis1;
        res.axis1 = axis1;
        res.axis2 = axis2;
        res.jointType = JointType.Revolute;
        return res;
    }
    // #endif

    public intoRaw(): RawGenericJoint {
        let rawA1 = VectorOps.intoRaw(this.anchor1);
        let rawA2 = VectorOps.intoRaw(this.anchor2);
        let rawAx;
        let result;
        let limitsEnabled = false;
        let limitsMin = 0.0;
        let limitsMax = 0.0;

        switch (this.jointType) {
            case JointType.Fixed:
                let rawFra1 = RotationOps.intoRaw(this.frame1);
                let rawFra2 = RotationOps.intoRaw(this.frame2);
                result = RawGenericJoint.fixed(rawA1, rawFra1, rawA2, rawFra2);
                rawFra1.free();
                rawFra2.free();
                break;
            case JointType.Spring:
                result = RawGenericJoint.spring(
                    this.length,
                    this.stiffness,
                    this.damping,
                    rawA1,
                    rawA2,
                );
                break;
            case JointType.Rope:
                result = RawGenericJoint.rope(this.length, rawA1, rawA2);
                break;
            case JointType.Prismatic:
                rawAx = VectorOps.intoRaw(this.axis);

                if (!!this.limitsEnabled) {
                    limitsEnabled = true;
                    limitsMin = this.limits[0];
                    limitsMax = this.limits[1];
                }

                // #if DIM2
                result = RawGenericJoint.prismatic(
                    rawA1,
                    rawA2,
                    rawAx,
                    limitsEnabled,
                    limitsMin,
                    limitsMax,
                );
                // #endif

                // #if DIM3
                result = RawGenericJoint.prismatic(
                    rawA1,
                    rawA2,
                    rawAx,
                    limitsEnabled,
                    limitsMin,
                    limitsMax,
                );
                // #endif

                rawAx.free();
                break;
            // #if DIM2
            case JointType.Revolute:
                result = RawGenericJoint.revolute(rawA1, rawA2);
                break;
            // #endif
            // #if DIM3
            case JointType.Generic:
                rawAx = VectorOps.intoRaw(this.axis);
                // implicit type cast: axesMask is a JointAxesMask bitflag enum,
                // we're treating it as a u8 on the Rust side
                let rawAxesMask = this.axesMask;
                result = RawGenericJoint.generic(
                    rawA1,
                    rawA2,
                    rawAx,
                    rawAxesMask,
                );
                break;
            case JointType.Spherical:
                result = RawGenericJoint.spherical(rawA1, rawA2);
                break;
            case JointType.Revolute:
                if (!!this.axis1 && !!this.axis2) {
                    let rawAx1 = VectorOps.intoRaw(this.axis1);
                    let rawAx2 = VectorOps.intoRaw(this.axis2);
                    result = RawGenericJoint.revoluteWithAxes(
                        rawA1,
                        rawA2,
                        rawAx1,
                        rawAx2,
                    );
                    rawAx1.free();
                    rawAx2.free();
                } else {
                    rawAx = VectorOps.intoRaw(this.axis);
                    result = RawGenericJoint.revolute(rawA1, rawA2, rawAx);
                    rawAx.free();
                }
                break;
            // #endif
        }

        rawA1.free();
        rawA2.free();

        return result;
    }
}
