import {RawImpulseJointSet} from "../raw";
import {Coarena} from "../coarena";
import {RigidBodySet} from "./rigid_body_set";
import {
    RevoluteImpulseJoint,
    FixedImpulseJoint,
    ImpulseJoint,
    ImpulseJointHandle,
    JointData,
    JointType,
    PrismaticImpulseJoint,
    // #if DIM3
    SphericalImpulseJoint,
    // #endif
} from "./impulse_joint";
import {IslandManager} from "./island_manager";
import {RigidBodyHandle} from "./rigid_body";
import {Collider, ColliderHandle} from "../geometry";

/**
 * A set of joints.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `jointSet.free()`
 * once you are done using it (and all the joints it created).
 */
export class ImpulseJointSet {
    raw: RawImpulseJointSet;
    private map: Coarena<ImpulseJoint>;

    /**
     * Release the WASM memory occupied by this joint set.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;

        if (!!this.map) {
            this.map.clear();
        }
        this.map = undefined;
    }

    constructor(raw?: RawImpulseJointSet) {
        this.raw = raw || new RawImpulseJointSet();
        this.map = new Coarena<ImpulseJoint>();
        // Initialize the map with the existing elements, if any.
        if (raw) {
            raw.forEachJointHandle((handle: ImpulseJointHandle) => {
                this.map.set(handle, ImpulseJoint.newTyped(raw, null, handle));
            });
        }
    }

    /** @internal */
    public finalizeDeserialization(bodies: RigidBodySet) {
        this.map.forEach((joint) => joint.finalizeDeserialization(bodies));
    }

    /**
     * Creates a new joint and return its integer handle.
     *
     * @param bodies - The set of rigid-bodies containing the bodies the joint is attached to.
     * @param desc - The joint's parameters.
     * @param parent1 - The handle of the first rigid-body this joint is attached to.
     * @param parent2 - The handle of the second rigid-body this joint is attached to.
     * @param wakeUp - Should the attached rigid-bodies be awakened?
     */
    public createJoint(
        bodies: RigidBodySet,
        desc: JointData,
        parent1: RigidBodyHandle,
        parent2: RigidBodyHandle,
        wakeUp: boolean,
    ): ImpulseJoint {
        const rawParams = desc.intoRaw();
        const handle = this.raw.createJoint(
            rawParams,
            parent1,
            parent2,
            wakeUp,
        );
        rawParams.free();
        let joint = ImpulseJoint.newTyped(this.raw, bodies, handle);
        this.map.set(handle, joint);
        return joint;
    }

    /**
     * Remove a joint from this set.
     *
     * @param handle - The integer handle of the joint.
     * @param wakeUp - If `true`, the rigid-bodies attached by the removed joint will be woken-up automatically.
     */
    public remove(handle: ImpulseJointHandle, wakeUp: boolean) {
        this.raw.remove(handle, wakeUp);
        this.unmap(handle);
    }

    /**
     * Calls the given closure with the integer handle of each impulse joint attached to this rigid-body.
     *
     * @param f - The closure called with the integer handle of each impulse joint attached to the rigid-body.
     */
    public forEachJointHandleAttachedToRigidBody(
        handle: RigidBodyHandle,
        f: (handle: ImpulseJointHandle) => void,
    ) {
        this.raw.forEachJointAttachedToRigidBody(handle, f);
    }

    /**
     * Internal function, do not call directly.
     * @param handle
     */
    public unmap(handle: ImpulseJointHandle) {
        this.map.delete(handle);
    }

    /**
     * The number of joints on this set.
     */
    public len(): number {
        return this.map.len();
    }

    /**
     * Does this set contain a joint with the given handle?
     *
     * @param handle - The joint handle to check.
     */
    public contains(handle: ImpulseJointHandle): boolean {
        return this.get(handle) != null;
    }

    /**
     * Gets the joint with the given handle.
     *
     * Returns `null` if no joint with the specified handle exists.
     *
     * @param handle - The integer handle of the joint to retrieve.
     */
    public get(handle: ImpulseJointHandle): ImpulseJoint | null {
        return this.map.get(handle);
    }

    /**
     * Applies the given closure to each joint contained by this set.
     *
     * @param f - The closure to apply.
     */
    public forEach(f: (joint: ImpulseJoint) => void) {
        this.map.forEach(f);
    }

    /**
     * Gets all joints in the list.
     *
     * @returns joint list.
     */
    public getAll(): ImpulseJoint[] {
        return this.map.getAll();
    }
}
