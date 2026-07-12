import {RawRigidBodySet, RawRigidBodyType} from "../raw";
import {Coarena} from "../coarena";
import {VectorOps, RotationOps} from "../math";
import {
    RigidBody,
    RigidBodyDesc,
    RigidBodyHandle,
    RigidBodyType,
} from "./rigid_body";
import {ColliderSet} from "../geometry";
import {ImpulseJointSet} from "./impulse_joint_set";
import {MultibodyJointSet} from "./multibody_joint_set";
import {IslandManager} from "./island_manager";

/**
 * A set of rigid bodies that can be handled by a physics pipeline.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `rigidBodySet.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class RigidBodySet {
    raw: RawRigidBodySet;
    private map: Coarena<RigidBody>;

    /**
     * Release the WASM memory occupied by this rigid-body set.
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

    constructor(raw?: RawRigidBodySet) {
        this.raw = raw || new RawRigidBodySet();
        this.map = new Coarena<RigidBody>();
        // deserialize
        if (raw) {
            raw.forEachRigidBodyHandle((handle: RigidBodyHandle) => {
                this.map.set(handle, new RigidBody(raw, null, handle));
            });
        }
    }

    /**
     * Internal method, do not call this explicitly.
     */
    public finalizeDeserialization(colliderSet: ColliderSet) {
        this.map.forEach((rb) => rb.finalizeDeserialization(colliderSet));
    }

    /**
     * Creates a new rigid-body and return its integer handle.
     *
     * @param desc - The description of the rigid-body to create.
     */
    public createRigidBody(
        colliderSet: ColliderSet,
        desc: RigidBodyDesc,
    ): RigidBody {
        let rawTra = VectorOps.intoRaw(desc.translation);
        let rawRot = RotationOps.intoRaw(desc.rotation);
        let rawLv = VectorOps.intoRaw(desc.linvel);
        let rawCom = VectorOps.intoRaw(desc.centerOfMass);

        // #if DIM3
        let rawAv = VectorOps.intoRaw(desc.angvel);
        let rawPrincipalInertia = VectorOps.intoRaw(
            desc.principalAngularInertia,
        );
        let rawInertiaFrame = RotationOps.intoRaw(
            desc.angularInertiaLocalFrame,
        );
        // #endif

        let handle = this.raw.createRigidBody(
            desc.enabled,
            rawTra,
            rawRot,
            desc.gravityScale,
            desc.mass,
            desc.massOnly,
            rawCom,
            rawLv,
            // #if DIM2
            desc.angvel,
            desc.principalAngularInertia,
            desc.translationsEnabledX,
            desc.translationsEnabledY,
            desc.rotationsEnabled,
            // #endif
            // #if DIM3
            rawAv,
            rawPrincipalInertia,
            rawInertiaFrame,
            desc.translationsEnabledX,
            desc.translationsEnabledY,
            desc.translationsEnabledZ,
            desc.rotationsEnabledX,
            desc.rotationsEnabledY,
            desc.rotationsEnabledZ,
            // #endif
            desc.linearDamping,
            desc.angularDamping,
            desc.status as number as RawRigidBodyType,
            desc.canSleep,
            desc.sleeping,
            desc.softCcdPrediction,
            desc.ccdEnabled,
            desc.dominanceGroup,
            desc.additionalSolverIterations,
        );

        rawTra.free();
        rawRot.free();
        rawLv.free();
        rawCom.free();

        // #if DIM3
        rawAv.free();
        rawPrincipalInertia.free();
        rawInertiaFrame.free();
        // #endif

        const body = new RigidBody(this.raw, colliderSet, handle);
        body.userData = desc.userData;

        this.map.set(handle, body);

        return body;
    }

    /**
     * Removes a rigid-body from this set.
     *
     * This will also remove all the colliders and joints attached to the rigid-body.
     *
     * @param handle - The integer handle of the rigid-body to remove.
     * @param colliders - The set of colliders that may contain colliders attached to the removed rigid-body.
     * @param impulseJoints - The set of impulse joints that may contain joints attached to the removed rigid-body.
     * @param multibodyJoints - The set of multibody joints that may contain joints attached to the removed rigid-body.
     */
    public remove(
        handle: RigidBodyHandle,
        islands: IslandManager,
        colliders: ColliderSet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        // Unmap the entities that will be removed automatically because of the rigid-body removals.
        for (let i = 0; i < this.raw.rbNumColliders(handle); i += 1) {
            colliders.unmap(this.raw.rbCollider(handle, i));
        }

        impulseJoints.forEachJointHandleAttachedToRigidBody(handle, (handle) =>
            impulseJoints.unmap(handle),
        );
        multibodyJoints.forEachJointHandleAttachedToRigidBody(
            handle,
            (handle) => multibodyJoints.unmap(handle),
        );

        // Remove the rigid-body.
        this.raw.remove(
            handle,
            islands.raw,
            colliders.raw,
            impulseJoints.raw,
            multibodyJoints.raw,
        );
        this.map.delete(handle);
    }

    /**
     * The number of rigid-bodies on this set.
     */
    public len(): number {
        return this.map.len();
    }

    /**
     * Does this set contain a rigid-body with the given handle?
     *
     * @param handle - The rigid-body handle to check.
     */
    public contains(handle: RigidBodyHandle): boolean {
        return this.get(handle) != null;
    }

    /**
     * Gets the rigid-body with the given handle.
     *
     * @param handle - The handle of the rigid-body to retrieve.
     */
    public get(handle: RigidBodyHandle): RigidBody | null {
        return this.map.get(handle);
    }

    /**
     * Applies the given closure to each rigid-body contained by this set.
     *
     * @param f - The closure to apply.
     */
    public forEach(f: (body: RigidBody) => void) {
        this.map.forEach(f);
    }

    /**
     * Applies the given closure to each active rigid-bodies contained by this set.
     *
     * A rigid-body is active if it is not sleeping, i.e., if it moved recently.
     *
     * @param f - The closure to apply.
     */
    public forEachActiveRigidBody(
        islands: IslandManager,
        f: (body: RigidBody) => void,
    ) {
        islands.forEachActiveRigidBodyHandle((handle) => {
            f(this.get(handle));
        });
    }

    /**
     * Gets all rigid-bodies in the list.
     *
     * @returns rigid-bodies list.
     */
    public getAll(): RigidBody[] {
        return this.map.getAll();
    }
}
