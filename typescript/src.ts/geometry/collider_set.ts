import {RawColliderSet} from "../raw";
import {Coarena} from "../coarena";
import {RotationOps, VectorOps} from "../math";
import {Collider, ColliderDesc, ColliderHandle} from "./collider";
import {ImpulseJointHandle, IslandManager, RigidBodyHandle} from "../dynamics";
import {RigidBodySet} from "../dynamics";

/**
 * A set of rigid bodies that can be handled by a physics pipeline.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `colliderSet.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class ColliderSet {
    raw: RawColliderSet;
    private map: Coarena<Collider>;

    /**
     * Release the WASM memory occupied by this collider set.
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

    constructor(raw?: RawColliderSet) {
        this.raw = raw || new RawColliderSet();
        this.map = new Coarena<Collider>();
        // Initialize the map with the existing elements, if any.
        if (raw) {
            raw.forEachColliderHandle((handle: ColliderHandle) => {
                this.map.set(handle, new Collider(this, handle, null));
            });
        }
    }

    /** @internal */
    public castClosure<Res>(
        f?: (collider: Collider) => Res,
    ): (handle: ColliderHandle) => Res | undefined {
        return (handle) => {
            if (!!f) {
                return f(this.get(handle));
            } else {
                return undefined;
            }
        };
    }

    /** @internal */
    public finalizeDeserialization(bodies: RigidBodySet) {
        this.map.forEach((collider) =>
            collider.finalizeDeserialization(bodies),
        );
    }

    /**
     * Creates a new collider and return its integer handle.
     *
     * @param bodies - The set of bodies where the collider's parent can be found.
     * @param desc - The collider's description.
     * @param parentHandle - The integer handle of the rigid-body this collider is attached to.
     */
    public createCollider(
        bodies: RigidBodySet,
        desc: ColliderDesc,
        parentHandle: RigidBodyHandle,
    ): Collider {
        let hasParent = parentHandle != undefined && parentHandle != null;

        if (hasParent && isNaN(parentHandle))
            throw Error(
                "Cannot create a collider with a parent rigid-body handle that is not a number.",
            );

        let rawShape = desc.shape.intoRaw();
        let rawTra = VectorOps.intoRaw(desc.translation);
        let rawRot = RotationOps.intoRaw(desc.rotation);
        let rawCom = VectorOps.intoRaw(desc.centerOfMass);

        // #if DIM3
        let rawPrincipalInertia = VectorOps.intoRaw(
            desc.principalAngularInertia,
        );
        let rawInertiaFrame = RotationOps.intoRaw(
            desc.angularInertiaLocalFrame,
        );
        // #endif

        let handle = this.raw.createCollider(
            desc.enabled,
            rawShape,
            rawTra,
            rawRot,
            desc.massPropsMode,
            desc.mass,
            rawCom,
            // #if DIM2
            desc.principalAngularInertia,
            // #endif
            // #if DIM3
            rawPrincipalInertia,
            rawInertiaFrame,
            // #endif
            desc.density,
            desc.friction,
            desc.restitution,
            desc.frictionCombineRule,
            desc.restitutionCombineRule,
            desc.isSensor,
            desc.collisionGroups,
            desc.solverGroups,
            desc.activeCollisionTypes,
            desc.activeHooks,
            desc.activeEvents,
            desc.contactForceEventThreshold,
            desc.contactSkin,
            hasParent,
            hasParent ? parentHandle : 0,
            bodies.raw,
        );

        rawShape.free();
        rawTra.free();
        rawRot.free();
        rawCom.free();

        // #if DIM3
        rawPrincipalInertia.free();
        rawInertiaFrame.free();
        // #endif

        let parent = hasParent ? bodies.get(parentHandle) : null;
        let collider = new Collider(this, handle, parent, desc.shape);
        this.map.set(handle, collider);
        return collider;
    }

    /**
     * Remove a collider from this set.
     *
     * @param handle - The integer handle of the collider to remove.
     * @param bodies - The set of rigid-body containing the rigid-body the collider is attached to.
     * @param wakeUp - If `true`, the rigid-body the removed collider is attached to will be woken-up automatically.
     */
    public remove(
        handle: ColliderHandle,
        islands: IslandManager,
        bodies: RigidBodySet,
        wakeUp: boolean,
    ) {
        this.raw.remove(handle, islands.raw, bodies.raw, wakeUp);
        this.unmap(handle);
    }

    /**
     * Internal function, do not call directly.
     * @param handle
     */
    public unmap(handle: ImpulseJointHandle) {
        this.map.delete(handle);
    }

    /**
     * Gets the rigid-body with the given handle.
     *
     * @param handle - The handle of the rigid-body to retrieve.
     */
    public get(handle: ColliderHandle): Collider | null {
        return this.map.get(handle);
    }

    /**
     * The number of colliders on this set.
     */
    public len(): number {
        return this.map.len();
    }

    /**
     * Does this set contain a collider with the given handle?
     *
     * @param handle - The collider handle to check.
     */
    public contains(handle: ColliderHandle): boolean {
        return this.get(handle) != null;
    }

    /**
     * Applies the given closure to each collider contained by this set.
     *
     * @param f - The closure to apply.
     */
    public forEach(f: (collider: Collider) => void) {
        this.map.forEach(f);
    }

    /**
     * Gets all colliders in the list.
     *
     * @returns collider list.
     */
    public getAll(): Collider[] {
        return this.map.getAll();
    }
}
