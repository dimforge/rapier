import {RawColliderSet} from "../raw";
import {Coarena} from "../coarena";
import {RotationOps, VectorOps} from "../math";
import {Collider, ColliderDesc, ColliderHandle} from "./collider";
import {
    ImpulseJointHandle,
    IslandManager,
    RigidBodyHandle,
    RigidBodySet,
    SoftBodySet,
    SoftMeshBinding,
} from "../dynamics";

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
     * Creates a collider holding a soft body's deformable collision mesh: a polyline (2D) or a
     * triangle mesh (3D) built with the `DEFORMABLE` flag, whose vertices follow the cluster
     * of the parent proxy through `binding`.
     *
     * Returns `null` when the binding fails: the parent is not a live cluster proxy, the shape
     * is not a deformable mesh, or a vertex could not be bound.
     *
     * @param bodies - The set of bodies where the parent proxy can be found.
     * @param softBodies - The set of soft bodies owning the cluster.
     * @param desc - The collider's description.
     * @param binding - How the mesh follows the cluster.
     * @param parentHandle - The handle of the cluster proxy (see `SoftBody.rootBody`,
     *                       `SoftBody.clusterProxy`).
     */
    public createDeformableCollider(
        bodies: RigidBodySet,
        softBodies: SoftBodySet,
        desc: ColliderDesc,
        binding: SoftMeshBinding,
        parentHandle: RigidBodyHandle,
    ): Collider | null {
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

        let handle = this.raw.createDeformableCollider(
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
            binding.rawMode(),
            binding.particles,
            binding.eps,
            binding.selfContacts,
            parentHandle,
            bodies.raw,
            softBodies.raw,
        );

        rawShape.free();
        rawTra.free();
        rawRot.free();
        rawCom.free();

        // #if DIM3
        rawPrincipalInertia.free();
        rawInertiaFrame.free();
        // #endif

        if (handle === undefined) {
            return null;
        }

        let parent = bodies.get(parentHandle);
        let collider = new Collider(this, handle, parent, desc.shape);
        this.map.set(handle, collider);
        return collider;
    }

    /**
     * Wraps the colliders the engine created on its own (the deformable surfaces and particle
     * colliders of soft bodies) that have no JavaScript wrapper yet.
     */
    public mapNewColliders(bodies: RigidBodySet) {
        this.raw.forEachColliderHandle((handle: ColliderHandle) => {
            if (!this.map.get(handle)) {
                let parentHandle = this.raw.coParent(handle);
                let parent =
                    parentHandle === undefined
                        ? null
                        : bodies.get(parentHandle);
                this.map.set(handle, new Collider(this, handle, parent));
            }
        });
    }

    /**
     * Drops the wrappers of the colliders the engine removed on its own (the colliders of
     * removed soft bodies and clusters).
     */
    public unmapRemovedColliders() {
        for (let collider of this.map.getAll()) {
            if (!this.raw.contains(collider.handle)) {
                this.map.delete(collider.handle);
            }
        }
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
        softBodies: SoftBodySet,
        wakeUp: boolean,
    ) {
        this.raw.remove(
            handle,
            islands.raw,
            bodies.raw,
            softBodies.raw,
            wakeUp,
        );
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
