import {Rotation, RotationOps, Vector, VectorOps, scratchBuffer} from "../math";
import {CoefficientCombineRule, RigidBody, RigidBodySet} from "../dynamics";
import {ActiveHooks, ActiveEvents} from "../pipeline";
import {InteractionGroups} from "./interaction_groups";
import {
    Shape,
    Cuboid,
    Ball,
    ShapeType,
    Capsule,
    Voxels,
    TriMesh,
    Polyline,
    Heightfield,
    Segment,
    Triangle,
    RoundTriangle,
    RoundCuboid,
    HalfSpace,
    TriMeshFlags,
    // #if DIM2
    ConvexPolygon,
    RoundConvexPolygon,
    // #endif
    // #if DIM3
    Cylinder,
    RoundCylinder,
    Cone,
    RoundCone,
    ConvexPolyhedron,
    RoundConvexPolyhedron,
    HeightFieldFlags,
    // #endif
} from "./shape";
import {Ray, RayIntersection} from "./ray";
import {PointProjection} from "./point";
import {ColliderShapeCastHit, ShapeCastHit} from "./toi";
import {ShapeContact} from "./contact";
import {ColliderSet} from "./collider_set";

/**
 * Flags affecting whether collision-detection happens between two colliders
 * depending on the type of rigid-bodies they are attached to.
 */
export enum ActiveCollisionTypes {
    /**
     * Enable collision-detection between a collider attached to a dynamic body
     * and another collider attached to a dynamic body.
     */
    DYNAMIC_DYNAMIC = 0b0000_0000_0000_0001,
    /**
     * Enable collision-detection between a collider attached to a dynamic body
     * and another collider attached to a kinematic body.
     */
    DYNAMIC_KINEMATIC = 0b0000_0000_0000_1100,
    /**
     * Enable collision-detection between a collider attached to a dynamic body
     * and another collider attached to a fixed body (or not attached to any body).
     */
    DYNAMIC_FIXED = 0b0000_0000_0000_0010,
    /**
     * Enable collision-detection between a collider attached to a kinematic body
     * and another collider attached to a kinematic body.
     */
    KINEMATIC_KINEMATIC = 0b1100_1100_0000_0000,

    /**
     * Enable collision-detection between a collider attached to a kinematic body
     * and another collider attached to a fixed body (or not attached to any body).
     */
    KINEMATIC_FIXED = 0b0010_0010_0000_0000,

    /**
     * Enable collision-detection between a collider attached to a fixed body (or
     * not attached to any body) and another collider attached to a fixed body (or
     * not attached to any body).
     */
    FIXED_FIXED = 0b0000_0000_0010_0000,
    /**
     * The default active collision types, enabling collisions between a dynamic body
     * and another body of any type, but not enabling collisions between two non-dynamic bodies.
     */
    DEFAULT = DYNAMIC_KINEMATIC | DYNAMIC_DYNAMIC | DYNAMIC_FIXED,
    /**
     * Enable collisions between any kind of rigid-bodies (including between two non-dynamic bodies).
     */
    ALL = DYNAMIC_KINEMATIC |
        DYNAMIC_DYNAMIC |
        DYNAMIC_FIXED |
        KINEMATIC_KINEMATIC |
        KINEMATIC_FIXED |
        KINEMATIC_KINEMATIC,
}

/**
 * The integer identifier of a collider added to a `ColliderSet`.
 */
export type ColliderHandle = number;

/**
 * A geometric entity that can be attached to a body so it can be affected
 * by contacts and proximity queries.
 */
export class Collider {
    private colliderSet: ColliderSet; // The Collider won't need to free this.
    readonly handle: ColliderHandle;
    private _shape: Shape; // TODO: deprecate/remove this since it isn’t a reliable way of getting the latest shape properties.
    private _parent: RigidBody | null;

    constructor(
        colliderSet: ColliderSet,
        handle: ColliderHandle,
        parent: RigidBody | null,
        shape?: Shape,
    ) {
        this.colliderSet = colliderSet;
        this.handle = handle;
        this._parent = parent;
        this._shape = shape;
    }

    /** @internal */
    public finalizeDeserialization(bodies: RigidBodySet) {
        if (this.handle != null) {
            this._parent = bodies.get(
                this.colliderSet.raw.coParent(this.handle),
            );
        }
    }

    private ensureShapeIsCached() {
        if (!this._shape)
            this._shape = Shape.fromRaw(this.colliderSet.raw, this.handle);
    }

    /**
     * The shape of this collider.
     */
    public get shape(): Shape {
        this.ensureShapeIsCached();
        return this._shape;
    }

    /**
     * Set the internal cached JS shape to null.
     *
     * This can be useful if you want to free some memory (assuming you are not
     * holding any other references to the shape object), or in order to force
     * the recalculation of the JS shape (the next time the `shape` getter is
     * accessed) from the WASM source of truth.
     */
    public clearShapeCache() {
        this._shape = null;
    }

    /**
     * Checks if this collider is still valid (i.e. that it has
     * not been deleted from the collider set yet).
     */
    public isValid(): boolean {
        return this.colliderSet.raw.contains(this.handle);
    }

    /**
     * The world-space translation of this collider.
     *
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public translation(target?: Vector): Vector {
        this.colliderSet.raw.coTranslation(this.handle, scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The translation of this collider relative to its parent rigid-body.
     *
     * Returns `null` if the collider doesn’t have a parent rigid-body.
     *
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public translationWrtParent(target?: Vector): Vector | null {
        const hasParent = this.colliderSet.raw.coTranslationWrtParent(
            this.handle,
            scratchBuffer,
        );
        return hasParent ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    // #if DIM2
    /**
     * The world-space orientation of this collider.
     */
    public rotation(): number {
        return this.colliderSet.raw.coRotation(this.handle);
    }
    // #endif

    // #if DIM3
    /**
     * The world-space orientation of this collider.
     *
     * @param {Rotation?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public rotation(target?: Rotation): Rotation {
        this.colliderSet.raw.coRotation(this.handle, scratchBuffer);
        return RotationOps.fromBuffer(scratchBuffer, target);
    }
    // #endif

    // #if DIM2
    /**
     * The orientation of this collider relative to its parent rigid-body.
     *
     * Returns `null` if the collider doesn’t have a parent rigid-body.
     */
    public rotationWrtParent(): Rotation | null {
        const val = this.colliderSet.raw.coRotationWrtParent(this.handle);
        return isNaN(val) ? null : val;
    }
    // #endif

    // #if DIM3
    /**
     * The orientation of this collider relative to its parent rigid-body.
     *
     * Returns `null` if the collider doesn’t have a parent rigid-body.
     *
     * @param {Rotation?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public rotationWrtParent(target?: Rotation): Rotation | null {
        const hasParent = this.colliderSet.raw.coRotationWrtParent(
            this.handle,
            scratchBuffer,
        );
        return hasParent ? RotationOps.fromBuffer(scratchBuffer, target) : null;
    }
    // #endif

    /**
     * Is this collider a sensor?
     */
    public isSensor(): boolean {
        return this.colliderSet.raw.coIsSensor(this.handle);
    }

    /**
     * Sets whether this collider is a sensor.
     * @param isSensor - If `true`, the collider will be a sensor.
     */
    public setSensor(isSensor: boolean) {
        this.colliderSet.raw.coSetSensor(this.handle, isSensor);
    }

    /**
     * Sets the new shape of the collider.
     * @param shape - The collider’s new shape.
     */
    public setShape(shape: Shape) {
        let rawShape = shape.intoRaw();
        this.colliderSet.raw.coSetShape(this.handle, rawShape);
        rawShape.free();
        this._shape = shape;
    }

    /**
     * Sets whether this collider is enabled or not.
     *
     * @param enabled - Set to `false` to disable this collider (its parent rigid-body won’t be disabled automatically by this).
     */
    public setEnabled(enabled: boolean) {
        this.colliderSet.raw.coSetEnabled(this.handle, enabled);
    }

    /**
     * Is this collider enabled?
     */
    public isEnabled(): boolean {
        return this.colliderSet.raw.coIsEnabled(this.handle);
    }

    /**
     * Sets the restitution coefficient of the collider to be created.
     *
     * @param restitution - The restitution coefficient in `[0, 1]`. A value of 0 (the default) means no bouncing behavior
     *                   while 1 means perfect bouncing (though energy may still be lost due to numerical errors of the
     *                   constraints solver).
     */
    public setRestitution(restitution: number) {
        this.colliderSet.raw.coSetRestitution(this.handle, restitution);
    }

    /**
     * Sets the friction coefficient of the collider to be created.
     *
     * @param friction - The friction coefficient. Must be greater or equal to 0. This is generally smaller than 1. The
     *                   higher the coefficient, the stronger friction forces will be for contacts with the collider
     *                   being built.
     */
    public setFriction(friction: number) {
        this.colliderSet.raw.coSetFriction(this.handle, friction);
    }

    /**
     * Gets the rule used to combine the friction coefficients of two colliders
     * colliders involved in a contact.
     */
    public frictionCombineRule(): CoefficientCombineRule {
        return this.colliderSet.raw.coFrictionCombineRule(this.handle);
    }

    /**
     * Sets the rule used to combine the friction coefficients of two colliders
     * colliders involved in a contact.
     *
     * @param rule − The combine rule to apply.
     */
    public setFrictionCombineRule(rule: CoefficientCombineRule) {
        this.colliderSet.raw.coSetFrictionCombineRule(this.handle, rule);
    }

    /**
     * Gets the rule used to combine the restitution coefficients of two colliders
     * colliders involved in a contact.
     */
    public restitutionCombineRule(): CoefficientCombineRule {
        return this.colliderSet.raw.coRestitutionCombineRule(this.handle);
    }

    /**
     * Sets the rule used to combine the restitution coefficients of two colliders
     * colliders involved in a contact.
     *
     * @param rule − The combine rule to apply.
     */
    public setRestitutionCombineRule(rule: CoefficientCombineRule) {
        this.colliderSet.raw.coSetRestitutionCombineRule(this.handle, rule);
    }

    /**
     * Sets the collision groups used by this collider.
     *
     * Two colliders will interact iff. their collision groups are compatible.
     * See the documentation of `InteractionGroups` for details on teh used bit pattern.
     *
     * @param groups - The collision groups used for the collider being built.
     */
    public setCollisionGroups(groups: InteractionGroups) {
        this.colliderSet.raw.coSetCollisionGroups(this.handle, groups);
    }

    /**
     * Sets the solver groups used by this collider.
     *
     * Forces between two colliders in contact will be computed iff their solver
     * groups are compatible.
     * See the documentation of `InteractionGroups` for details on the used bit pattern.
     *
     * @param groups - The solver groups used for the collider being built.
     */
    public setSolverGroups(groups: InteractionGroups) {
        this.colliderSet.raw.coSetSolverGroups(this.handle, groups);
    }

    /**
     * Sets the contact skin for this collider.
     *
     * See the documentation of `ColliderDesc.setContactSkin` for additional details.
     */
    public contactSkin(): number {
        return this.colliderSet.raw.coContactSkin(this.handle);
    }

    /**
     * Sets the contact skin for this collider.
     *
     * See the documentation of `ColliderDesc.setContactSkin` for additional details.
     *
     * @param thickness - The contact skin thickness.
     */
    public setContactSkin(thickness: number) {
        return this.colliderSet.raw.coSetContactSkin(this.handle, thickness);
    }

    /**
     * Get the physics hooks active for this collider.
     */
    public activeHooks(): ActiveHooks {
        return this.colliderSet.raw.coActiveHooks(this.handle);
    }

    /**
     * Set the physics hooks active for this collider.
     *
     * Use this to enable custom filtering rules for contact/intersecstion pairs involving this collider.
     *
     * @param activeHooks - The hooks active for contact/intersection pairs involving this collider.
     */
    public setActiveHooks(activeHooks: ActiveHooks) {
        this.colliderSet.raw.coSetActiveHooks(this.handle, activeHooks);
    }

    /**
     * The events active for this collider.
     */
    public activeEvents(): ActiveEvents {
        return this.colliderSet.raw.coActiveEvents(this.handle);
    }

    /**
     * Set the events active for this collider.
     *
     * Use this to enable contact and/or intersection event reporting for this collider.
     *
     * @param activeEvents - The events active for contact/intersection pairs involving this collider.
     */
    public setActiveEvents(activeEvents: ActiveEvents) {
        this.colliderSet.raw.coSetActiveEvents(this.handle, activeEvents);
    }

    /**
     * Gets the collision types active for this collider.
     */
    public activeCollisionTypes(): ActiveCollisionTypes {
        return this.colliderSet.raw.coActiveCollisionTypes(this.handle);
    }

    /**
     * Sets the total force magnitude beyond which a contact force event can be emitted.
     *
     * @param threshold - The new force threshold.
     */
    public setContactForceEventThreshold(threshold: number) {
        return this.colliderSet.raw.coSetContactForceEventThreshold(
            this.handle,
            threshold,
        );
    }

    /**
     * The total force magnitude beyond which a contact force event can be emitted.
     */
    public contactForceEventThreshold(): number {
        return this.colliderSet.raw.coContactForceEventThreshold(this.handle);
    }

    /**
     * Set the collision types active for this collider.
     *
     * @param activeCollisionTypes - The hooks active for contact/intersection pairs involving this collider.
     */
    public setActiveCollisionTypes(activeCollisionTypes: ActiveCollisionTypes) {
        this.colliderSet.raw.coSetActiveCollisionTypes(
            this.handle,
            activeCollisionTypes,
        );
    }

    /**
     * Sets the uniform density of this collider.
     *
     * This will override any previous mass-properties set by `this.setDensity`,
     * `this.setMass`, `this.setMassProperties`, `ColliderDesc.density`,
     * `ColliderDesc.mass`, or `ColliderDesc.massProperties` for this collider.
     *
     * The mass and angular inertia of this collider will be computed automatically based on its
     * shape.
     */
    public setDensity(density: number) {
        this.colliderSet.raw.coSetDensity(this.handle, density);
    }

    /**
     * Sets the mass of this collider.
     *
     * This will override any previous mass-properties set by `this.setDensity`,
     * `this.setMass`, `this.setMassProperties`, `ColliderDesc.density`,
     * `ColliderDesc.mass`, or `ColliderDesc.massProperties` for this collider.
     *
     * The angular inertia of this collider will be computed automatically based on its shape
     * and this mass value.
     */
    public setMass(mass: number) {
        this.colliderSet.raw.coSetMass(this.handle, mass);
    }

    // #if DIM3
    /**
     * Sets the mass of this collider.
     *
     * This will override any previous mass-properties set by `this.setDensity`,
     * `this.setMass`, `this.setMassProperties`, `ColliderDesc.density`,
     * `ColliderDesc.mass`, or `ColliderDesc.massProperties` for this collider.
     */
    public setMassProperties(
        mass: number,
        centerOfMass: Vector,
        principalAngularInertia: Vector,
        angularInertiaLocalFrame: Rotation,
    ) {
        let rawCom = VectorOps.intoRaw(centerOfMass);
        let rawPrincipalInertia = VectorOps.intoRaw(principalAngularInertia);
        let rawInertiaFrame = RotationOps.intoRaw(angularInertiaLocalFrame);

        this.colliderSet.raw.coSetMassProperties(
            this.handle,
            mass,
            rawCom,
            rawPrincipalInertia,
            rawInertiaFrame,
        );

        rawCom.free();
        rawPrincipalInertia.free();
        rawInertiaFrame.free();
    }

    // #endif

    // #if DIM2
    /**
     * Sets the mass of this collider.
     *
     * This will override any previous mass-properties set by `this.setDensity`,
     * `this.setMass`, `this.setMassProperties`, `ColliderDesc.density`,
     * `ColliderDesc.mass`, or `ColliderDesc.massProperties` for this collider.
     */
    public setMassProperties(
        mass: number,
        centerOfMass: Vector,
        principalAngularInertia: number,
    ) {
        let rawCom = VectorOps.intoRaw(centerOfMass);
        this.colliderSet.raw.coSetMassProperties(
            this.handle,
            mass,
            rawCom,
            principalAngularInertia,
        );
        rawCom.free();
    }

    // #endif

    /**
     * Sets the translation of this collider.
     *
     * @param tra - The world-space position of the collider.
     */
    public setTranslation(tra: Vector) {
        // #if DIM2
        this.colliderSet.raw.coSetTranslation(this.handle, tra.x, tra.y);
        // #endif
        // #if DIM3
        this.colliderSet.raw.coSetTranslation(this.handle, tra.x, tra.y, tra.z);
        // #endif
    }

    /**
     * Sets the translation of this collider relative to its parent rigid-body.
     *
     * Does nothing if this collider isn't attached to a rigid-body.
     *
     * @param tra - The new translation of the collider relative to its parent.
     */
    public setTranslationWrtParent(tra: Vector) {
        // #if DIM2
        this.colliderSet.raw.coSetTranslationWrtParent(
            this.handle,
            tra.x,
            tra.y,
        );
        // #endif
        // #if DIM3
        this.colliderSet.raw.coSetTranslationWrtParent(
            this.handle,
            tra.x,
            tra.y,
            tra.z,
        );
        // #endif
    }

    // #if DIM3
    /**
     * Sets the rotation quaternion of this collider.
     *
     * This does nothing if a zero quaternion is provided.
     *
     * @param rotation - The rotation to set.
     */
    public setRotation(rot: Rotation) {
        this.colliderSet.raw.coSetRotation(
            this.handle,
            rot.x,
            rot.y,
            rot.z,
            rot.w,
        );
    }

    /**
     * Sets the rotation quaternion of this collider relative to its parent rigid-body.
     *
     * This does nothing if a zero quaternion is provided or if this collider isn't
     * attached to a rigid-body.
     *
     * @param rotation - The rotation to set.
     */
    public setRotationWrtParent(rot: Rotation) {
        this.colliderSet.raw.coSetRotationWrtParent(
            this.handle,
            rot.x,
            rot.y,
            rot.z,
            rot.w,
        );
    }

    // #endif
    // #if DIM2
    /**
     * Sets the rotation angle of this collider.
     *
     * @param angle - The rotation angle, in radians.
     */
    public setRotation(angle: number) {
        this.colliderSet.raw.coSetRotation(this.handle, angle);
    }

    /**
     * Sets the rotation angle of this collider relative to its parent rigid-body.
     *
     * Does nothing if this collider isn't attached to a rigid-body.
     *
     * @param angle - The rotation angle, in radians.
     */
    public setRotationWrtParent(angle: number) {
        this.colliderSet.raw.coSetRotationWrtParent(this.handle, angle);
    }

    // #endif

    /**
     * The type of the shape of this collider.
     */
    public shapeType(): ShapeType {
        return this.colliderSet.raw.coShapeType(
            this.handle,
        ) as number as ShapeType;
    }

    /**
     * The half-extents of this collider if it is a cuboid shape.
     *
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public halfExtents(target?: Vector): Vector | null {
        const isCuboid = this.colliderSet.raw.coHalfExtents(
            this.handle,
            scratchBuffer,
        );
        return isCuboid ? VectorOps.fromBuffer(scratchBuffer, target) : null;
    }

    /**
     * Sets the half-extents of this collider if it is a cuboid shape.
     *
     * @param newHalfExtents - desired half extents.
     */
    public setHalfExtents(newHalfExtents: Vector) {
        const rawPoint = VectorOps.intoRaw(newHalfExtents);
        this.colliderSet.raw.coSetHalfExtents(this.handle, rawPoint);
    }

    /**
     * The radius of this collider if it is a ball, cylinder, capsule, or cone shape.
     */
    public radius(): number {
        return this.colliderSet.raw.coRadius(this.handle);
    }

    /**
     * Sets the radius of this collider if it is a ball, cylinder, capsule, or cone shape.
     *
     * @param newRadius - desired radius.
     */
    public setRadius(newRadius: number): void {
        this.colliderSet.raw.coSetRadius(this.handle, newRadius);
    }

    /**
     * The radius of the round edges of this collider if it is a round cylinder.
     */
    public roundRadius(): number {
        return this.colliderSet.raw.coRoundRadius(this.handle);
    }

    /**
     * Sets the radius of the round edges of this collider if it has round edges.
     *
     * @param newBorderRadius - desired round edge radius.
     */
    public setRoundRadius(newBorderRadius: number) {
        this.colliderSet.raw.coSetRoundRadius(this.handle, newBorderRadius);
    }

    /**
     * The half height of this collider if it is a cylinder, capsule, or cone shape.
     */
    public halfHeight(): number {
        return this.colliderSet.raw.coHalfHeight(this.handle);
    }

    /**
     * Sets the half height of this collider if it is a cylinder, capsule, or cone shape.
     *
     * @param newHalfheight - desired half height.
     */
    public setHalfHeight(newHalfheight: number) {
        this.colliderSet.raw.coSetHalfHeight(this.handle, newHalfheight);
    }

    /**
     * If this collider has a Voxels shape, this will mark the voxel at the
     * given grid coordinates as filled or empty (depending on the `filled`
     * argument).
     *
     * Each input value is assumed to be an integer.
     *
     * The operation is O(1), unless the provided coordinates are out of the
     * bounds of the currently allocated internal grid in which case the grid
     * will be grown automatically.
     */
    public setVoxel(
        ix: number,
        iy: number,
        // #if DIM3
        iz: number,
        // #endif
        filled: boolean,
    ) {
        this.colliderSet.raw.coSetVoxel(
            this.handle,
            ix,
            iy,
            // #if DIM3
            iz,
            // #endif
            filled,
        );
        // We modified the shape, invalidate it to keep our cache
        // up-to-date the next time the user requests the shape data.
        // PERF: this isn’t ideal for performances as this adds a
        //       hidden, non-constant, cost.
        this._shape = null;
    }

    /**
     * If this and `voxels2` are voxel colliders, and a voxel from `this` was
     * modified with `setVoxel`, this will ensure that a
     * moving object transitioning across the boundaries of these colliders
     * won’t suffer from the "internal edges" artifact.
     *
     * The indices `ix, iy, iz` indicate the integer coordinates of the voxel in
     * the local coordinate frame of `this`.
     *
     * If the voxels in `voxels2` live in a different coordinate space from `this`,
     * then the `shift_*` argument indicate the distance, in voxel units, between
     * the origin of `this` to the origin of `voxels2`.
     *
     * This method is intended to be called between `this` and all the other
     * voxels colliders with a domain intersecting `this` or sharing a domain
     * boundary. This is an incremental maintenance of the effect of
     * `combineVoxelStates`.
     */
    public propagateVoxelChange(
        voxels2: Collider,
        ix: number,
        iy: number,
        // #if DIM3
        iz: number,
        // #endif
        shift_x: number,
        shift_y: number,
        // #if DIM3
        shift_z: number,
        // #endif
    ) {
        this.colliderSet.raw.coPropagateVoxelChange(
            this.handle,
            voxels2.handle,
            ix,
            iy,
            // #if DIM3
            iz,
            // #endif
            shift_x,
            shift_y,
            // #if DIM3
            shift_z,
            // #endif
        );
        // We modified the shape, invalidate it to keep our cache
        // up-to-date the next time the user requests the shape data.
        // PERF: this isn’t ideal for performances as this adds a
        //       hidden, non-constant, cost.
        this._shape = null;
    }

    /**
     * If this and `voxels2` are voxel colliders, this will ensure that a
     * moving object transitioning across the boundaries of these colliders
     * won’t suffer from the "internal edges" artifact.
     *
     * If the voxels in `voxels2` live in a different coordinate space from `this`,
     * then the `shift_*` argument indicate the distance, in voxel units, between
     * the origin of `this` to the origin of `voxels2`.
     *
     * This method is intended to be called once between all pairs of voxels
     * colliders with intersecting domains or shared boundaries.
     *
     * If either voxels collider is then modified with `setVoxel`, the
     * `propagateVoxelChange` method must be called to maintain the coupling
     * between the voxels shapes after the modification.
     */
    public combineVoxelStates(
        voxels2: Collider,
        shift_x: number,
        shift_y: number,
        // #if DIM3
        shift_z: number,
        // #endif
    ) {
        this.colliderSet.raw.coCombineVoxelStates(
            this.handle,
            voxels2.handle,
            shift_x,
            shift_y,
            // #if DIM3
            shift_z,
            // #endif
        );
        // We modified the shape, invalidate it to keep our cache
        // up-to-date the next time the user requests the shape data.
        // PERF: this isn’t ideal for performances as this adds a
        //       hidden, non-constant, cost.
        this._shape = null;
    }

    /**
     * If this collider has a triangle mesh, polyline, convex polygon, or convex polyhedron shape,
     * this returns the vertex buffer of said shape.
     */
    public vertices(): Float32Array {
        return this.colliderSet.raw.coVertices(this.handle);
    }

    /**
     * If this collider has a triangle mesh, polyline, or convex polyhedron shape,
     * this returns the index buffer of said shape.
     */
    public indices(): Uint32Array | undefined {
        return this.colliderSet.raw.coIndices(this.handle);
    }

    /**
     * If this collider has a heightfield shape, this returns the heights buffer of
     * the heightfield.
     * In 3D, the returned height matrix is provided in column-major order.
     */
    public heightfieldHeights(): Float32Array {
        return this.colliderSet.raw.coHeightfieldHeights(this.handle);
    }

    /**
     * If this collider has a heightfield shape, this returns the scale
     * applied to it.
     *
     * @param {Vector?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public heightfieldScale(target?: Vector): Vector | null {
        const isHeightfield = this.colliderSet.raw.coHeightfieldScale(
            this.handle,
            scratchBuffer,
        );
        return isHeightfield
            ? VectorOps.fromBuffer(scratchBuffer, target)
            : null;
    }

    // #if DIM3
    /**
     * If this collider has a heightfield shape, this returns the number of
     * rows of its height matrix.
     */
    public heightfieldNRows(): number {
        return this.colliderSet.raw.coHeightfieldNRows(this.handle);
    }

    /**
     * If this collider has a heightfield shape, this returns the number of
     * columns of its height matrix.
     */
    public heightfieldNCols(): number {
        return this.colliderSet.raw.coHeightfieldNCols(this.handle);
    }

    // #endif

    /**
     * The rigid-body this collider is attached to.
     */
    public parent(): RigidBody | null {
        return this._parent;
    }

    /**
     * The friction coefficient of this collider.
     */
    public friction(): number {
        return this.colliderSet.raw.coFriction(this.handle);
    }

    /**
     * The restitution coefficient of this collider.
     */
    public restitution(): number {
        return this.colliderSet.raw.coRestitution(this.handle);
    }

    /**
     * The density of this collider.
     */
    public density(): number {
        return this.colliderSet.raw.coDensity(this.handle);
    }

    /**
     * The mass of this collider.
     */
    public mass(): number {
        return this.colliderSet.raw.coMass(this.handle);
    }

    /**
     * The volume of this collider.
     */
    public volume(): number {
        return this.colliderSet.raw.coVolume(this.handle);
    }

    /**
     * The collision groups of this collider.
     */
    public collisionGroups(): InteractionGroups {
        return this.colliderSet.raw.coCollisionGroups(this.handle);
    }

    /**
     * The solver groups of this collider.
     */
    public solverGroups(): InteractionGroups {
        return this.colliderSet.raw.coSolverGroups(this.handle);
    }

    /**
     * Tests if this collider contains a point.
     *
     * @param point - The point to test.
     */
    public containsPoint(point: Vector): boolean {
        let rawPoint = VectorOps.intoRaw(point);
        let result = this.colliderSet.raw.coContainsPoint(
            this.handle,
            rawPoint,
        );

        rawPoint.free();

        return result;
    }

    /**
     * Find the projection of a point on this collider.
     *
     * @param point - The point to project.
     * @param solid - If this is set to `true` then the collider shapes are considered to
     *   be plain (if the point is located inside of a plain shape, its projection is the point
     *   itself). If it is set to `false` the collider shapes are considered to be hollow
     *   (if the point is located inside of an hollow shape, it is projected on the shape's
     *   boundary).
     */
    public projectPoint(
        point: Vector,
        solid: boolean,
        target?: PointProjection,
    ): PointProjection | null {
        let rawPoint = VectorOps.intoRaw(point);
        let result = PointProjection.fromBuffer(
            this.colliderSet.raw.coProjectPoint(this.handle, rawPoint, solid),
            target,
        );

        rawPoint.free();

        return result;
    }

    /**
     * Tests if this collider intersects the given ray.
     *
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     */
    public intersectsRay(ray: Ray, maxToi: number): boolean {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let result = this.colliderSet.raw.coIntersectsRay(
            this.handle,
            rawOrig,
            rawDir,
            maxToi,
        );

        rawOrig.free();
        rawDir.free();

        return result;
    }

    /**
     * Computes the smallest time between this and the given shape under translational movement are separated by a distance smaller or equal to distance.
     *
     * @param collider1Vel - The constant velocity of the current shape to cast (i.e. the cast direction).
     * @param shape2 - The shape to cast against.
     * @param shape2Pos - The position of the second shape.
     * @param shape2Rot - The rotation of the second shape.
     * @param shape2Vel - The constant velocity of the second shape.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                         will be returned.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the distance traveled by the shape to `collider1Vel.norm() * maxToi`.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param {ShapeCastHit?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public castShape(
        collider1Vel: Vector,
        shape2: Shape,
        shape2Pos: Vector,
        shape2Rot: Rotation,
        shape2Vel: Vector,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        target?: ShapeCastHit,
    ): ShapeCastHit | null {
        let rawCollider1Vel = VectorOps.intoRaw(collider1Vel);
        let rawShape2Pos = VectorOps.intoRaw(shape2Pos);
        let rawShape2Rot = RotationOps.intoRaw(shape2Rot);
        let rawShape2Vel = VectorOps.intoRaw(shape2Vel);
        let rawShape2 = shape2.intoRaw();

        const rawShapeCastHit = this.colliderSet.raw.coCastShape(
            this.handle,
            rawCollider1Vel,
            rawShape2,
            rawShape2Pos,
            rawShape2Rot,
            rawShape2Vel,
            targetDistance,
            maxToi,
            stopAtPenetration,
        );

        let result = null;
        if (rawShapeCastHit) {
            rawShapeCastHit.getComponents(scratchBuffer);
            result = ShapeCastHit.fromBuffer(null, scratchBuffer, target);
            rawShapeCastHit.free();
        }

        rawCollider1Vel.free();
        rawShape2Pos.free();
        rawShape2Rot.free();
        rawShape2Vel.free();
        rawShape2.free();

        return result;
    }

    /**
     * Computes the smallest time between this and the given collider under translational movement are separated by a distance smaller or equal to distance.
     *
     * @param collider1Vel - The constant velocity of the current collider to cast (i.e. the cast direction).
     * @param collider2 - The collider to cast against.
     * @param collider2Vel - The constant velocity of the second collider.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                         will be returned.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param {ColliderShapeCastHit?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public castCollider(
        collider1Vel: Vector,
        collider2: Collider,
        collider2Vel: Vector,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        target?: ColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        let rawCollider1Vel = VectorOps.intoRaw(collider1Vel);
        let rawCollider2Vel = VectorOps.intoRaw(collider2Vel);

        const rawColliderShapeCastHit = this.colliderSet.raw.coCastCollider(
            this.handle,
            rawCollider1Vel,
            collider2.handle,
            rawCollider2Vel,
            targetDistance,
            maxToi,
            stopAtPenetration,
        );

        let result = null;
        if (rawColliderShapeCastHit) {
            const colliderHandle: number =
                rawColliderShapeCastHit.colliderHandle();
            rawColliderShapeCastHit.getComponents(scratchBuffer);
            result = ColliderShapeCastHit.fromBuffer(
                this.colliderSet.get(colliderHandle),
                scratchBuffer,
                target,
            );
            rawColliderShapeCastHit.free();
        }

        rawCollider1Vel.free();
        rawCollider2Vel.free();

        return result;
    }

    public intersectsShape(
        shape2: Shape,
        shapePos2: Vector,
        shapeRot2: Rotation,
    ): boolean {
        let rawPos2 = VectorOps.intoRaw(shapePos2);
        let rawRot2 = RotationOps.intoRaw(shapeRot2);
        let rawShape2 = shape2.intoRaw();

        let result = this.colliderSet.raw.coIntersectsShape(
            this.handle,
            rawShape2,
            rawPos2,
            rawRot2,
        );

        rawPos2.free();
        rawRot2.free();
        rawShape2.free();

        return result;
    }

    /**
     * Computes one pair of contact points between the shape owned by this collider and the given shape.
     *
     * @param shape2 - The second shape.
     * @param shape2Pos - The initial position of the second shape.
     * @param shape2Rot - The rotation of the second shape.
     * @param prediction - The prediction value, if the shapes are separated by a distance greater than this value, test will fail.
     * @returns `null` if the shapes are separated by a distance greater than prediction, otherwise contact details. The result is given in world-space.
     */
    contactShape(
        shape2: Shape,
        shape2Pos: Vector,
        shape2Rot: Rotation,
        prediction: number,
        target?: ShapeContact,
    ): ShapeContact | null {
        let rawPos2 = VectorOps.intoRaw(shape2Pos);
        let rawRot2 = RotationOps.intoRaw(shape2Rot);
        let rawShape2 = shape2.intoRaw();

        let result = ShapeContact.fromBuffer(
            this.colliderSet.raw.coContactShape(
                this.handle,
                rawShape2,
                rawPos2,
                rawRot2,
                prediction,
            ),
            target,
        );

        rawPos2.free();
        rawRot2.free();
        rawShape2.free();

        return result;
    }

    /**
     * Computes one pair of contact points between the collider and the given collider.
     *
     * @param collider2 - The second collider.
     * @param prediction - The prediction value, if the shapes are separated by a distance greater than this value, test will fail.
     * @returns `null` if the shapes are separated by a distance greater than prediction, otherwise contact details. The result is given in world-space.
     */
    contactCollider(
        collider2: Collider,
        prediction: number,
        target?: ShapeContact,
    ): ShapeContact | null {
        let result = ShapeContact.fromBuffer(
            this.colliderSet.raw.coContactCollider(
                this.handle,
                collider2.handle,
                prediction,
            ),
            target,
        );

        return result;
    }

    /**
     * Find the closest intersection between a ray and this collider.
     *
     * This also computes the normal at the hit point.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @returns The time-of-impact between this collider and the ray, or `-1` if there is no intersection.
     */
    public castRay(ray: Ray, maxToi: number, solid: boolean): number {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let result = this.colliderSet.raw.coCastRay(
            this.handle,
            rawOrig,
            rawDir,
            maxToi,
            solid,
        );

        rawOrig.free();
        rawDir.free();

        return result;
    }

    /**
     * Find the closest intersection between a ray and this collider.
     *
     * This also computes the normal at the hit point.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     */
    public castRayAndGetNormal(
        ray: Ray,
        maxToi: number,
        solid: boolean,
        target?: RayIntersection,
    ): RayIntersection | null {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let result = RayIntersection.fromBuffer(
            this.colliderSet.raw.coCastRayAndGetNormal(
                this.handle,
                rawOrig,
                rawDir,
                maxToi,
                solid,
            ),
            target,
        );

        rawOrig.free();
        rawDir.free();

        return result;
    }
}

export enum MassPropsMode {
    Density,
    Mass,
    MassProps,
}

export class ColliderDesc {
    enabled: boolean;
    shape: Shape;
    massPropsMode: MassPropsMode;
    mass: number;
    centerOfMass: Vector;
    // #if DIM2
    principalAngularInertia: number;
    rotationsEnabled: boolean;
    // #endif
    // #if DIM3
    principalAngularInertia: Vector;
    angularInertiaLocalFrame: Rotation;
    // #endif
    density: number;
    friction: number;
    restitution: number;
    rotation: Rotation;
    translation: Vector;
    isSensor: boolean;
    collisionGroups: InteractionGroups;
    solverGroups: InteractionGroups;
    frictionCombineRule: CoefficientCombineRule;
    restitutionCombineRule: CoefficientCombineRule;
    activeEvents: ActiveEvents;
    activeHooks: ActiveHooks;
    activeCollisionTypes: ActiveCollisionTypes;
    contactForceEventThreshold: number;
    contactSkin: number;

    /**
     * Initializes a collider descriptor from the collision shape.
     *
     * @param shape - The shape of the collider being built.
     */
    constructor(shape: Shape) {
        this.enabled = true;
        this.shape = shape;
        this.massPropsMode = MassPropsMode.Density;
        this.density = 1.0;
        this.friction = 0.5;
        this.restitution = 0.0;
        this.rotation = RotationOps.identity();
        this.translation = VectorOps.zeros();
        this.isSensor = false;
        this.collisionGroups = 0xffff_ffff;
        this.solverGroups = 0xffff_ffff;
        this.frictionCombineRule = CoefficientCombineRule.Average;
        this.restitutionCombineRule = CoefficientCombineRule.Average;
        this.activeCollisionTypes = ActiveCollisionTypes.DEFAULT;
        this.activeEvents = ActiveEvents.NONE;
        this.activeHooks = ActiveHooks.NONE;
        this.mass = 0.0;
        this.centerOfMass = VectorOps.zeros();
        this.contactForceEventThreshold = 0.0;
        this.contactSkin = 0.0;

        // #if DIM2
        this.principalAngularInertia = 0.0;
        this.rotationsEnabled = true;
        // #endif
        // #if DIM3
        this.principalAngularInertia = VectorOps.zeros();
        this.angularInertiaLocalFrame = RotationOps.identity();
        // #endif
    }

    /**
     * Create a new collider descriptor with a ball shape.
     *
     * @param radius - The radius of the ball.
     */
    public static ball(radius: number): ColliderDesc {
        const shape = new Ball(radius);
        return new ColliderDesc(shape);
    }

    /**
     * Create a new collider descriptor with a capsule shape.
     *
     * @param halfHeight - The half-height of the capsule, along the `y` axis.
     * @param radius - The radius of the capsule basis.
     */
    public static capsule(halfHeight: number, radius: number): ColliderDesc {
        const shape = new Capsule(halfHeight, radius);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new segment shape.
     *
     * @param a - The first point of the segment.
     * @param b - The second point of the segment.
     */
    public static segment(a: Vector, b: Vector): ColliderDesc {
        const shape = new Segment(a, b);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new triangle shape.
     *
     * @param a - The first point of the triangle.
     * @param b - The second point of the triangle.
     * @param c - The third point of the triangle.
     */
    public static triangle(a: Vector, b: Vector, c: Vector): ColliderDesc {
        const shape = new Triangle(a, b, c);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new triangle shape with round corners.
     *
     * @param a - The first point of the triangle.
     * @param b - The second point of the triangle.
     * @param c - The third point of the triangle.
     * @param borderRadius - The radius of the borders of this triangle. In 3D,
     *   this is also equal to half the thickness of the triangle.
     */
    public static roundTriangle(
        a: Vector,
        b: Vector,
        c: Vector,
        borderRadius: number,
    ): ColliderDesc {
        const shape = new RoundTriangle(a, b, c, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a polyline shape.
     *
     * @param vertices - The coordinates of the polyline's vertices.
     * @param indices - The indices of the polyline's segments. If this is `undefined` or `null`,
     *    the vertices are assumed to describe a line strip.
     */
    public static polyline(
        vertices: Float32Array,
        indices?: Uint32Array | null,
    ): ColliderDesc {
        const shape = new Polyline(vertices, indices);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a shape made of voxels.
     *
     * @param data - Defines the set of voxels. If this is a `Int32Array` then
     *               each voxel is defined from its (signed) grid coordinates,
     *               with 3 (resp 2) contiguous integers per voxel in 3D (resp 2D).
     *               If this is a `Float32Array`, each voxel will be such that
     *               they contain at least one point from this array (where each
     *               point is defined from 3 (resp 2) contiguous numbers per point
     *               in 3D (resp 2D).
     * @param voxelSize - The size of each voxel.
     */
    public static voxels(
        voxels: Float32Array | Int32Array,
        voxelSize: Vector,
    ): ColliderDesc {
        const shape = new Voxels(voxels, voxelSize);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a triangle mesh shape.
     *
     * @param vertices - The coordinates of the triangle mesh's vertices.
     * @param indices - The indices of the triangle mesh's triangles.
     */
    public static trimesh(
        vertices: Float32Array,
        indices: Uint32Array,
        flags?: TriMeshFlags,
    ): ColliderDesc {
        const shape = new TriMesh(vertices, indices, flags);
        return new ColliderDesc(shape);
    }

    // #if DIM2
    /**
     * Creates a new collider descriptor with a rectangular shape.
     *
     * @param hx - The half-width of the rectangle along its local `x` axis.
     * @param hy - The half-width of the rectangle along its local `y` axis.
     */
    public static cuboid(hx: number, hy: number): ColliderDesc {
        const shape = new Cuboid(hx, hy);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a rectangular shape with round borders.
     *
     * @param hx - The half-width of the rectangle along its local `x` axis.
     * @param hy - The half-width of the rectangle along its local `y` axis.
     * @param borderRadius - The radius of the cuboid's borders.
     */
    public static roundCuboid(
        hx: number,
        hy: number,
        borderRadius: number,
    ): ColliderDesc {
        const shape = new RoundCuboid(hx, hy, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider description with a halfspace (infinite plane) shape.
     *
     * @param normal - The outward normal of the plane.
     */
    public static halfspace(normal: Vector): ColliderDesc {
        const shape = new HalfSpace(normal);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a heightfield shape.
     *
     * @param heights - The heights of the heightfield, along its local `y` axis.
     * @param scale - The scale factor applied to the heightfield.
     */
    public static heightfield(
        heights: Float32Array,
        scale: Vector,
    ): ColliderDesc {
        const shape = new Heightfield(heights, scale);
        return new ColliderDesc(shape);
    }

    /**
     * Computes the convex-hull of the given points and use the resulting
     * convex polygon as the shape for this new collider descriptor.
     *
     * @param points - The point that will be used to compute the convex-hull.
     */
    public static convexHull(points: Float32Array): ColliderDesc | null {
        const shape = new ConvexPolygon(points, false);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor that uses the given set of points assumed
     * to form a convex polyline (no convex-hull computation will be done).
     *
     * @param vertices - The vertices of the convex polyline.
     */
    public static convexPolyline(vertices: Float32Array): ColliderDesc | null {
        const shape = new ConvexPolygon(vertices, true);
        return new ColliderDesc(shape);
    }

    /**
     * Computes the convex-hull of the given points and use the resulting
     * convex polygon as the shape for this new collider descriptor. A
     * border is added to that convex polygon to give it round corners.
     *
     * @param points - The point that will be used to compute the convex-hull.
     * @param borderRadius - The radius of the round border added to the convex polygon.
     */
    public static roundConvexHull(
        points: Float32Array,
        borderRadius: number,
    ): ColliderDesc | null {
        const shape = new RoundConvexPolygon(points, borderRadius, false);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor that uses the given set of points assumed
     * to form a round convex polyline (no convex-hull computation will be done).
     *
     * @param vertices - The vertices of the convex polyline.
     * @param borderRadius - The radius of the round border added to the convex polyline.
     */
    public static roundConvexPolyline(
        vertices: Float32Array,
        borderRadius: number,
    ): ColliderDesc | null {
        const shape = new RoundConvexPolygon(vertices, borderRadius, true);
        return new ColliderDesc(shape);
    }

    // #endif

    // #if DIM3
    /**
     * Creates a new collider descriptor with a cuboid shape.
     *
     * @param hx - The half-width of the rectangle along its local `x` axis.
     * @param hy - The half-width of the rectangle along its local `y` axis.
     * @param hz - The half-width of the rectangle along its local `z` axis.
     */
    public static cuboid(hx: number, hy: number, hz: number): ColliderDesc {
        const shape = new Cuboid(hx, hy, hz);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a rectangular shape with round borders.
     *
     * @param hx - The half-width of the rectangle along its local `x` axis.
     * @param hy - The half-width of the rectangle along its local `y` axis.
     * @param hz - The half-width of the rectangle along its local `z` axis.
     * @param borderRadius - The radius of the cuboid's borders.
     */
    public static roundCuboid(
        hx: number,
        hy: number,
        hz: number,
        borderRadius: number,
    ): ColliderDesc {
        const shape = new RoundCuboid(hx, hy, hz, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor with a heightfield shape.
     *
     * @param nrows − The number of rows in the heights matrix.
     * @param ncols - The number of columns in the heights matrix.
     * @param heights - The heights of the heightfield along its local `y` axis,
     *                  provided as a matrix stored in column-major order.
     * @param scale - The scale factor applied to the heightfield.
     */
    public static heightfield(
        nrows: number,
        ncols: number,
        heights: Float32Array,
        scale: Vector,
        flags?: HeightFieldFlags,
    ): ColliderDesc {
        const shape = new Heightfield(nrows, ncols, heights, scale, flags);
        return new ColliderDesc(shape);
    }

    /**
     * Create a new collider descriptor with a cylinder shape.
     *
     * @param halfHeight - The half-height of the cylinder, along the `y` axis.
     * @param radius - The radius of the cylinder basis.
     */
    public static cylinder(halfHeight: number, radius: number): ColliderDesc {
        const shape = new Cylinder(halfHeight, radius);
        return new ColliderDesc(shape);
    }

    /**
     * Create a new collider descriptor with a cylinder shape with rounded corners.
     *
     * @param halfHeight - The half-height of the cylinder, along the `y` axis.
     * @param radius - The radius of the cylinder basis.
     * @param borderRadius - The radius of the cylinder's rounded edges and vertices.
     */
    public static roundCylinder(
        halfHeight: number,
        radius: number,
        borderRadius: number,
    ): ColliderDesc {
        const shape = new RoundCylinder(halfHeight, radius, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Create a new collider descriptor with a cone shape.
     *
     * @param halfHeight - The half-height of the cone, along the `y` axis.
     * @param radius - The radius of the cone basis.
     */
    public static cone(halfHeight: number, radius: number): ColliderDesc {
        const shape = new Cone(halfHeight, radius);
        return new ColliderDesc(shape);
    }

    /**
     * Create a new collider descriptor with a cone shape with rounded corners.
     *
     * @param halfHeight - The half-height of the cone, along the `y` axis.
     * @param radius - The radius of the cone basis.
     * @param borderRadius - The radius of the cone's rounded edges and vertices.
     */
    public static roundCone(
        halfHeight: number,
        radius: number,
        borderRadius: number,
    ): ColliderDesc {
        const shape = new RoundCone(halfHeight, radius, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Computes the convex-hull of the given points and use the resulting
     * convex polyhedron as the shape for this new collider descriptor.
     *
     * @param points - The point that will be used to compute the convex-hull.
     */
    public static convexHull(points: Float32Array): ColliderDesc | null {
        const shape = new ConvexPolyhedron(points, null);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor that uses the given set of points assumed
     * to form a convex polyline (no convex-hull computation will be done).
     *
     * @param vertices - The vertices of the convex polyline.
     */
    public static convexMesh(
        vertices: Float32Array,
        indices?: Uint32Array | null,
    ): ColliderDesc | null {
        const shape = new ConvexPolyhedron(vertices, indices);
        return new ColliderDesc(shape);
    }

    /**
     * Computes the convex-hull of the given points and use the resulting
     * convex polyhedron as the shape for this new collider descriptor. A
     * border is added to that convex polyhedron to give it round corners.
     *
     * @param points - The point that will be used to compute the convex-hull.
     * @param borderRadius - The radius of the round border added to the convex polyhedron.
     */
    public static roundConvexHull(
        points: Float32Array,
        borderRadius: number,
    ): ColliderDesc | null {
        const shape = new RoundConvexPolyhedron(points, null, borderRadius);
        return new ColliderDesc(shape);
    }

    /**
     * Creates a new collider descriptor that uses the given set of points assumed
     * to form a round convex polyline (no convex-hull computation will be done).
     *
     * @param vertices - The vertices of the convex polyline.
     * @param borderRadius - The radius of the round border added to the convex polyline.
     */
    public static roundConvexMesh(
        vertices: Float32Array,
        indices: Uint32Array | null,
        borderRadius: number,
    ): ColliderDesc | null {
        const shape = new RoundConvexPolyhedron(
            vertices,
            indices,
            borderRadius,
        );
        return new ColliderDesc(shape);
    }

    // #endif

    // #if DIM2
    /**
     * Sets the position of the collider to be created relative to the rigid-body it is attached to.
     */
    public setTranslation(x: number, y: number): ColliderDesc {
        if (typeof x != "number" || typeof y != "number")
            throw TypeError("The translation components must be numbers.");

        this.translation = {x: x, y: y};
        return this;
    }

    // #endif

    // #if DIM3
    /**
     * Sets the position of the collider to be created relative to the rigid-body it is attached to.
     */
    public setTranslation(x: number, y: number, z: number): ColliderDesc {
        if (
            typeof x != "number" ||
            typeof y != "number" ||
            typeof z != "number"
        )
            throw TypeError("The translation components must be numbers.");

        this.translation = {x: x, y: y, z: z};
        return this;
    }

    // #endif

    /**
     * Sets the rotation of the collider to be created relative to the rigid-body it is attached to.
     *
     * @param rot - The rotation of the collider to be created relative to the rigid-body it is attached to.
     */
    public setRotation(rot: Rotation): ColliderDesc {
        // #if DIM2
        this.rotation = rot;
        // #endif
        // #if DIM3
        RotationOps.copy(this.rotation, rot);
        // #endif
        return this;
    }

    /**
     * Sets whether or not the collider being created is a sensor.
     *
     * A sensor collider does not take part of the physics simulation, but generates
     * proximity events.
     *
     * @param sensor - Set to `true` of the collider built is to be a sensor.
     */
    public setSensor(sensor: boolean): ColliderDesc {
        this.isSensor = sensor;
        return this;
    }

    /**
     * Sets whether the created collider will be enabled or disabled.
     * @param enabled − If set to `false` the collider will be disabled at creation.
     */
    public setEnabled(enabled: boolean): ColliderDesc {
        this.enabled = enabled;
        return this;
    }

    /**
     * Sets the contact skin of the collider.
     *
     * The contact skin acts as if the collider was enlarged with a skin of width `skin_thickness`
     * around it, keeping objects further apart when colliding.
     *
     * A non-zero contact skin can increase performance, and in some cases, stability. However
     * it creates a small gap between colliding object (equal to the sum of their skin). If the
     * skin is sufficiently small, this might not be visually significant or can be hidden by the
     * rendering assets.
     */
    public setContactSkin(thickness: number): ColliderDesc {
        this.contactSkin = thickness;
        return this;
    }

    /**
     * Sets the density of the collider being built.
     *
     * The mass and angular inertia tensor will be computed automatically based on this density and the collider’s shape.
     *
     * @param density - The density to set, must be greater or equal to 0. A density of 0 means that this collider
     *                  will not affect the mass or angular inertia of the rigid-body it is attached to.
     */
    public setDensity(density: number): ColliderDesc {
        this.massPropsMode = MassPropsMode.Density;
        this.density = density;
        return this;
    }

    /**
     * Sets the mass of the collider being built.
     *
     * The angular inertia tensor will be computed automatically based on this mass and the collider’s shape.
     *
     * @param mass - The mass to set, must be greater or equal to 0.
     */
    public setMass(mass: number): ColliderDesc {
        this.massPropsMode = MassPropsMode.Mass;
        this.mass = mass;
        return this;
    }

    // #if DIM2
    /**
     * Sets the mass properties of the collider being built.
     *
     * This replaces the mass-properties automatically computed from the collider's density and shape.
     * These mass-properties will be added to the mass-properties of the rigid-body this collider will be attached to.
     *
     * @param mass − The mass of the collider to create.
     * @param centerOfMass − The center-of-mass of the collider to create.
     * @param principalAngularInertia − The principal angular inertia of the collider to create.
     */
    public setMassProperties(
        mass: number,
        centerOfMass: Vector,
        principalAngularInertia: number,
    ): ColliderDesc {
        this.massPropsMode = MassPropsMode.MassProps;
        this.mass = mass;
        VectorOps.copy(this.centerOfMass, centerOfMass);
        this.principalAngularInertia = principalAngularInertia;
        return this;
    }

    // #endif

    // #if DIM3
    /**
     * Sets the mass properties of the collider being built.
     *
     * This replaces the mass-properties automatically computed from the collider's density and shape.
     * These mass-properties will be added to the mass-properties of the rigid-body this collider will be attached to.
     *
     * @param mass − The mass of the collider to create.
     * @param centerOfMass − The center-of-mass of the collider to create.
     * @param principalAngularInertia − The initial principal angular inertia of the collider to create.
     *                                  These are the eigenvalues of the angular inertia matrix.
     * @param angularInertiaLocalFrame − The initial local angular inertia frame of the collider to create.
     *                                   These are the eigenvectors of the angular inertia matrix.
     */
    public setMassProperties(
        mass: number,
        centerOfMass: Vector,
        principalAngularInertia: Vector,
        angularInertiaLocalFrame: Rotation,
    ): ColliderDesc {
        this.massPropsMode = MassPropsMode.MassProps;
        this.mass = mass;
        VectorOps.copy(this.centerOfMass, centerOfMass);
        VectorOps.copy(this.principalAngularInertia, principalAngularInertia);
        RotationOps.copy(
            this.angularInertiaLocalFrame,
            angularInertiaLocalFrame,
        );
        return this;
    }

    // #endif

    /**
     * Sets the restitution coefficient of the collider to be created.
     *
     * @param restitution - The restitution coefficient in `[0, 1]`. A value of 0 (the default) means no bouncing behavior
     *                   while 1 means perfect bouncing (though energy may still be lost due to numerical errors of the
     *                   constraints solver).
     */
    public setRestitution(restitution: number): ColliderDesc {
        this.restitution = restitution;
        return this;
    }

    /**
     * Sets the friction coefficient of the collider to be created.
     *
     * @param friction - The friction coefficient. Must be greater or equal to 0. This is generally smaller than 1. The
     *                   higher the coefficient, the stronger friction forces will be for contacts with the collider
     *                   being built.
     */
    public setFriction(friction: number): ColliderDesc {
        this.friction = friction;
        return this;
    }

    /**
     * Sets the rule used to combine the friction coefficients of two colliders
     * colliders involved in a contact.
     *
     * @param rule − The combine rule to apply.
     */
    public setFrictionCombineRule(rule: CoefficientCombineRule): ColliderDesc {
        this.frictionCombineRule = rule;
        return this;
    }

    /**
     * Sets the rule used to combine the restitution coefficients of two colliders
     * colliders involved in a contact.
     *
     * @param rule − The combine rule to apply.
     */
    public setRestitutionCombineRule(
        rule: CoefficientCombineRule,
    ): ColliderDesc {
        this.restitutionCombineRule = rule;
        return this;
    }

    /**
     * Sets the collision groups used by this collider.
     *
     * Two colliders will interact iff. their collision groups are compatible.
     * See the documentation of `InteractionGroups` for details on teh used bit pattern.
     *
     * @param groups - The collision groups used for the collider being built.
     */
    public setCollisionGroups(groups: InteractionGroups): ColliderDesc {
        this.collisionGroups = groups;
        return this;
    }

    /**
     * Sets the solver groups used by this collider.
     *
     * Forces between two colliders in contact will be computed iff their solver
     * groups are compatible.
     * See the documentation of `InteractionGroups` for details on the used bit pattern.
     *
     * @param groups - The solver groups used for the collider being built.
     */
    public setSolverGroups(groups: InteractionGroups): ColliderDesc {
        this.solverGroups = groups;
        return this;
    }

    /**
     * Set the physics hooks active for this collider.
     *
     * Use this to enable custom filtering rules for contact/intersecstion pairs involving this collider.
     *
     * @param activeHooks - The hooks active for contact/intersection pairs involving this collider.
     */
    public setActiveHooks(activeHooks: ActiveHooks): ColliderDesc {
        this.activeHooks = activeHooks;
        return this;
    }

    /**
     * Set the events active for this collider.
     *
     * Use this to enable contact and/or intersection event reporting for this collider.
     *
     * @param activeEvents - The events active for contact/intersection pairs involving this collider.
     */
    public setActiveEvents(activeEvents: ActiveEvents): ColliderDesc {
        this.activeEvents = activeEvents;
        return this;
    }

    /**
     * Set the collision types active for this collider.
     *
     * @param activeCollisionTypes - The hooks active for contact/intersection pairs involving this collider.
     */
    public setActiveCollisionTypes(
        activeCollisionTypes: ActiveCollisionTypes,
    ): ColliderDesc {
        this.activeCollisionTypes = activeCollisionTypes;
        return this;
    }

    /**
     * Sets the total force magnitude beyond which a contact force event can be emitted.
     *
     * @param threshold - The force threshold to set.
     */
    public setContactForceEventThreshold(threshold: number): ColliderDesc {
        this.contactForceEventThreshold = threshold;
        return this;
    }
}
