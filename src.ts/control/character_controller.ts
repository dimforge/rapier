import {RawKinematicCharacterController, RawCharacterCollision} from "../raw";
import {Rotation, Vector, VectorOps, scratchBuffer} from "../math";
import {
    BroadPhase,
    Collider,
    ColliderSet,
    InteractionGroups,
    NarrowPhase,
    Shape,
} from "../geometry";
import {QueryFilterFlags, World} from "../pipeline";
import {IntegrationParameters, RigidBody, RigidBodySet} from "../dynamics";

/**
 * A collision between the character and an obstacle hit on its path.
 */
export class CharacterCollision {
    /** The collider involved in the collision. Null if the collider no longer exists in the physics world. */
    public collider: Collider | null;
    /** The translation delta applied to the character before this collision took place. */
    public translationDeltaApplied: Vector;
    /** The translation delta the character would move after this collision if there is no other obstacles. */
    public translationDeltaRemaining: Vector;
    /** The time-of-impact between the character and the obstacles. */
    public toi: number;
    /** The world-space contact point on the collider when the collision happens. */
    public witness1: Vector;
    /** The local-space contact point on the character when the collision happens. */
    public witness2: Vector;
    /** The world-space outward contact normal on the collider when the collision happens. */
    public normal1: Vector;
    /** The local-space outward contact normal on the character when the collision happens. */
    public normal2: Vector;
}

/**
 * A character controller for controlling kinematic bodies and parentless colliders by hitting
 * and sliding against obstacles.
 */
export class KinematicCharacterController {
    private raw: RawKinematicCharacterController;
    private rawCharacterCollision: RawCharacterCollision;

    private params: IntegrationParameters;
    private broadPhase: BroadPhase;
    private narrowPhase: NarrowPhase;
    private bodies: RigidBodySet;
    private colliders: ColliderSet;
    private _applyImpulsesToDynamicBodies: boolean;
    private _characterMass: number | null;

    constructor(
        offset: number,
        params: IntegrationParameters,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
    ) {
        this.params = params;
        this.bodies = bodies;
        this.colliders = colliders;
        this.broadPhase = broadPhase;
        this.narrowPhase = narrowPhase;
        this.raw = new RawKinematicCharacterController(offset);
        this.rawCharacterCollision = new RawCharacterCollision();
        this._applyImpulsesToDynamicBodies = false;
        this._characterMass = null;
    }

    /** @internal */
    public free() {
        if (!!this.raw) {
            this.raw.free();
            this.rawCharacterCollision.free();
        }

        this.raw = undefined;
        this.rawCharacterCollision = undefined;
    }

    /**
     * The direction that goes "up". Used to determine where the floor is, and the floor’s angle.
     */
    public up(): Vector {
        return this.raw.up();
    }

    /**
     * Sets the direction that goes "up". Used to determine where the floor is, and the floor’s angle.
     */
    public setUp(vector: Vector) {
        let rawVect = VectorOps.intoRaw(vector);
        return this.raw.setUp(rawVect);
        rawVect.free();
    }

    public applyImpulsesToDynamicBodies(): boolean {
        return this._applyImpulsesToDynamicBodies;
    }

    public setApplyImpulsesToDynamicBodies(enabled: boolean) {
        this._applyImpulsesToDynamicBodies = enabled;
    }

    /**
     * Returns the custom value of the character mass, if it was set by `this.setCharacterMass`.
     */
    public characterMass(): number | null {
        return this._characterMass;
    }

    /**
     * Set the mass of the character to be used for impulse resolution if `self.applyImpulsesToDynamicBodies`
     * is set to `true`.
     *
     * If no character mass is set explicitly (or if it is set to `null`) it is automatically assumed to be equal
     * to the mass of the rigid-body the character collider is attached to; or equal to 0 if the character collider
     * isn’t attached to any rigid-body.
     *
     * @param mass - The mass to set.
     */
    public setCharacterMass(mass: number | null) {
        this._characterMass = mass;
    }

    /**
     * A small gap to preserve between the character and its surroundings.
     *
     * This value should not be too large to avoid visual artifacts, but shouldn’t be too small
     * (must not be zero) to improve numerical stability of the character controller.
     */
    public offset(): number {
        return this.raw.offset();
    }

    /**
     * Sets a small gap to preserve between the character and its surroundings.
     *
     * This value should not be too large to avoid visual artifacts, but shouldn’t be too small
     * (must not be zero) to improve numerical stability of the character controller.
     */
    public setOffset(value: number) {
        this.raw.setOffset(value);
    }

    /// Increase this number if your character appears to get stuck when sliding against surfaces.
    ///
    /// This is a small distance applied to the movement toward the contact normals of shapes hit
    /// by the character controller. This helps shape-casting not getting stuck in an always-penetrating
    /// state during the sliding calculation.
    ///
    /// This value should remain fairly small since it can introduce artificial "bumps" when sliding
    /// along a flat surface.
    public normalNudgeFactor(): number {
        return this.raw.normalNudgeFactor();
    }

    /// Increase this number if your character appears to get stuck when sliding against surfaces.
    ///
    /// This is a small distance applied to the movement toward the contact normals of shapes hit
    /// by the character controller. This helps shape-casting not getting stuck in an always-penetrating
    /// state during the sliding calculation.
    ///
    /// This value should remain fairly small since it can introduce artificial "bumps" when sliding
    /// along a flat surface.
    public setNormalNudgeFactor(value: number) {
        this.raw.setNormalNudgeFactor(value);
    }

    /**
     * Is sliding against obstacles enabled?
     */
    public slideEnabled(): boolean {
        return this.raw.slideEnabled();
    }

    /**
     * Enable or disable sliding against obstacles.
     */
    public setSlideEnabled(enabled: boolean) {
        this.raw.setSlideEnabled(enabled);
    }

    /**
     * The maximum step height a character can automatically step over.
     */
    public autostepMaxHeight(): number | null {
        return this.raw.autostepMaxHeight();
    }

    /**
     * The minimum width of free space that must be available after stepping on a stair.
     */
    public autostepMinWidth(): number | null {
        return this.raw.autostepMinWidth();
    }

    /**
     * Can the character automatically step over dynamic bodies too?
     */
    public autostepIncludesDynamicBodies(): boolean | null {
        return this.raw.autostepIncludesDynamicBodies();
    }

    /**
     * Is automatically stepping over small objects enabled?
     */
    public autostepEnabled(): boolean {
        return this.raw.autostepEnabled();
    }

    /**
     * Enabled automatically stepping over small objects.
     *
     * @param maxHeight - The maximum step height a character can automatically step over.
     * @param minWidth - The minimum width of free space that must be available after stepping on a stair.
     * @param includeDynamicBodies - Can the character automatically step over dynamic bodies too?
     */
    public enableAutostep(
        maxHeight: number,
        minWidth: number,
        includeDynamicBodies: boolean,
    ) {
        this.raw.enableAutostep(maxHeight, minWidth, includeDynamicBodies);
    }

    /**
     * Disable automatically stepping over small objects.
     */
    public disableAutostep() {
        return this.raw.disableAutostep();
    }

    /**
     * The maximum angle (radians) between the floor’s normal and the `up` vector that the
     * character is able to climb.
     */
    public maxSlopeClimbAngle(): number {
        return this.raw.maxSlopeClimbAngle();
    }

    /**
     * Sets the maximum angle (radians) between the floor’s normal and the `up` vector that the
     * character is able to climb.
     */
    public setMaxSlopeClimbAngle(angle: number) {
        this.raw.setMaxSlopeClimbAngle(angle);
    }

    /**
     * The minimum angle (radians) between the floor’s normal and the `up` vector before the
     * character starts to slide down automatically.
     */
    public minSlopeSlideAngle(): number {
        return this.raw.minSlopeSlideAngle();
    }

    /**
     * Sets the minimum angle (radians) between the floor’s normal and the `up` vector before the
     * character starts to slide down automatically.
     */
    public setMinSlopeSlideAngle(angle: number) {
        this.raw.setMinSlopeSlideAngle(angle);
    }

    /**
     * If snap-to-ground is enabled, should the character be automatically snapped to the ground if
     * the distance between the ground and its feet are smaller than the specified threshold?
     */
    public snapToGroundDistance(): number | null {
        return this.raw.snapToGroundDistance();
    }

    /**
     * Enables automatically snapping the character to the ground if the distance between
     * the ground and its feet are smaller than the specified threshold.
     */
    public enableSnapToGround(distance: number) {
        this.raw.enableSnapToGround(distance);
    }

    /**
     * Disables automatically snapping the character to the ground.
     */
    public disableSnapToGround() {
        this.raw.disableSnapToGround();
    }

    /**
     * Is automatically snapping the character to the ground enabled?
     */
    public snapToGroundEnabled(): boolean {
        return this.raw.snapToGroundEnabled();
    }

    /**
     * Computes the movement the given collider is able to execute after hitting and sliding on obstacles.
     *
     * @param collider - The collider to move.
     * @param desiredTranslationDelta - The desired collider movement.
     * @param filterFlags - Flags for excluding whole subsets of colliders from the obstacles taken into account.
     * @param filterGroups - Groups for excluding colliders with incompatible collision groups from the obstacles
     *                       taken into account.
     * @param filterPredicate - Any collider for which this closure returns `false` will be excluded from the
     *                          obstacles taken into account.
     */
    public computeColliderMovement(
        collider: Collider,
        desiredTranslationDelta: Vector,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        let rawTranslationDelta = VectorOps.intoRaw(desiredTranslationDelta);
        this.raw.computeColliderMovement(
            this.params.dt,
            this.broadPhase.raw,
            this.narrowPhase.raw,
            this.bodies.raw,
            this.colliders.raw,
            collider.handle,
            rawTranslationDelta,
            this._applyImpulsesToDynamicBodies,
            this._characterMass,
            filterFlags,
            filterGroups,
            this.colliders.castClosure(filterPredicate),
        );
        rawTranslationDelta.free();
    }

    /**
     * The movement computed by the last call to `this.computeColliderMovement`.
     */
    public computedMovement(target?: Vector): Vector {
        this.raw.computedMovement(scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The result of ground detection computed by the last call to `this.computeColliderMovement`.
     */
    public computedGrounded(): boolean {
        return this.raw.computedGrounded();
    }

    /**
     * The number of collisions against obstacles detected along the path of the last call
     * to `this.computeColliderMovement`.
     */
    public numComputedCollisions(): number {
        return this.raw.numComputedCollisions();
    }

    /**
     * Returns the collision against one of the obstacles detected along the path of the last
     * call to `this.computeColliderMovement`.
     *
     * @param i - The i-th collision will be returned.
     * @param out - If this argument is set, it will be filled with the collision information.
     */
    public computedCollision(
        i: number,
        out?: CharacterCollision,
    ): CharacterCollision | null {
        if (!this.raw.computedCollision(i, this.rawCharacterCollision)) {
            return null;
        } else {
            let c = this.rawCharacterCollision;
            out = out ?? new CharacterCollision();
            c.translationDeltaApplied(scratchBuffer);
            out.translationDeltaApplied = VectorOps.fromBuffer(
                scratchBuffer,
                out.translationDeltaApplied,
            );
            c.translationDeltaRemaining(scratchBuffer);
            out.translationDeltaRemaining = VectorOps.fromBuffer(
                scratchBuffer,
                out.translationDeltaRemaining,
            );
            out.toi = c.toi();
            c.worldWitness1(scratchBuffer);
            out.witness1 = VectorOps.fromBuffer(scratchBuffer, out.witness1);
            c.worldWitness2(scratchBuffer);
            out.witness2 = VectorOps.fromBuffer(scratchBuffer, out.witness2);
            c.worldNormal1(scratchBuffer);
            out.normal1 = VectorOps.fromBuffer(scratchBuffer, out.normal1);
            c.worldNormal2(scratchBuffer);
            out.normal2 = VectorOps.fromBuffer(scratchBuffer, out.normal2);
            out.collider = this.colliders.get(c.handle());
            return out;
        }
    }
}
