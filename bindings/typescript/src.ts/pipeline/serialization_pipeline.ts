import {
    RawAppliedDelta,
    RawSerializationPipeline,
    RawVector,
    RawWorldDelta,
} from "../raw";
import {Vector, VectorOps} from "../math";
import {
    IntegrationParameters,
    IslandManager,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodySet,
    SoftBodySet,
} from "../dynamics";
import {BroadPhase, ColliderSet, NarrowPhase} from "../geometry";
import {World} from "./world";

/**
 * The category of a snapshot-delta error, its `kind` property:
 * - `"tracking_disabled"`: change tracking is disabled on the sender.
 * - `"unknown_baseline"`: the tick was not issued by the sender's current change trackers.
 * - `"baseline_mismatch"`: the delta was produced against another state than the receiver's.
 * - `"malformed"`: the bytes are not a delta or tick of this rapier.js build, or a delta to
 *   encode exceeds the 256 MiB size limit.
 * - `"missing_shape"`: the delta references a shape the receiver does not hold.
 */
export type SnapshotDeltaErrorKind =
    | "tracking_disabled"
    | "unknown_baseline"
    | "baseline_mismatch"
    | "malformed"
    | "missing_shape";

/**
 * The `Error` thrown by the snapshot-delta methods of `World`, with a machine-readable `kind`.
 */
export type SnapshotDeltaError = Error & {kind: SnapshotDeltaErrorKind};

/**
 * A pipeline for serializing the physics scene.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `serializationPipeline.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class SerializationPipeline {
    raw: RawSerializationPipeline;

    /**
     * Release the WASM memory occupied by this serialization pipeline.
     */
    free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    constructor(raw?: RawSerializationPipeline) {
        this.raw = raw || new RawSerializationPipeline();
    }

    /**
     * Serialize a complete physics state into a single byte array.
     * @param gravity - The current gravity affecting the simulation.
     * @param integrationParameters - The integration parameters of the simulation.
     * @param broadPhase - The broad-phase of the simulation.
     * @param narrowPhase - The narrow-phase of the simulation.
     * @param bodies - The rigid-bodies taking part into the simulation.
     * @param colliders - The colliders taking part into the simulation.
     * @param impulseJoints - The impulse joints taking part into the simulation.
     * @param multibodyJoints - The multibody joints taking part into the simulation.
     */
    public serializeAll(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ): Uint8Array {
        let rawGra = VectorOps.intoRaw(gravity);

        const res = this.raw.serializeAll(
            rawGra,
            integrationParameters.raw,
            islands.raw,
            broadPhase.raw,
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            softBodies.raw,
            impulseJoints.raw,
            multibodyJoints.raw,
        );
        rawGra.free();

        return res;
    }

    /**
     * Starts stamping the writes of every component with change ticks, for `takeDelta`.
     * Restarting it invalidates every tick issued before.
     */
    public enableChangeTracking(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        // The engine's default tracker ids repeat across page loads on wasm, so seed them here.
        const seed = randomSeed();
        this.withRawGravity(gravity, (rawGra) =>
            this.raw.enableChangeTracking(
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                seed[0],
                seed[1],
            ),
        );
    }

    /**
     * Stops stamping writes on every component, and drops the stamps.
     */
    public disableChangeTracking(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        this.withRawGravity(gravity, (rawGra) =>
            this.raw.disableChangeTracking(
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
            ),
        );
    }

    /**
     * The serialized tick naming the current physics state (see `World.changeTick`).
     *
     * Throws a `SnapshotDeltaError` of kind `"tracking_disabled"` if change tracking is
     * disabled.
     */
    public changeTick(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ): Uint8Array {
        return this.withRawGravity(gravity, (rawGra) =>
            this.raw.changeTick(
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
            ),
        );
    }

    /**
     * Declares that the components hold the sender's state at `tick` (see
     * `World.setDeltaBaseline`).
     *
     * Throws a `SnapshotDeltaError` of kind `"malformed"` if `tick` is not a serialized tick.
     */
    public setDeltaBaseline(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
        tick: Uint8Array,
    ) {
        this.withRawGravity(gravity, (rawGra) =>
            this.raw.setDeltaBaseline(
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                tick,
            ),
        );
    }

    /**
     * Serializes the changes made to the physics state since the tick `since` (or the whole
     * state if `since` is not given). Requires change tracking.
     *
     * Throws a `SnapshotDeltaError`, see `World.takeDelta`.
     *
     * @param since - A tick returned by an earlier `takeDelta`, by `changeTick`, or by
     *                `applyDelta`.
     * @returns The serialized delta, and the tick a receiver is at once it applied it.
     */
    public takeDelta(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
        since?: Uint8Array,
    ): {delta: Uint8Array; tick: Uint8Array} {
        const rawDelta: RawWorldDelta = this.withRawGravity(gravity, (rawGra) =>
            this.raw.takeDelta(
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                since,
            ),
        );
        const res = {delta: rawDelta.delta, tick: rawDelta.tick};
        rawDelta.free();
        return res;
    }

    /**
     * Patches a delta serialized by `takeDelta` into the given components and integration
     * parameters, starting from the given gravity.
     *
     * Throws a `SnapshotDeltaError`, see `World.applyDelta`. A delta that does not decode or
     * targets another baseline changes nothing; after any other error, the gravity, integration
     * parameters and the components patched before the error keep their new state.
     *
     * @param onGravity - Called with the resulting gravity, also when the patch fails partway,
     *                    but not when the delta does not decode or targets another baseline.
     * @param onChanges - Called with the handles the patch added, removed or modified, also
     *                    when the patch fails partway. Only valid during the call.
     * @returns The tick to acknowledge to the sender.
     */
    public applyDelta(
        delta: Uint8Array,
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
        onGravity: (gravity: Vector) => void,
        onChanges?: (changes: RawAppliedDelta) => void,
    ): Uint8Array {
        const rawGra = VectorOps.intoRaw(gravity);
        let applied: RawAppliedDelta;
        try {
            applied = this.raw.applyDelta(
                delta,
                rawGra,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                softBodies.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
            );
        } catch (e) {
            // The delta did not decode or targets another baseline: the world is untouched.
            rawGra.free();
            throw e;
        }
        // The world may be patched, even if `applied.error` is set.
        onGravity(VectorOps.fromRaw(rawGra));
        try {
            if (onChanges) onChanges(applied);
            const error = applied.error;
            if (error !== undefined) throw error;
            return applied.tick;
        } finally {
            applied.free();
        }
    }

    private withRawGravity<T>(gravity: Vector, f: (rawGra: RawVector) => T): T {
        let rawGra = VectorOps.intoRaw(gravity);
        try {
            return f(rawGra);
        } finally {
            rawGra.free();
        }
    }

    /**
     * Deserialize the complete physics state from a single byte array.
     *
     * @param data - The byte array to deserialize.
     */
    public deserializeAll(data: Uint8Array): World {
        return World.fromRaw(this.raw.deserializeAll(data));
    }
}

/** Two random 32-bit words, from `crypto.getRandomValues` when available. */
function randomSeed(): Uint32Array {
    const seed = new Uint32Array(2);
    const crypto = (globalThis as any).crypto;
    if (crypto && typeof crypto.getRandomValues === "function") {
        crypto.getRandomValues(seed);
    } else {
        seed[0] = Math.floor(Math.random() * 0x100000000);
        seed[1] = Math.floor(Math.random() * 0x100000000);
    }
    return seed;
}
