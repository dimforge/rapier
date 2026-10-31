import {RawSerializationPipeline, RawVector, RawWorldDelta} from "../raw";
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
     * Throws an `Error` with `kind` `"tracking_disabled"` if change tracking is disabled.
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
     * Throws an `Error` with `kind` `"malformed"` if `tick` is not a serialized tick.
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
     * Throws an `Error` with a `kind` property, see `World.takeDelta`.
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
     * Throws an `Error` with a `kind` property, see `World.applyDelta`. Once the delta decoded,
     * the gravity, integration parameters and the components patched before the error keep
     * their new state.
     *
     * @param onGravity - Called with the resulting gravity, also when the delta fails to apply.
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
    ): Uint8Array {
        let rawGra = VectorOps.intoRaw(gravity);
        try {
            return this.raw.applyDelta(
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
        } finally {
            onGravity(VectorOps.fromRaw(rawGra));
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
