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
     * Starts stamping the writes of every component with change ticks, for `deltaSince`.
     * Restarting it invalidates every tick issued before.
     */
    public enableChangeTracking(
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        this.raw.enableChangeTracking(
            islands.raw,
            broadPhase.raw,
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            softBodies.raw,
            impulseJoints.raw,
            multibodyJoints.raw,
        );
    }

    /**
     * Stops stamping writes on every component, and drops the stamps.
     */
    public disableChangeTracking(
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ) {
        this.raw.disableChangeTracking(
            islands.raw,
            broadPhase.raw,
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            softBodies.raw,
            impulseJoints.raw,
            multibodyJoints.raw,
        );
    }

    /**
     * Serializes the changes made to the physics state since the tick `since` (or the whole
     * state if `since` is not given). Requires change tracking.
     *
     * Throws if change tracking is disabled, or if `since` was not issued by the current change
     * trackers.
     *
     * @param since - A tick returned by an earlier `deltaSince` or by `applyDelta`.
     * @returns The serialized delta, and the tick a receiver is at once it applied it.
     */
    public deltaSince(
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
        let rawGra = VectorOps.intoRaw(gravity);
        let rawDelta: RawWorldDelta;
        try {
            rawDelta = this.raw.deltaSince(
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
            );
        } finally {
            rawGra.free();
        }

        const res = {delta: rawDelta.delta, tick: rawDelta.tick};
        rawDelta.free();
        return res;
    }

    /**
     * Patches a delta serialized by `deltaSince` into the given components and integration
     * parameters.
     *
     * Throws if `delta` is malformed or was produced against another state than the current
     * one; the components patched before the error keep their new state.
     *
     * @returns The gravity carried by the delta, and the tick to acknowledge to the sender.
     */
    public applyDelta(
        delta: Uint8Array,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        softBodies: SoftBodySet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
    ): {gravity: Vector; tick: Uint8Array} {
        let rawGra = RawVector.zero();
        let tick: Uint8Array;
        try {
            tick = this.raw.applyDelta(
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
            rawGra.free();
            throw e;
        }
        return {gravity: VectorOps.fromRaw(rawGra), tick};
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
