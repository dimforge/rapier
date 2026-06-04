import {RawContactForceEvent, RawEventQueue} from "../raw";
import {RigidBodyHandle} from "../dynamics";
import {Collider, ColliderHandle} from "../geometry";
import {Vector, VectorOps, scratchBuffer} from "../math";

/**
 * Flags indicating what events are enabled for colliders.
 */
export enum ActiveEvents {
    NONE = 0,
    /**
     * Enable collision events.
     */
    COLLISION_EVENTS = 0b0001,
    /**
     * Enable contact force events.
     */
    CONTACT_FORCE_EVENTS = 0b0010,
}

/**
 * Event occurring when the sum of the magnitudes of the
 * contact forces between two colliders exceed a threshold.
 *
 * This object should **not** be stored anywhere. Its properties can only be
 * read from within the closure given to `EventHandler.drainContactForceEvents`.
 */
export class TempContactForceEvent {
    raw: RawContactForceEvent;

    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    /**
     * The first collider involved in the contact.
     */
    public collider1(): ColliderHandle {
        return this.raw.collider1();
    }

    /**
     * The second collider involved in the contact.
     */
    public collider2(): ColliderHandle {
        return this.raw.collider2();
    }

    /**
     * The sum of all the forces between the two colliders.
     */
    public totalForce(target?: Vector): Vector {
        this.raw.total_force(scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The sum of the magnitudes of each force between the two colliders.
     *
     * Note that this is **not** the same as the magnitude of `self.total_force`.
     * Here we are summing the magnitude of all the forces, instead of taking
     * the magnitude of their sum.
     */
    public totalForceMagnitude(): number {
        return this.raw.total_force_magnitude();
    }

    /**
     * The world-space (unit) direction of the force with strongest magnitude.
     */
    public maxForceDirection(target?: Vector): Vector {
        this.raw.max_force_direction(scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The magnitude of the largest force at a contact point of this contact pair.
     */
    public maxForceMagnitude(): number {
        return this.raw.max_force_magnitude();
    }
}

/**
 * A structure responsible for collecting events generated
 * by the physics engine.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `eventQueue.free()`
 * once you are done using it.
 */
export class EventQueue {
    raw: RawEventQueue;

    /**
     * Creates a new event collector.
     *
     * @param autoDrain -setting this to `true` is strongly recommended. If true, the collector will
     * be automatically drained before each `world.step(collector)`. If false, the collector will
     * keep all events in memory unless it is manually drained/cleared; this may lead to unbounded use of
     * RAM if no drain is performed.
     */
    constructor(autoDrain: boolean, raw?: RawEventQueue) {
        this.raw = raw || new RawEventQueue(autoDrain);
    }

    /**
     * Release the WASM memory occupied by this event-queue.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    /**
     * Applies the given javascript closure on each collision event of this collector, then clear
     * the internal collision event buffer.
     *
     * @param f - JavaScript closure applied to each collision event. The
     * closure must take three arguments: two integers representing the handles of the colliders
     * involved in the collision, and a boolean indicating if the collision started (true) or stopped
     * (false).
     */
    public drainCollisionEvents(
        f: (
            handle1: ColliderHandle,
            handle2: ColliderHandle,
            started: boolean,
        ) => void,
    ) {
        this.raw.drainCollisionEvents(f);
    }

    /**
     * Applies the given javascript closure on each contact force event of this collector, then clear
     * the internal collision event buffer.
     *
     * @param f - JavaScript closure applied to each collision event. The
     *            closure must take one `TempContactForceEvent` argument.
     */
    public drainContactForceEvents(f: (event: TempContactForceEvent) => void) {
        let event = new TempContactForceEvent();
        this.raw.drainContactForceEvents((raw: RawContactForceEvent) => {
            event.raw = raw;
            f(event);
            event.free();
        });
    }

    /**
     * Removes all events contained by this collector
     */
    public clear() {
        this.raw.clear();
    }
}
