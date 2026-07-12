import {RigidBodyHandle} from "../dynamics";
import {ColliderHandle} from "../geometry";

export enum ActiveHooks {
    NONE = 0,
    FILTER_CONTACT_PAIRS = 0b0001,
    FILTER_INTERSECTION_PAIRS = 0b0010,
    // MODIFY_SOLVER_CONTACTS = 0b0100, /* Not supported yet in JS. */
}

export enum SolverFlags {
    EMPTY = 0b000,
    COMPUTE_IMPULSE = 0b001,
}

export interface PhysicsHooks {
    /**
     * Function that determines if contacts computation should happen between two colliders, and how the
     * constraints solver should behave for these contacts.
     *
     * This will only be executed and taken into account if at least one of the involved colliders contains the
     * `ActiveHooks.FILTER_CONTACT_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterContactPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): SolverFlags | null;

    /**
     * Function that determines if intersection computation should happen between two colliders (where at least
     * one is a sensor).
     *
     * This will only be executed and taken into account if `one of the involved colliders contains the
     * `ActiveHooks.FILTER_INTERSECTION_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterIntersectionPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): boolean;
}
