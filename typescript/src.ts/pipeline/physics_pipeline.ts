import {RawPhysicsPipeline} from "../raw";
import {Vector, VectorOps} from "../math";
import {
    IntegrationParameters,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodyHandle,
    RigidBodySet,
    CCDSolver,
    IslandManager,
} from "../dynamics";
import {
    BroadPhase,
    ColliderHandle,
    ColliderSet,
    NarrowPhase,
} from "../geometry";
import {EventQueue} from "./event_queue";
import {PhysicsHooks} from "./physics_hooks";

export class PhysicsPipeline {
    raw: RawPhysicsPipeline;

    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    constructor(raw?: RawPhysicsPipeline) {
        this.raw = raw || new RawPhysicsPipeline();
    }

    public step(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
        ccdSolver: CCDSolver,
        eventQueue?: EventQueue,
        hooks?: PhysicsHooks,
    ) {
        let rawG = VectorOps.intoRaw(gravity);

        if (!!eventQueue) {
            this.raw.stepWithEvents(
                rawG,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                ccdSolver.raw,
                eventQueue.raw,
                hooks,
                !!hooks ? hooks.filterContactPair : null,
                !!hooks ? hooks.filterIntersectionPair : null,
            );
        } else {
            this.raw.step(
                rawG,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                ccdSolver.raw,
            );
        }

        rawG.free();
    }
}
