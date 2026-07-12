import {
    RawBroadPhase,
    RawCCDSolver,
    RawColliderSet,
    RawDeserializedWorld,
    RawIntegrationParameters,
    RawIslandManager,
    RawImpulseJointSet,
    RawMultibodyJointSet,
    RawNarrowPhase,
    RawPhysicsPipeline,
    RawRigidBodySet,
    RawSerializationPipeline,
    RawDebugRenderPipeline,
} from "../raw";

import {
    BroadPhase,
    Collider,
    ColliderDesc,
    ColliderHandle,
    ColliderSet,
    InteractionGroups,
    NarrowPhase,
    PointColliderProjection,
    Ray,
    RayColliderIntersection,
    RayColliderHit,
    Shape,
    ColliderShapeCastHit,
    TempContactManifold,
} from "../geometry";
import {
    CCDSolver,
    IntegrationParameters,
    IslandManager,
    ImpulseJoint,
    ImpulseJointHandle,
    MultibodyJoint,
    MultibodyJointHandle,
    JointData,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBody,
    RigidBodyDesc,
    RigidBodyHandle,
    RigidBodySet,
} from "../dynamics";
import {Rotation, Vector, VectorOps} from "../math";
import {PhysicsPipeline} from "./physics_pipeline";
import {QueryFilterFlags} from "./query_pipeline";
import {SerializationPipeline} from "./serialization_pipeline";
import {EventQueue} from "./event_queue";
import {PhysicsHooks} from "./physics_hooks";
import {DebugRenderBuffers, DebugRenderPipeline} from "./debug_render_pipeline";
import {
    KinematicCharacterController,
    PidAxesMask,
    PidController,
} from "../control";
import {Coarena} from "../coarena";

// #if DIM3
import {DynamicRayCastVehicleController} from "../control";

// #endif

/**
 * The physics world.
 *
 * This contains all the data-structures necessary for creating and simulating
 * bodies with contacts, joints, and external forces.
 */
export class World {
    public gravity: Vector;
    integrationParameters: IntegrationParameters;
    islands: IslandManager;
    broadPhase: BroadPhase;
    narrowPhase: NarrowPhase;
    bodies: RigidBodySet;
    colliders: ColliderSet;
    impulseJoints: ImpulseJointSet;
    multibodyJoints: MultibodyJointSet;
    ccdSolver: CCDSolver;
    physicsPipeline: PhysicsPipeline;
    serializationPipeline: SerializationPipeline;
    debugRenderPipeline: DebugRenderPipeline;
    characterControllers: Set<KinematicCharacterController>;
    pidControllers: Set<PidController>;

    // #if DIM3
    vehicleControllers: Set<DynamicRayCastVehicleController>;

    // #endif

    /**
     * Release the WASM memory occupied by this physics world.
     *
     * All the fields of this physics world will be freed as well,
     * so there is no need to call their `.free()` methods individually.
     */
    public free() {
        this.integrationParameters.free();
        this.islands.free();
        this.broadPhase.free();
        this.narrowPhase.free();
        this.bodies.free();
        this.colliders.free();
        this.impulseJoints.free();
        this.multibodyJoints.free();
        this.ccdSolver.free();
        this.physicsPipeline.free();
        this.serializationPipeline.free();
        this.debugRenderPipeline.free();
        this.characterControllers.forEach((controller) => controller.free());
        this.pidControllers.forEach((controller) => controller.free());

        // #if DIM3
        this.vehicleControllers.forEach((controller) => controller.free());
        // #endif

        this.integrationParameters = undefined;
        this.islands = undefined;
        this.broadPhase = undefined;
        this.narrowPhase = undefined;
        this.bodies = undefined;
        this.colliders = undefined;
        this.ccdSolver = undefined;
        this.impulseJoints = undefined;
        this.multibodyJoints = undefined;
        this.physicsPipeline = undefined;
        this.serializationPipeline = undefined;
        this.debugRenderPipeline = undefined;
        this.characterControllers = undefined;
        this.pidControllers = undefined;

        // #if DIM3
        this.vehicleControllers = undefined;
        // #endif
    }

    constructor(
        gravity: Vector,
        rawIntegrationParameters?: RawIntegrationParameters,
        rawIslands?: RawIslandManager,
        rawBroadPhase?: RawBroadPhase,
        rawNarrowPhase?: RawNarrowPhase,
        rawBodies?: RawRigidBodySet,
        rawColliders?: RawColliderSet,
        rawImpulseJoints?: RawImpulseJointSet,
        rawMultibodyJoints?: RawMultibodyJointSet,
        rawCCDSolver?: RawCCDSolver,
        rawPhysicsPipeline?: RawPhysicsPipeline,
        rawSerializationPipeline?: RawSerializationPipeline,
        rawDebugRenderPipeline?: RawDebugRenderPipeline,
    ) {
        this.gravity = gravity;
        this.integrationParameters = new IntegrationParameters(
            rawIntegrationParameters,
        );
        this.islands = new IslandManager(rawIslands);
        this.broadPhase = new BroadPhase(rawBroadPhase);
        this.narrowPhase = new NarrowPhase(rawNarrowPhase);
        this.bodies = new RigidBodySet(rawBodies);
        this.colliders = new ColliderSet(rawColliders);
        this.impulseJoints = new ImpulseJointSet(rawImpulseJoints);
        this.multibodyJoints = new MultibodyJointSet(rawMultibodyJoints);
        this.ccdSolver = new CCDSolver(rawCCDSolver);
        this.physicsPipeline = new PhysicsPipeline(rawPhysicsPipeline);
        this.serializationPipeline = new SerializationPipeline(
            rawSerializationPipeline,
        );
        this.debugRenderPipeline = new DebugRenderPipeline(
            rawDebugRenderPipeline,
        );
        this.characterControllers = new Set<KinematicCharacterController>();
        this.pidControllers = new Set<PidController>();

        // #if DIM3
        this.vehicleControllers = new Set<DynamicRayCastVehicleController>();
        // #endif

        this.impulseJoints.finalizeDeserialization(this.bodies);
        this.bodies.finalizeDeserialization(this.colliders);
        this.colliders.finalizeDeserialization(this.bodies);
    }

    public static fromRaw(raw: RawDeserializedWorld): World {
        if (!raw) return null;

        return new World(
            VectorOps.fromRaw(raw.takeGravity()),
            raw.takeIntegrationParameters(),
            raw.takeIslandManager(),
            raw.takeBroadPhase(),
            raw.takeNarrowPhase(),
            raw.takeBodies(),
            raw.takeColliders(),
            raw.takeImpulseJoints(),
            raw.takeMultibodyJoints(),
        );
    }

    /**
     * Takes a snapshot of this world.
     *
     * Use `World.restoreSnapshot` to create a new physics world with a state identical to
     * the state when `.takeSnapshot()` is called.
     */
    public takeSnapshot(): Uint8Array {
        return this.serializationPipeline.serializeAll(
            this.gravity,
            this.integrationParameters,
            this.islands,
            this.broadPhase,
            this.narrowPhase,
            this.bodies,
            this.colliders,
            this.impulseJoints,
            this.multibodyJoints,
        );
    }

    /**
     * Creates a new physics world from a snapshot.
     *
     * This new physics world will be an identical copy of the snapshoted physics world.
     */
    public static restoreSnapshot(data: Uint8Array): World {
        let deser = new SerializationPipeline();
        return deser.deserializeAll(data);
    }

    /**
     * Computes all the lines (and their colors) needed to render the scene.
     *
     * @param filterFlags - Flags for excluding whole subsets of colliders from rendering.
     * @param filterPredicate - Any collider for which this closure returns `false` will be excluded from the
     *                          debug rendering.
     */
    public debugRender(
        filterFlags?: QueryFilterFlags,
        filterPredicate?: (collider: Collider) => boolean,
    ): DebugRenderBuffers {
        this.debugRenderPipeline.render(
            this.bodies,
            this.colliders,
            this.impulseJoints,
            this.multibodyJoints,
            this.narrowPhase,
            filterFlags,
            filterPredicate,
        );
        return new DebugRenderBuffers(
            this.debugRenderPipeline.vertices,
            this.debugRenderPipeline.colors,
        );
    }

    /**
     * Advance the simulation by one time step.
     *
     * All events generated by the physics engine are ignored.
     *
     * @param EventQueue - (optional) structure responsible for collecting
     *   events generated by the physics engine.
     */
    public step(eventQueue?: EventQueue, hooks?: PhysicsHooks) {
        this.physicsPipeline.step(
            this.gravity,
            this.integrationParameters,
            this.islands,
            this.broadPhase,
            this.narrowPhase,
            this.bodies,
            this.colliders,
            this.impulseJoints,
            this.multibodyJoints,
            this.ccdSolver,
            eventQueue,
            hooks,
        );
    }

    /**
     * Update colliders positions after rigid-bodies moved.
     *
     * When a rigid-body moves, the positions of the colliders attached to it need to be updated. This update is
     * generally automatically done at the beginning and the end of each simulation step with World.step.
     * If the positions need to be updated without running a simulation step this method can be called manually.
     */
    public propagateModifiedBodyPositionsToColliders() {
        this.bodies.raw.propagateModifiedBodyPositionsToColliders(
            this.colliders.raw,
        );
    }

    // TODO: This needs to trigger a broad-phase update but without emitting collision events?
    // /**
    //  * Ensure subsequent scene queries take into account the collider positions set before this method is called.
    //  *
    //  * This does not step the physics simulation forward.
    //  */
    // public updateSceneQueries() {
    //     this.propagateModifiedBodyPositionsToColliders();
    //     this.queryPipeline.update(this.colliders);
    // }

    /**
     * The current simulation timestep.
     */
    get timestep(): number {
        return this.integrationParameters.dt;
    }

    /**
     * Sets the new simulation timestep.
     *
     * The simulation timestep governs by how much the physics state of the world will
     * be integrated. A simulation timestep should:
     * - be as small as possible. Typical values evolve around 0.016 (assuming the chosen unit is milliseconds,
     * corresponds to the time between two frames of a game running at 60FPS).
     * - not vary too much during the course of the simulation. A timestep with large variations may
     * cause instabilities in the simulation.
     *
     * @param dt - The timestep length, in seconds.
     */
    set timestep(dt: number) {
        this.integrationParameters.dt = dt;
    }

    /**
     * The approximate size of most dynamic objects in the scene.
     *
     * See the documentation of the `World.lengthUnit` setter for further details.
     */
    get lengthUnit(): number {
        return this.integrationParameters.lengthUnit;
    }

    /**
     * The approximate size of most dynamic objects in the scene.
     *
     * This value is used internally to estimate some length-based tolerance. In particular, the
     * values `IntegrationParameters.allowedLinearError`,
     * `IntegrationParameters.maxPenetrationCorrection`,
     * `IntegrationParameters.predictionDistance`, `RigidBodyActivation.linearThreshold`
     * are scaled by this value implicitly.
     *
     * This value can be understood as the number of units-per-meter in your physical world compared
     * to a human-sized world in meter. For example, in a 2d game, if your typical object size is 100
     * pixels, set the `[`Self::length_unit`]` parameter to 100.0. The physics engine will interpret
     * it as if 100 pixels is equivalent to 1 meter in its various internal threshold.
     * (default `1.0`).
     */
    set lengthUnit(unitsPerMeter: number) {
        this.integrationParameters.lengthUnit = unitsPerMeter;
    }

    /**
     * The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     */
    get numSolverIterations(): number {
        return this.integrationParameters.numSolverIterations;
    }

    /**
     * Sets the number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     *
     * The greater this value is, the most rigid and realistic the physics simulation will be.
     * However a greater number of iterations is more computationally intensive.
     *
     * @param niter - The new number of solver iterations.
     */
    set numSolverIterations(niter: number) {
        this.integrationParameters.numSolverIterations = niter;
    }

    /**
     * Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     */
    get numInternalPgsIterations(): number {
        return this.integrationParameters.numInternalPgsIterations;
    }

    /**
     * Sets the Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     *
     * Increasing this parameter will improve stability of the simulation. It will have a lesser effect than
     * increasing `numSolverIterations` but is also less computationally expensive.
     *
     * @param niter - The new number of internal PGS iterations.
     */
    set numInternalPgsIterations(niter: number) {
        this.integrationParameters.numInternalPgsIterations = niter;
    }

    /**
     * The number of substeps continuous collision-detection can run (default: `1`).
     */
    get maxCcdSubsteps(): number {
        return this.integrationParameters.maxCcdSubsteps;
    }

    /**
     * Sets the number of substeps continuous collision-detection can run (default: `1`).
     *
     * CCD operates using a "motion clamping" mechanism where all fast-moving object trajectories will
     * be truncated to their first impact on their path. The number of CCD substeps beyond 1 indicate how
     * many times that trajectory will be updated and continued after a hit. This can results in smoother
     * paths, but at a significant computational cost.
     *
     * @param niter - The new maximum number of CCD substeps. Setting to `0` disables CCD entirely.
     */
    set maxCcdSubsteps(substeps: number) {
        this.integrationParameters.maxCcdSubsteps = substeps;
    }

    /**
     * Creates a new rigid-body from the given rigid-body descriptor.
     *
     * @param body - The description of the rigid-body to create.
     */
    public createRigidBody(body: RigidBodyDesc): RigidBody {
        return this.bodies.createRigidBody(this.colliders, body);
    }

    /**
     * Creates a new character controller.
     *
     * @param offset - The artificial gap added between the character’s chape and its environment.
     */
    public createCharacterController(
        offset: number,
    ): KinematicCharacterController {
        let controller = new KinematicCharacterController(
            offset,
            this.integrationParameters,
            this.broadPhase,
            this.narrowPhase,
            this.bodies,
            this.colliders,
        );
        this.characterControllers.add(controller);
        return controller;
    }

    /**
     * Removes a character controller from this world.
     *
     * @param controller - The character controller to remove.
     */
    public removeCharacterController(controller: KinematicCharacterController) {
        this.characterControllers.delete(controller);
        controller.free();
    }

    /**
     * Creates a new PID (Proportional-Integral-Derivative) controller.
     *
     * @param kp - The Proportional gain applied to the instantaneous linear position errors.
     *             This is usually set to a multiple of the inverse of simulation step time
     *             (e.g. `60` if the delta-time is `1.0 / 60.0`).
     * @param ki - The linear gain applied to the Integral part of the PID controller.
     * @param kd - The Derivative gain applied to the instantaneous linear velocity errors.
     *             This is usually set to a value in `[0.0, 1.0]` where `0.0` implies no damping
     *             (no correction of velocity errors) and `1.0` implies complete damping (velocity errors
     *             are corrected in a single simulation step).
     * @param axes - The axes affected by this controller.
     *               Only coordinate axes with a bit flags set to `true` will be taken into
     *               account when calculating the errors and corrections.
     */
    public createPidController(
        kp: number,
        ki: number,
        kd: number,
        axes: PidAxesMask,
    ): PidController {
        let controller = new PidController(
            this.integrationParameters,
            this.bodies,
            kp,
            ki,
            kd,
            axes,
        );
        this.pidControllers.add(controller);
        return controller;
    }

    /**
     * Removes a PID controller from this world.
     *
     * @param controller - The PID controller to remove.
     */
    public removePidController(controller: PidController) {
        this.pidControllers.delete(controller);
        controller.free();
    }

    // #if DIM3
    /**
     * Creates a new vehicle controller.
     *
     * @param chassis - The rigid-body used as the chassis of the vehicle controller. When the vehicle
     *                  controller is updated, it will change directly the rigid-body’s velocity. This
     *                  rigid-body must be a dynamic or kinematic-velocity-based rigid-body.
     */
    public createVehicleController(
        chassis: RigidBody,
    ): DynamicRayCastVehicleController {
        let controller = new DynamicRayCastVehicleController(
            chassis,
            this.broadPhase,
            this.narrowPhase,
            this.bodies,
            this.colliders,
        );
        this.vehicleControllers.add(controller);
        return controller;
    }

    /**
     * Removes a vehicle controller from this world.
     *
     * @param controller - The vehicle controller to remove.
     */
    public removeVehicleController(
        controller: DynamicRayCastVehicleController,
    ) {
        this.vehicleControllers.delete(controller);
        controller.free();
    }

    // #endif

    /**
     * Creates a new collider.
     *
     * @param desc - The description of the collider.
     * @param parent - The rigid-body this collider is attached to.
     */
    public createCollider(desc: ColliderDesc, parent?: RigidBody): Collider {
        let parentHandle = parent ? parent.handle : undefined;
        return this.colliders.createCollider(this.bodies, desc, parentHandle);
    }

    /**
     * Creates a new impulse joint from the given joint descriptor.
     *
     * @param params - The description of the joint to create.
     * @param parent1 - The first rigid-body attached to this joint.
     * @param parent2 - The second rigid-body attached to this joint.
     * @param wakeUp - Should the attached rigid-bodies be awakened?
     */
    public createImpulseJoint(
        params: JointData,
        parent1: RigidBody,
        parent2: RigidBody,
        wakeUp: boolean,
    ): ImpulseJoint {
        return this.impulseJoints.createJoint(
            this.bodies,
            params,
            parent1.handle,
            parent2.handle,
            wakeUp,
        );
    }

    /**
     * Creates a new multibody joint from the given joint descriptor.
     *
     * @param params - The description of the joint to create.
     * @param parent1 - The first rigid-body attached to this joint.
     * @param parent2 - The second rigid-body attached to this joint.
     * @param wakeUp - Should the attached rigid-bodies be awakened?
     */
    public createMultibodyJoint(
        params: JointData,
        parent1: RigidBody,
        parent2: RigidBody,
        wakeUp: boolean,
    ): MultibodyJoint {
        return this.multibodyJoints.createJoint(
            params,
            parent1.handle,
            parent2.handle,
            wakeUp,
        );
    }

    /**
     * Retrieves a rigid-body from its handle.
     *
     * @param handle - The integer handle of the rigid-body to retrieve.
     */
    public getRigidBody(handle: RigidBodyHandle): RigidBody {
        return this.bodies.get(handle);
    }

    /**
     * Retrieves a collider from its handle.
     *
     * @param handle - The integer handle of the collider to retrieve.
     */
    public getCollider(handle: ColliderHandle): Collider {
        return this.colliders.get(handle);
    }

    /**
     * Retrieves an impulse joint from its handle.
     *
     * @param handle - The integer handle of the impulse joint to retrieve.
     */
    public getImpulseJoint(handle: ImpulseJointHandle): ImpulseJoint {
        return this.impulseJoints.get(handle);
    }

    /**
     * Retrieves an multibody joint from its handle.
     *
     * @param handle - The integer handle of the multibody joint to retrieve.
     */
    public getMultibodyJoint(handle: MultibodyJointHandle): MultibodyJoint {
        return this.multibodyJoints.get(handle);
    }

    /**
     * Removes the given rigid-body from this physics world.
     *
     * This will remove this rigid-body as well as all its attached colliders and joints.
     * Every other bodies touching or attached by joints to this rigid-body will be woken-up.
     *
     * @param body - The rigid-body to remove.
     */
    public removeRigidBody(body: RigidBody) {
        if (this.bodies) {
            this.bodies.remove(
                body.handle,
                this.islands,
                this.colliders,
                this.impulseJoints,
                this.multibodyJoints,
            );
        }
    }

    /**
     * Removes the given collider from this physics world.
     *
     * @param collider - The collider to remove.
     * @param wakeUp - If set to `true`, the rigid-body this collider is attached to will be awaken.
     */
    public removeCollider(collider: Collider, wakeUp: boolean) {
        if (this.colliders) {
            this.colliders.remove(
                collider.handle,
                this.islands,
                this.bodies,
                wakeUp,
            );
        }
    }

    /**
     * Removes the given impulse joint from this physics world.
     *
     * @param joint - The impulse joint to remove.
     * @param wakeUp - If set to `true`, the rigid-bodies attached by this joint will be awaken.
     */
    public removeImpulseJoint(joint: ImpulseJoint, wakeUp: boolean) {
        if (this.impulseJoints) {
            this.impulseJoints.remove(joint.handle, wakeUp);
        }
    }

    /**
     * Removes the given multibody joint from this physics world.
     *
     * @param joint - The multibody joint to remove.
     * @param wakeUp - If set to `true`, the rigid-bodies attached by this joint will be awaken.
     */
    public removeMultibodyJoint(joint: MultibodyJoint, wakeUp: boolean) {
        if (this.impulseJoints) {
            this.multibodyJoints.remove(joint.handle, wakeUp);
        }
    }

    /**
     * Applies the given closure to each collider managed by this physics world.
     *
     * @param f(collider) - The function to apply to each collider managed by this physics world. Called as `f(collider)`.
     */
    public forEachCollider(f: (collider: Collider) => void) {
        this.colliders.forEach(f);
    }

    /**
     * Applies the given closure to each rigid-body managed by this physics world.
     *
     * @param f(body) - The function to apply to each rigid-body managed by this physics world. Called as `f(collider)`.
     */
    public forEachRigidBody(f: (body: RigidBody) => void) {
        this.bodies.forEach(f);
    }

    /**
     * Applies the given closure to each active rigid-body managed by this physics world.
     *
     * After a short time of inactivity, a rigid-body is automatically deactivated ("asleep") by
     * the physics engine in order to save computational power. A sleeping rigid-body never moves
     * unless it is moved manually by the user.
     *
     * @param f - The function to apply to each active rigid-body managed by this physics world. Called as `f(collider)`.
     */
    public forEachActiveRigidBody(f: (body: RigidBody) => void) {
        this.bodies.forEachActiveRigidBody(this.islands, f);
    }

    /**
     * Find the closest intersection between a ray and the physics world.
     *
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param filter - The callback to filter out which collider will be hit.
     */
    public castRay(
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): RayColliderHit | null {
        return this.broadPhase.castRay(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            ray,
            maxToi,
            solid,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Find the closest intersection between a ray and the physics world.
     *
     * This also computes the normal at the hit point.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     */
    public castRayAndGetNormal(
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): RayColliderIntersection | null {
        return this.broadPhase.castRayAndGetNormal(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            ray,
            maxToi,
            solid,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Cast a ray and collects all the intersections between a ray and the scene.
     *
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param callback - The callback called once per hit (in no particular order) between a ray and a collider.
     *   If this callback returns `false`, then the cast will stop and no further hits will be detected/reported.
     */
    public intersectionsWithRay(
        ray: Ray,
        maxToi: number,
        solid: boolean,
        callback: (intersect: RayColliderIntersection) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.broadPhase.intersectionsWithRay(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            ray,
            maxToi,
            solid,
            callback,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Gets the handle of up to one collider intersecting the given shape.
     *
     * @param shapePos - The position of the shape used for the intersection test.
     * @param shapeRot - The orientation of the shape used for the intersection test.
     * @param shape - The shape used for the intersection test.
     * @param groups - The bit groups and filter associated to the ray, in order to only
     *   hit the colliders with collision groups compatible with the ray's group.
     */
    public intersectionWithShape(
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): Collider | null {
        let handle = this.broadPhase.intersectionWithShape(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            shapePos,
            shapeRot,
            shape,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
        return handle != null ? this.colliders.get(handle) : null;
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param point - The point to project.
     * @param solid - If this is set to `true` then the collider shapes are considered to
     *   be plain (if the point is located inside of a plain shape, its projection is the point
     *   itself). If it is set to `false` the collider shapes are considered to be hollow
     *   (if the point is located inside of an hollow shape, it is projected on the shape's
     *   boundary).
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     */
    public projectPoint(
        point: Vector,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): PointColliderProjection | null {
        return this.broadPhase.projectPoint(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            point,
            solid,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param point - The point to project.
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     */
    public projectPointAndGetFeature(
        point: Vector,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): PointColliderProjection | null {
        return this.broadPhase.projectPointAndGetFeature(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            point,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Find all the colliders containing the given point.
     *
     * @param point - The point used for the containment test.
     * @param groups - The bit groups and filter associated to the point to test, in order to only
     *   test on colliders with collision groups compatible with the ray's group.
     * @param callback - A function called with the handles of each collider with a shape
     *   containing the `point`.
     */
    public intersectionsWithPoint(
        point: Vector,
        callback: (handle: Collider) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.broadPhase.intersectionsWithPoint(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            point,
            this.colliders.castClosure(callback),
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Casts a shape at a constant linear velocity and retrieve the first collider it hits.
     * This is similar to ray-casting except that we are casting a whole shape instead of
     * just a point (the ray origin).
     *
     * @param shapePos - The initial position of the shape to cast.
     * @param shapeRot - The initial rotation of the shape to cast.
     * @param shapeVel - The constant velocity of the shape to cast (i.e. the cast direction).
     * @param shape - The shape to cast.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                         will be returned.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param groups - The bit groups and filter associated to the shape to cast, in order to only
     *   test on colliders with collision groups compatible with this group.
     */
    public castShape(
        shapePos: Vector,
        shapeRot: Rotation,
        shapeVel: Vector,
        shape: Shape,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ): ColliderShapeCastHit | null {
        return this.broadPhase.castShape(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            shapePos,
            shapeRot,
            shapeVel,
            shape,
            targetDistance,
            maxToi,
            stopAtPenetration,
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Retrieve all the colliders intersecting the given shape.
     *
     * @param shapePos - The position of the shape to test.
     * @param shapeRot - The orientation of the shape to test.
     * @param shape - The shape to test.
     * @param groups - The bit groups and filter associated to the shape to test, in order to only
     *   test on colliders with collision groups compatible with this group.
     * @param callback - A function called with the handles of each collider intersecting the `shape`.
     */
    public intersectionsWithShape(
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        callback: (collider: Collider) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: Collider,
        filterExcludeRigidBody?: RigidBody,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.broadPhase.intersectionsWithShape(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            shapePos,
            shapeRot,
            shape,
            this.colliders.castClosure(callback),
            filterFlags,
            filterGroups,
            filterExcludeCollider ? filterExcludeCollider.handle : null,
            filterExcludeRigidBody ? filterExcludeRigidBody.handle : null,
            this.colliders.castClosure(filterPredicate),
        );
    }

    /**
     * Finds the handles of all the colliders with an AABB intersecting the given AABB.
     *
     * @param aabbCenter - The center of the AABB to test.
     * @param aabbHalfExtents - The half-extents of the AABB to test.
     * @param callback - The callback that will be called with the handles of all the colliders
     *                   currently intersecting the given AABB.
     */
    public collidersWithAabbIntersectingAabb(
        aabbCenter: Vector,
        aabbHalfExtents: Vector,
        callback: (handle: Collider) => boolean,
    ) {
        this.broadPhase.collidersWithAabbIntersectingAabb(
            this.narrowPhase,
            this.bodies,
            this.colliders,
            aabbCenter,
            aabbHalfExtents,
            this.colliders.castClosure(callback),
        );
    }

    /**
     * Enumerates all the colliders potentially in contact with the given collider.
     *
     * @param collider1 - The second collider involved in the contact.
     * @param f - Closure that will be called on each collider that is in contact with `collider1`.
     */
    public contactPairsWith(
        collider1: Collider,
        f: (collider2: Collider) => void,
    ) {
        this.narrowPhase.contactPairsWith(
            collider1.handle,
            this.colliders.castClosure(f),
        );
    }

    /**
     * Enumerates all the colliders intersecting the given colliders, assuming one of them
     * is a sensor.
     */
    public intersectionPairsWith(
        collider1: Collider,
        f: (collider2: Collider) => void,
    ) {
        this.narrowPhase.intersectionPairsWith(
            collider1.handle,
            this.colliders.castClosure(f),
        );
    }

    /**
     * Iterates through all the contact manifolds between the given pair of colliders.
     *
     * @param collider1 - The first collider involved in the contact.
     * @param collider2 - The second collider involved in the contact.
     * @param f - Closure that will be called on each contact manifold between the two colliders. If the second argument
     *            passed to this closure is `true`, then the contact manifold data is flipped, i.e., methods like `localNormal1`
     *            actually apply to the `collider2` and fields like `localNormal2` apply to the `collider1`.
     */
    public contactPair(
        collider1: Collider,
        collider2: Collider,
        f: (manifold: TempContactManifold, flipped: boolean) => void,
    ) {
        this.narrowPhase.contactPair(collider1.handle, collider2.handle, f);
    }

    /**
     * Returns `true` if `collider1` and `collider2` intersect and at least one of them is a sensor.
     * @param collider1 − The first collider involved in the intersection.
     * @param collider2 − The second collider involved in the intersection.
     */
    public intersectionPair(collider1: Collider, collider2: Collider): boolean {
        return this.narrowPhase.intersectionPair(
            collider1.handle,
            collider2.handle,
        );
    }

    /**
     * Sets whether internal performance profiling is enabled (default: false).
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    set profilerEnabled(enabled: boolean) {
        this.physicsPipeline.raw.set_profiler_enabled(enabled);
    }

    /**
     * Indicates if the internal performance profiling is enabled.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    get profilerEnabled(): boolean {
        return this.physicsPipeline.raw.is_profiler_enabled();
    }

    /**
     * The time spent in milliseconds by the last step to run the entire simulation step.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingStep(): number {
        return this.physicsPipeline.raw.timing_step();
    }

    /**
     * The time spent in milliseconds by the last step to run the collision-detection
     * (broad-phase + narrow-phase).
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCollisionDetection(): number {
        return this.physicsPipeline.raw.timing_collision_detection();
    }

    /**
     * The time spent in milliseconds by the last step to run the broad-phase.
     *
     * This timing is included in `timingCollisionDetection`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingBroadPhase(): number {
        return this.physicsPipeline.raw.timing_broad_phase();
    }

    /**
     * The time spent in milliseconds by the last step to run the narrow-phase.
     *
     * This timing is included in `timingCollisionDetection`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingNarrowPhase(): number {
        return this.physicsPipeline.raw.timing_narrow_phase();
    }

    /**
     * The time spent in milliseconds by the last step to run the constraint solver.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingSolver(): number {
        return this.physicsPipeline.raw.timing_solver();
    }

    /**
     * The time spent in milliseconds by the last step to run the constraint
     * initialization.
     *
     * This timing is included in `timingSolver`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingVelocityAssembly(): number {
        return this.physicsPipeline.raw.timing_velocity_assembly();
    }

    /**
     * The time spent in milliseconds by the last step to run the constraint
     * resolution.
     *
     * This timing is included in `timingSolver`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingVelocityResolution(): number {
        return this.physicsPipeline.raw.timing_velocity_resolution();
    }

    /**
     * The time spent in milliseconds by the last step to run the rigid-body
     * velocity update.
     *
     * This timing is included in `timingSolver`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingVelocityUpdate(): number {
        return this.physicsPipeline.raw.timing_velocity_update();
    }

    /**
     * The time spent in milliseconds by writing rigid-body velocities
     * calculated by the solver back into the rigid-bodies.
     *
     * This timing is included in `timingSolver`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingVelocityWriteback(): number {
        return this.physicsPipeline.raw.timing_velocity_writeback();
    }

    /**
     * The total time spent in CCD detection and resolution.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCcd(): number {
        return this.physicsPipeline.raw.timing_ccd();
    }

    /**
     * The total time spent searching for the continuous hits during CCD.
     *
     * This timing is included in `timingCcd`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCcdToiComputation(): number {
        return this.physicsPipeline.raw.timing_ccd_toi_computation();
    }

    /**
     * The total time spent in the broad-phase during CCD.
     *
     * This timing is included in `timingCcd`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCcdBroadPhase(): number {
        return this.physicsPipeline.raw.timing_ccd_broad_phase();
    }

    /**
     * The total time spent in the narrow-phase during CCD.
     *
     * This timing is included in `timingCcd`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCcdNarrowPhase(): number {
        return this.physicsPipeline.raw.timing_ccd_narrow_phase();
    }

    /**
     * The total time spent in the constraints resolution during CCD.
     *
     * This timing is included in `timingCcd`.
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingCcdSolver(): number {
        return this.physicsPipeline.raw.timing_ccd_solver();
    }

    /**
     * The total time spent in the islands calculation during CCD.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingIslandConstruction(): number {
        return this.physicsPipeline.raw.timing_island_construction();
    }

    /**
     * The total time spent propagating detected user changes.
     *
     * Only works if the internal profiler is enabled with `World.profilerEnabled = true`.
     */
    public timingUserChanges(): number {
        return this.physicsPipeline.raw.timing_user_changes();
    }
}
