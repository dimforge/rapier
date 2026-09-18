import {
    RawSoftBodyBuilder,
    RawSoftBodyMaterial,
    RawSoftBodySet,
    RawSoftBodyTearEvent,
    RawSoftBodyCellModel,
    RawSoftEdgePlasticFlow,
    RawSoftMeshBindingMode,
} from "../raw";
import {Rotation, RotationOps, Vector, VectorOps, scratchBuffer} from "../math";
import {RigidBody, RigidBodyHandle} from "./rigid_body";
import {RigidBodySet} from "./rigid_body_set";
import {Collider, ColliderDesc, ColliderSet} from "../geometry";

/**
 * The integer identifier of a soft body added to a `SoftBodySet`.
 */
export type SoftBodyHandle = number;

/**
 * The constitutive model of a soft body's cells (triangles in 2D, tetrahedra in 3D).
 */
export enum SoftBodyCellModel {
    /**
     * Per-cell area/volume preservation constraints (the cell's shape is held by its edges).
     */
    Volume = 0,
    /**
     * Corotational linear elasticity: a linear material in the cell's rotated frame.
     */
    Corotational = 1,
    /**
     * Neo-Hookean hyperelasticity, which resists inversion and large compressions.
     */
    NeoHookean = 2,
}

/**
 * Which strains make a soft body's edge rest lengths flow plastically.
 */
export enum SoftEdgePlasticFlow {
    Both = 0,
    Compression = 1,
    Tension = 2,
}

/**
 * How the vertices of a deformable collider follow the particles of its cluster.
 */
export enum SoftMeshBindingMode {
    /**
     * Vertex `i` of the mesh follows the `i`-th particle of the binding's particle list.
     */
    Direct = 0,
    /**
     * Every vertex follows the particle of the cluster closest to it.
     */
    DirectByPosition = 1,
    /**
     * Every vertex is embedded in the cell of the cluster holding it (cage simulation).
     */
    Skinned = 2,
}

/**
 * Describes how a deformable collider is bound to the cluster of a soft body.
 */
export class SoftMeshBinding {
    mode: SoftMeshBindingMode;
    /**
     * For `SoftMeshBindingMode.Direct`: the particle followed by each vertex of the mesh.
     */
    particles: Uint32Array;
    /**
     * For `SoftMeshBindingMode.DirectByPosition`: the largest distance between a vertex and
     * the particle it follows.
     */
    eps: number;
    /**
     * Whether the mesh collides with itself.
     */
    selfContacts: boolean;

    constructor(mode: SoftMeshBindingMode) {
        this.mode = mode;
        this.particles = new Uint32Array(0);
        this.eps = 0.0;
        this.selfContacts = false;
    }

    /**
     * Binds vertex `i` of the mesh to `particles[i]`, which must belong to the cluster.
     */
    public static direct(particles: Uint32Array | number[]): SoftMeshBinding {
        let res = new SoftMeshBinding(SoftMeshBindingMode.Direct);
        res.particles = Uint32Array.from(particles);
        return res;
    }

    /**
     * Binds every vertex to the particle of the cluster closest to it, within `eps`.
     */
    public static directByPosition(eps: number): SoftMeshBinding {
        let res = new SoftMeshBinding(SoftMeshBindingMode.DirectByPosition);
        res.eps = eps;
        return res;
    }

    /**
     * Binds every vertex to the cell of the cluster holding it (cage simulation).
     */
    public static skinned(): SoftMeshBinding {
        return new SoftMeshBinding(SoftMeshBindingMode.Skinned);
    }

    /**
     * Enables collisions of this mesh with itself.
     */
    public setSelfContacts(enabled: boolean): SoftMeshBinding {
        this.selfContacts = enabled;
        return this;
    }

    /** @internal */
    public rawMode(): RawSoftMeshBindingMode {
        return this.mode as number as RawSoftMeshBindingMode;
    }
}

/**
 * Spring coefficients of an elastic constraint: a natural frequency (Hz) and a damping ratio.
 */
export interface SpringCoefficients {
    naturalFrequency: number;
    dampingRatio: number;
}

/**
 * The material of a soft body: the softness of its constraints, its elasticity, plasticity
 * and tearing thresholds.
 *
 * A material is a plain JavaScript object; pass it to `SoftBodyDesc.setMaterial` or
 * `SoftBody.setMaterial` to apply it.
 */
export class SoftBodyMaterial {
    /**
     * Softness of the structural edges.
     */
    edgeSoftness: SpringCoefficients;
    /**
     * Softness of the bending constraints (bend edges and dihedrals).
     */
    bendSoftness: SpringCoefficients;
    /**
     * Softness of the area/volume preservation constraints.
     */
    volumeSoftness: SpringCoefficients;
    /**
     * Softness of the shape-matching constraints.
     */
    shapeMatchingSoftness: SpringCoefficients;
    /**
     * Young's modulus of the elastic cells (Corotational and NeoHookean cell models).
     */
    youngModulus: number;
    /**
     * Poisson's ratio of the elastic cells.
     */
    poissonRatio: number;
    /**
     * Damping ratio of the elastic cells.
     */
    elasticDampingRatio: number;
    /**
     * Strain beyond which the rest shape of an elastic cell flows (`0` disables plasticity).
     */
    plasticYield: number;
    /**
     * Rate (per second) at which an elastic cell's rest shape follows its deformation past the
     * yield.
     */
    plasticCreep: number;
    /**
     * Largest accumulated plastic deformation of an elastic cell.
     */
    plasticMax: number;
    /**
     * Damping of the deformation velocity of the elastic cells.
     */
    deformationDamping: number;
    /**
     * Strain beyond which the rest length of an edge flows (`0` disables edge plasticity).
     */
    edgePlasticYield: number;
    /**
     * Rate (per second) at which an edge's rest length follows its stretch past the yield.
     */
    edgePlasticCreep: number;
    /**
     * Largest relative change of an edge's rest length.
     */
    edgePlasticMax: number;
    /**
     * Which strains make the edge rest lengths flow.
     */
    edgePlasticFlow: SoftEdgePlasticFlow;
    /**
     * Strain beyond which an element tears, or `null` for no strain-based tearing.
     */
    tearStrain: number | null;
    /**
     * Force beyond which an element tears, or `null` for no force-based tearing.
     */
    tearForce: number | null;
    /**
     * Time constant (seconds) of the smoothing applied to the load before comparing it to
     * `tearForce`.
     */
    tearSmoothing: number;
    /**
     * Multiplier of the tear thresholds for the interior elements (relative to the surface).
     */
    interiorStrength: number;
    /**
     * Largest number of elements torn per step.
     */
    maxTearsPerStep: number;
    /**
     * Smallest piece (in elements) a tear may split off, or `null` for the default.
     */
    minPiece: number | null;

    constructor() {
        SoftBodyMaterial.copyFromRaw(this, new RawSoftBodyMaterial(), true);
    }

    /**
     * A material whose edge, bend, volume and shape-matching softness all take the same value.
     */
    public static uniform(
        naturalFrequency: number,
        dampingRatio: number,
    ): SoftBodyMaterial {
        let res = new SoftBodyMaterial();
        let raw = RawSoftBodyMaterial.uniform(naturalFrequency, dampingRatio);
        SoftBodyMaterial.copyFromRaw(res, raw, true);
        return res;
    }

    /** @internal */
    public static fromRaw(raw: RawSoftBodyMaterial): SoftBodyMaterial {
        let res = new SoftBodyMaterial();
        SoftBodyMaterial.copyFromRaw(res, raw, true);
        return res;
    }

    /** @internal */
    private static copyFromRaw(
        target: SoftBodyMaterial,
        raw: RawSoftBodyMaterial,
        freeRaw: boolean,
    ) {
        target.edgeSoftness = {
            naturalFrequency: raw.edgeFrequency,
            dampingRatio: raw.edgeDampingRatio,
        };
        target.bendSoftness = {
            naturalFrequency: raw.bendFrequency,
            dampingRatio: raw.bendDampingRatio,
        };
        target.volumeSoftness = {
            naturalFrequency: raw.volumeFrequency,
            dampingRatio: raw.volumeDampingRatio,
        };
        target.shapeMatchingSoftness = {
            naturalFrequency: raw.shapeMatchingFrequency,
            dampingRatio: raw.shapeMatchingDampingRatio,
        };
        target.youngModulus = raw.youngModulus;
        target.poissonRatio = raw.poissonRatio;
        target.elasticDampingRatio = raw.elasticDampingRatio;
        target.plasticYield = raw.plasticYield;
        target.plasticCreep = raw.plasticCreep;
        target.plasticMax = raw.plasticMax;
        target.deformationDamping = raw.deformationDamping;
        target.edgePlasticYield = raw.edgePlasticYield;
        target.edgePlasticCreep = raw.edgePlasticCreep;
        target.edgePlasticMax = raw.edgePlasticMax;
        target.edgePlasticFlow =
            raw.edgePlasticFlow as number as SoftEdgePlasticFlow;
        target.tearStrain = raw.tearStrain ?? null;
        target.tearForce = raw.tearForce ?? null;
        target.tearSmoothing = raw.tearSmoothing;
        target.interiorStrength = raw.interiorStrength;
        target.maxTearsPerStep = raw.maxTearsPerStep;
        target.minPiece = raw.minPiece ?? null;
        if (freeRaw) {
            raw.free();
        }
    }

    /** @internal */
    public intoRaw(): RawSoftBodyMaterial {
        let raw = new RawSoftBodyMaterial();
        raw.edgeFrequency = this.edgeSoftness.naturalFrequency;
        raw.edgeDampingRatio = this.edgeSoftness.dampingRatio;
        raw.bendFrequency = this.bendSoftness.naturalFrequency;
        raw.bendDampingRatio = this.bendSoftness.dampingRatio;
        raw.volumeFrequency = this.volumeSoftness.naturalFrequency;
        raw.volumeDampingRatio = this.volumeSoftness.dampingRatio;
        raw.shapeMatchingFrequency =
            this.shapeMatchingSoftness.naturalFrequency;
        raw.shapeMatchingDampingRatio = this.shapeMatchingSoftness.dampingRatio;
        raw.youngModulus = this.youngModulus;
        raw.poissonRatio = this.poissonRatio;
        raw.elasticDampingRatio = this.elasticDampingRatio;
        raw.plasticYield = this.plasticYield;
        raw.plasticCreep = this.plasticCreep;
        raw.plasticMax = this.plasticMax;
        raw.deformationDamping = this.deformationDamping;
        raw.edgePlasticYield = this.edgePlasticYield;
        raw.edgePlasticCreep = this.edgePlasticCreep;
        raw.edgePlasticMax = this.edgePlasticMax;
        raw.edgePlasticFlow = this
            .edgePlasticFlow as number as RawSoftEdgePlasticFlow;
        raw.tearStrain = this.tearStrain ?? undefined;
        raw.tearForce = this.tearForce ?? undefined;
        raw.tearSmoothing = this.tearSmoothing;
        raw.interiorStrength = this.interiorStrength;
        raw.maxTearsPerStep = this.maxTearsPerStep;
        raw.minPiece = this.minPiece ?? undefined;
        return raw;
    }
}

/**
 * A soft body: particles linked by elastic constraints (edges, bending constraints and
 * cells), simulated together with the rigid bodies, contacts and joints of the world.
 *
 * A soft body lives in a `SoftBodySet` and is referenced by its integer `handle`. Its hidden
 * root rigid body (`rootBody()`) stands for the whole body in joints and islands; its clusters
 * add more rigid-body proxies that joints and colliders can attach to.
 */
export class SoftBody {
    private rawSet: RawSoftBodySet; // The SoftBody won't need to free this.
    private bodies: RigidBodySet;
    private colliders: ColliderSet;
    readonly handle: SoftBodyHandle;

    /**
     * An arbitrary user-defined object associated with this soft body.
     */
    public userData?: unknown;

    constructor(
        rawSet: RawSoftBodySet,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        handle: SoftBodyHandle,
    ) {
        this.rawSet = rawSet;
        this.bodies = bodies;
        this.colliders = colliders;
        this.handle = handle;
    }

    /** @internal */
    public finalizeDeserialization(
        bodies: RigidBodySet,
        colliders: ColliderSet,
    ) {
        this.bodies = bodies;
        this.colliders = colliders;
    }

    /**
     * Checks if this soft body is still valid (i.e. that it has not been deleted from the
     * soft-body set yet).
     */
    public isValid(): boolean {
        return this.rawSet.contains(this.handle);
    }

    /**
     * A counter incremented by every change of the body's topology (tears, cuts, splits).
     */
    public topologyVersion(): number {
        return this.rawSet.sbTopologyVersion(this.handle);
    }

    /*
     * Particles.
     */

    /**
     * The number of particles of this soft body.
     */
    public numParticles(): number {
        return this.rawSet.sbNumParticles(this.handle);
    }

    /**
     * The world-space position of the `i`-th particle.
     */
    public particlePosition(i: number, target?: Vector): Vector {
        this.rawSet.sbParticlePosition(this.handle, i, scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The world-space positions of every particle, flattened (two or three floats per
     * particle).
     */
    public particlePositions(): Float32Array {
        return this.rawSet.sbParticlePositions(this.handle);
    }

    /**
     * The velocity of the `i`-th particle.
     */
    public particleVelocity(i: number, target?: Vector): Vector {
        this.rawSet.sbParticleVelocity(this.handle, i, scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The velocities of every particle, flattened (two or three floats per particle).
     */
    public particleVelocities(): Float32Array {
        return this.rawSet.sbParticleVelocities(this.handle);
    }

    /**
     * The rest position of the `i`-th particle (the position its constraints hold it at).
     */
    public particleRestPosition(i: number, target?: Vector): Vector {
        this.rawSet.sbParticleRestPosition(this.handle, i, scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The mass of the `i`-th particle.
     */
    public particleMass(i: number): number {
        return this.rawSet.sbParticleMass(this.handle, i);
    }

    /**
     * Is the `i`-th particle pinned (held in place)?
     */
    public isParticlePinned(i: number): boolean {
        return this.rawSet.sbIsParticlePinned(this.handle, i);
    }

    /**
     * Does the `i`-th particle lie on the body's surface?
     */
    public isParticleOnSurface(i: number): boolean {
        return this.rawSet.sbIsParticleOnSurface(this.handle, i);
    }

    /**
     * Has an element of the `i`-th particle torn?
     */
    public isParticleDamaged(i: number): boolean {
        return this.rawSet.sbIsParticleDamaged(this.handle, i);
    }

    /**
     * Sets the world-space position of the `i`-th particle.
     */
    public setParticlePosition(i: number, position: Vector) {
        let rawPos = VectorOps.intoRaw(position);
        this.rawSet.sbSetParticlePosition(this.handle, i, rawPos);
        rawPos.free();
    }

    /**
     * Sets the velocity of the `i`-th particle.
     */
    public setParticleVelocity(i: number, velocity: Vector) {
        let rawVel = VectorOps.intoRaw(velocity);
        this.rawSet.sbSetParticleVelocity(this.handle, i, rawVel);
        rawVel.free();
    }

    /**
     * Moves the pinned `i`-th particle to `position` over the next step, like a
     * position-based kinematic body (it pushes what it meets), then holds it there.
     * Ignored for a free particle.
     */
    public setParticleKinematicTarget(i: number, position: Vector) {
        let rawPos = VectorOps.intoRaw(position);
        this.rawSet.sbSetParticleKinematicTarget(this.handle, i, rawPos);
        rawPos.free();
    }

    /**
     * Pins (or releases) the `i`-th particle.
     */
    public setParticlePinned(i: number, pinned: boolean) {
        this.rawSet.sbSetParticlePinned(this.handle, i, pinned);
    }

    /**
     * Attaches the `i`-th particle to a rigid body, at the particle's current position.
     */
    public attachParticle(i: number, body: RigidBody) {
        this.rawSet.sbAttachParticle(
            this.handle,
            i,
            body.handle,
            this.bodies.raw,
        );
    }

    /**
     * Detaches the `i`-th particle from the rigid body it was attached to.
     *
     * Returns `false` if the particle was not attached.
     */
    public detachParticle(i: number): boolean {
        return this.rawSet.sbDetachParticle(this.handle, i);
    }

    /**
     * The number of particles attached to rigid bodies.
     */
    public numAttachments(): number {
        return this.rawSet.sbNumAttachments(this.handle);
    }

    /**
     * The particle of the `i`-th attachment.
     */
    public attachmentParticle(i: number): number {
        return this.rawSet.sbAttachmentParticle(this.handle, i);
    }

    /**
     * The rigid body of the `i`-th attachment.
     */
    public attachmentBody(i: number): RigidBody {
        return this.bodies.get(this.rawSet.sbAttachmentBody(this.handle, i));
    }

    /*
     * Elements.
     */

    /**
     * The number of edges (structural and bending) of this soft body.
     */
    public numEdges(): number {
        return this.rawSet.sbNumEdges(this.handle);
    }

    /**
     * The particle pairs of every edge, flattened (two indices per edge).
     */
    public edges(): Uint32Array {
        return this.rawSet.sbEdges(this.handle);
    }

    /**
     * The rest length of the `i`-th edge.
     */
    public edgeRestLength(i: number): number {
        return this.rawSet.sbEdgeRestLength(this.handle, i);
    }

    /**
     * Is the `i`-th edge a bending edge (rather than a structural one)?
     */
    public isEdgeBend(i: number): boolean {
        return this.rawSet.sbEdgeIsBend(this.handle, i);
    }

    /**
     * The impulse applied by the `i`-th edge during the last step.
     */
    public edgeImpulse(i: number): number {
        return this.rawSet.sbEdgeImpulse(this.handle, i);
    }

    /**
     * The load of the `i`-th edge relative to its tear threshold (`1` tears it).
     */
    public edgeStress(i: number): number {
        return this.rawSet.sbEdgeStress(this.handle, i);
    }

    /**
     * The relative change of the `i`-th edge's rest length due to plastic flow.
     */
    public edgePlasticStrain(i: number): number {
        return this.rawSet.sbEdgePlasticStrain(this.handle, i);
    }

    /**
     * The multiplier of the tear thresholds of the `i`-th edge.
     */
    public edgeTearResistance(i: number): number {
        return this.rawSet.sbEdgeTearResistance(this.handle, i);
    }

    /**
     * The number of cells (triangles in 2D, tetrahedra in 3D) of this soft body.
     */
    public numCells(): number {
        return this.rawSet.sbNumCells(this.handle);
    }

    /**
     * The particles of every cell, flattened (three indices per cell in 2D, four in 3D).
     */
    public cells(): Uint32Array {
        return this.rawSet.sbCells(this.handle);
    }

    /**
     * The rest area/volume of the `i`-th cell.
     */
    public cellRestVolume(i: number): number {
        return this.rawSet.sbCellRestVolume(this.handle, i);
    }

    /**
     * The load of the `i`-th cell relative to its tear threshold (`1` tears it).
     */
    public cellStress(i: number): number {
        return this.rawSet.sbCellStress(this.handle, i);
    }

    /**
     * The stiffness multiplier of the `i`-th cell.
     */
    public cellStiffnessScale(i: number): number {
        return this.rawSet.sbCellStiffnessScale(this.handle, i);
    }

    /**
     * The multiplier of the tear thresholds of the `i`-th cell.
     */
    public cellTearResistance(i: number): number {
        return this.rawSet.sbCellTearResistance(this.handle, i);
    }

    // #if DIM3
    /**
     * The number of dihedral bending constraints of this soft body.
     */
    public numDihedrals(): number {
        return this.rawSet.sbNumDihedrals(this.handle);
    }

    /**
     * The particles of every dihedral, flattened (four indices per dihedral: the shared edge,
     * then the two opposite vertices).
     */
    public dihedrals(): Uint32Array {
        return this.rawSet.sbDihedrals(this.handle);
    }

    /**
     * The rest angle of the `i`-th dihedral.
     */
    public dihedralRestAngle(i: number): number {
        return this.rawSet.sbDihedralRestAngle(this.handle, i);
    }
    // #endif

    /**
     * The boundary elements of this soft body (segments in 2D, triangles in 3D), flattened
     * (two or three particle indices per element), oriented outward.
     */
    public boundary(): Uint32Array {
        return this.rawSet.sbBoundary(this.handle);
    }

    /*
     * Material.
     */

    /**
     * A copy of this soft body's material.
     */
    public material(): SoftBodyMaterial {
        return SoftBodyMaterial.fromRaw(this.rawSet.sbMaterial(this.handle));
    }

    /**
     * Replaces this soft body's material.
     */
    public setMaterial(material: SoftBodyMaterial) {
        let raw = material.intoRaw();
        this.rawSet.sbSetMaterial(this.handle, raw);
        raw.free();
    }

    /**
     * The constitutive model of this soft body's cells.
     */
    public cellModel(): SoftBodyCellModel {
        return this.rawSet.sbCellModel(
            this.handle,
        ) as number as SoftBodyCellModel;
    }

    /**
     * Is the global area/volume preservation of this soft body enabled?
     */
    public volumePreservationEnabled(): boolean {
        return this.rawSet.sbVolumePreservationEnabled(this.handle);
    }

    /**
     * Enables or disables the global area/volume preservation of this soft body.
     */
    public enableVolumePreservation(enabled: boolean) {
        this.rawSet.sbEnableVolumePreservation(this.handle, enabled);
    }

    /**
     * The rest area/volume enclosed by this soft body's closed surfaces.
     */
    public restVolume(): number {
        return this.rawSet.sbRestVolume(this.handle);
    }

    /**
     * The current area/volume enclosed by this soft body's closed surfaces.
     */
    public volume(): number {
        return this.rawSet.sbVolume(this.handle);
    }

    /**
     * The target volume multiplier of the volume preservation (`> 1` inflates the body).
     */
    public volumeFactor(): number {
        return this.rawSet.sbVolumeFactor(this.handle);
    }

    /**
     * Sets the target volume multiplier of the volume preservation (`> 1` inflates the body).
     */
    public setVolumeFactor(factor: number) {
        this.rawSet.sbSetVolumeFactor(this.handle, factor);
    }

    /**
     * The thickness of this soft body's particles.
     */
    public particleRadius(): number {
        return this.rawSet.sbParticleRadius(this.handle);
    }

    /**
     * Forgets every plastic deformation: the rest shapes return to their initial values.
     */
    public resetPlasticity() {
        this.rawSet.sbResetPlasticity(this.handle);
    }

    /*
     * Whole-body state.
     */

    /**
     * The hidden rigid body standing for this soft body in joints and islands.
     */
    public rootBody(): RigidBody {
        return this.bodies.get(this.rawSet.sbRootBody(this.handle));
    }

    /**
     * The soft body this one was split off from by a tear, if any.
     */
    public origin(): SoftBodyHandle | null {
        let origin = this.rawSet.sbOrigin(this.handle);
        return origin === undefined ? null : origin;
    }

    /**
     * The handles of the soft bodies that tears split off from this one.
     */
    public pieces(): SoftBodyHandle[] {
        return Array.from(this.rawSet.sbPieces(this.handle));
    }

    /**
     * The center of mass of this soft body's particles.
     */
    public centerOfMass(target?: Vector): Vector {
        this.rawSet.sbCenterOfMass(this.handle, scratchBuffer);
        return VectorOps.fromBuffer(scratchBuffer, target);
    }

    /**
     * The total mass of this soft body's particles.
     */
    public mass(): number {
        return this.rawSet.sbMass(this.handle);
    }

    /**
     * Is this soft body sleeping?
     */
    public isSleeping(): boolean {
        return this.rawSet.sbIsSleeping(this.handle);
    }

    /**
     * Wakes this soft body up.
     */
    public wakeUp() {
        this.rawSet.sbWakeUp(this.handle);
    }

    /**
     * Is this soft body enabled (simulated)?
     */
    public isEnabled(): boolean {
        return this.rawSet.sbIsEnabled(this.handle);
    }

    /**
     * Enables or disables this soft body.
     */
    public setEnabled(enabled: boolean) {
        this.rawSet.sbSetEnabled(this.handle, enabled);
    }

    /**
     * Sets the extra internal PGS iterations run per substep for this soft body and
     * everything it touches.
     */
    public setAdditionalPgsIterations(iterations: number) {
        this.rawSet.sbSetAdditionalPgsIterations(this.handle, iterations);
    }

    /**
     * The linear damping of this soft body's particles.
     */
    public linearDamping(): number {
        return this.rawSet.sbLinearDamping(this.handle);
    }

    /**
     * The gravity scale of this soft body's particles.
     */
    public gravityScale(): number {
        return this.rawSet.sbGravityScale(this.handle);
    }

    /*
     * Forces and impulses.
     */

    /**
     * Adds a force to every particle of this soft body (spread by mass).
     */
    public addForce(force: Vector, wakeUp: boolean) {
        let rawForce = VectorOps.intoRaw(force);
        this.rawSet.sbAddForce(this.handle, rawForce, wakeUp);
        rawForce.free();
    }

    /**
     * Adds a force to the `i`-th particle of this soft body.
     */
    public addParticleForce(i: number, force: Vector, wakeUp: boolean) {
        let rawForce = VectorOps.intoRaw(force);
        this.rawSet.sbAddParticleForce(this.handle, i, rawForce, wakeUp);
        rawForce.free();
    }

    /**
     * Resets the user forces applied to this soft body's particles.
     */
    public resetForces(wakeUp: boolean) {
        this.rawSet.sbResetForces(this.handle, wakeUp);
    }

    /**
     * Applies an impulse to every particle of this soft body (spread by mass).
     */
    public applyImpulse(impulse: Vector, wakeUp: boolean) {
        let rawImpulse = VectorOps.intoRaw(impulse);
        this.rawSet.sbApplyImpulse(this.handle, rawImpulse, wakeUp);
        rawImpulse.free();
    }

    /**
     * Applies an impulse to the `i`-th particle of this soft body.
     */
    public applyParticleImpulse(i: number, impulse: Vector, wakeUp: boolean) {
        let rawImpulse = VectorOps.intoRaw(impulse);
        this.rawSet.sbApplyParticleImpulse(this.handle, i, rawImpulse, wakeUp);
        rawImpulse.free();
    }

    /**
     * Applies an impulse to the particles within `falloffRadius` of `point`, scaled down
     * linearly with their distance to it (`falloffRadius <= 0` applies it to every particle).
     */
    public applyImpulseAtPoint(
        impulse: Vector,
        point: Vector,
        falloffRadius: number,
        wakeUp: boolean,
    ) {
        let rawImpulse = VectorOps.intoRaw(impulse);
        let rawPoint = VectorOps.intoRaw(point);
        this.rawSet.sbApplyImpulseAtPoint(
            this.handle,
            rawImpulse,
            rawPoint,
            falloffRadius,
            wakeUp,
        );
        rawImpulse.free();
        rawPoint.free();
    }

    /**
     * Applies an impulse of the given magnitude pushing the particles away from `center`
     * (a blast), scaled down linearly up to `falloffRadius`.
     */
    public applyRadialImpulse(
        center: Vector,
        magnitude: number,
        falloffRadius: number,
        wakeUp: boolean,
    ) {
        let rawCenter = VectorOps.intoRaw(center);
        this.rawSet.sbApplyRadialImpulse(
            this.handle,
            rawCenter,
            magnitude,
            falloffRadius,
            wakeUp,
        );
        rawCenter.free();
    }

    /*
     * Tearing.
     */

    /**
     * Requests the `i`-th edge to tear at the end of the next step.
     */
    public tearEdge(i: number) {
        this.rawSet.sbTearEdge(this.handle, i);
    }

    /**
     * Requests the `i`-th cell to tear at the end of the next step.
     */
    public tearCell(i: number) {
        this.rawSet.sbTearCell(this.handle, i);
    }

    /**
     * Are there tears requested for the next step?
     */
    public hasPendingTears(): boolean {
        return this.rawSet.sbHasPendingTears(this.handle);
    }

    /*
     * Clusters.
     */

    /**
     * The number of cluster slots of this soft body (some may have been removed: see
     * `isClusterLive`).
     */
    public numClusters(): number {
        return this.rawSet.sbNumClusters(this.handle);
    }

    /**
     * Does the `i`-th cluster still exist?
     */
    public isClusterLive(i: number): boolean {
        return this.rawSet.sbIsClusterLive(this.handle, i);
    }

    /**
     * The rigid body standing for the `i`-th cluster: joints and colliders attach to it.
     */
    public clusterProxy(i: number): RigidBody | null {
        let proxy = this.rawSet.sbClusterProxy(this.handle, i);
        return proxy === undefined ? null : this.bodies.get(proxy);
    }

    /**
     * The particles of the `i`-th cluster.
     */
    public clusterParticles(i: number): Uint32Array {
        return this.rawSet.sbClusterParticles(this.handle, i);
    }

    /**
     * Does the `i`-th cluster hold its shape by shape matching?
     */
    public clusterShapeMatchingEnabled(i: number): boolean {
        return this.rawSet.sbClusterShapeMatchingEnabled(this.handle, i);
    }

    /**
     * Enables or disables shape matching on the `i`-th cluster.
     */
    public enableClusterShapeMatching(i: number, enabled: boolean) {
        this.rawSet.sbEnableClusterShapeMatching(this.handle, i, enabled);
    }

    /**
     * Scales the stiffness of the elements of the `i`-th cluster.
     */
    public setClusterStiffnessScale(i: number, scale: number) {
        this.rawSet.sbSetClusterStiffnessScale(this.handle, i, scale);
    }

    /**
     * Overrides the softness of the edges of the `i`-th cluster (`null` restores the
     * material's).
     */
    public setClusterEdgeSoftness(
        i: number,
        softness: SpringCoefficients | null,
    ) {
        this.rawSet.sbSetClusterEdgeSoftness(
            this.handle,
            i,
            softness ? softness.naturalFrequency : undefined,
            softness ? softness.dampingRatio : undefined,
        );
    }

    /**
     * Scales the tear thresholds of the elements of the `i`-th cluster.
     */
    public setClusterTearResistance(i: number, resistance: number) {
        this.rawSet.sbSetClusterTearResistance(this.handle, i, resistance);
    }

    /**
     * Pins (or releases) every particle of the `i`-th cluster.
     */
    public setClusterPinned(i: number, pinned: boolean) {
        this.rawSet.sbSetClusterPinned(this.handle, i, pinned);
    }

    /**
     * Moves the `i`-th cluster rigidly to the given pose over the next step.
     */
    public setClusterKinematicTarget(
        i: number,
        translation: Vector,
        rotation: Rotation,
    ) {
        let rawTra = VectorOps.intoRaw(translation);
        let rawRot = RotationOps.intoRaw(rotation);
        this.rawSet.sbSetClusterKinematicTarget(this.handle, i, rawTra, rawRot);
        rawTra.free();
        rawRot.free();
    }

    /*
     * Collision meshes.
     */

    /**
     * The number of collision meshes held by this soft body's clusters (its own deformable
     * surface included).
     */
    public numMeshes(): number {
        return this.rawSet.sbNumMeshes(this.handle);
    }

    /**
     * The cluster holding the `i`-th collision mesh.
     */
    public meshCluster(i: number): number | null {
        let cluster = this.rawSet.sbMeshCluster(this.handle, i);
        return cluster === undefined ? null : cluster;
    }

    /**
     * The collider holding the `i`-th collision mesh.
     */
    public meshCollider(i: number): Collider | null {
        let collider = this.rawSet.sbMeshCollider(this.handle, i);
        return collider === undefined ? null : this.colliders.get(collider);
    }

    /**
     * Is the `i`-th collision mesh skinned (embedded in the cells) rather than bound
     * vertex-to-particle?
     */
    public isMeshSkinned(i: number): boolean {
        return this.rawSet.sbMeshIsSkinned(this.handle, i);
    }

    /**
     * Does the `i`-th collision mesh collide?
     */
    public meshCollisionEnabled(i: number): boolean {
        return this.rawSet.sbMeshCollisionEnabled(this.handle, i);
    }

    /**
     * Does the shape of the `i`-th collision mesh carry the `ORIENTED` flag (see
     * `SoftBodyDesc.setOriented`)?
     */
    public isMeshOriented(i: number): boolean {
        return this.rawSet.sbMeshIsOriented(this.handle, i);
    }

    /**
     * The world-space vertex positions of the `i`-th collision mesh, flattened.
     */
    public meshVertices(i: number): Float32Array {
        return this.rawSet.sbMeshVertices(this.handle, i);
    }

    /**
     * The elements of the `i`-th collision mesh, flattened (two vertex indices per segment in
     * 2D, three per triangle in 3D).
     */
    public meshIndices(i: number): Uint32Array {
        return this.rawSet.sbMeshIndices(this.handle, i);
    }

    /**
     * The index of the collision mesh held by a deformable collider of this soft body.
     */
    public meshOfCollider(collider: Collider): number | null {
        let mesh = this.rawSet.sbMeshOfCollider(this.handle, collider.handle);
        return mesh === undefined ? null : mesh;
    }
}

/**
 * The description of a soft body to create.
 *
 * Start from one of the generators (`SoftBodyDesc.rope`, `cloth`, `cuboid`, `sphere`,
 * `trimesh`, `volumetric` in 3D; `rope`, `polygon`, `disk`, `grid`, `polyline`, `volumetric`
 * in 2D) or from raw particle positions, then tune it with the chainable setters. The
 * description holds no WASM memory: it is applied when the soft body is created.
 */
export class SoftBodyDesc {
    private generator: () => RawSoftBodyBuilder;
    private setters: Array<(raw: RawSoftBodyBuilder) => void>;
    /**
     * The material of the soft body, or `null` to keep the default one (with `softness`
     * applied on top when set).
     */
    material: SoftBodyMaterial | null;
    /**
     * A uniform softness applied to every constraint of the material, or `null`.
     */
    softness: SpringCoefficients | null;
    /**
     * The uniform mass of the particles (ignored when `masses` is set).
     */
    particleMass: number;
    /**
     * The total mass of the body, spread over its particles, or `null` to use `particleMass`.
     */
    mass: number | null;
    /**
     * Per-particle masses, or `null` for uniform masses.
     */
    masses: Float32Array | null;
    /**
     * The indices of the pinned particles.
     */
    pinnedParticles: Uint32Array;
    /**
     * The constitutive model of the cells.
     */
    cellModel: SoftBodyCellModel;
    /**
     * Whether the area/volume enclosed by the body's closed surfaces is preserved (`null`:
     * what the generator chose, off for raw positions).
     */
    volumePreservation: boolean | null;
    /**
     * The target volume multiplier of the volume preservation (`> 1` inflates the body).
     */
    volumeFactor: number;
    /**
     * Whether shape matching holds the body's shape (`null`: what the generator chose, off for
     * raw positions).
     */
    shapeMatching: boolean | null;
    /**
     * Whether the body's surface collides with itself.
     */
    selfContacts: boolean;
    /**
     * Whether the shape of the body's collision surface is built with the `ORIENTED` flag
     * (`null`: whenever the surface is closed).
     */
    oriented: boolean | null;
    /**
     * The thickness of the particles (`null`: what the generator chose).
     */
    particleRadius: number | null;
    /**
     * The template of the body's colliders (their shape is replaced by the body's deformable
     * surface), or `null` for a body without collisions.
     */
    surfaceCollider: ColliderDesc | null;
    /**
     * Whether the body collides through its skin rather than through its cells' boundary.
     */
    skinCollision: boolean;
    /**
     * A translation applied to every particle.
     */
    translation: Vector | null;
    /**
     * Linear damping of the particles.
     */
    linearDamping: number;
    /**
     * Gravity scale of the particles.
     */
    gravityScale: number;
    /**
     * Extra solver substeps requested for the body and everything it touches.
     */
    additionalSolverIterations: number;
    /**
     * Extra internal PGS iterations per substep for the body and everything it touches.
     */
    additionalPgsIterations: number;
    /**
     * Whether the body may fall asleep.
     */
    canSleep: boolean;
    /**
     * The dominance group of the body.
     */
    dominanceGroup: number;
    /**
     * An arbitrary user-defined object associated with the soft body.
     */
    userData?: unknown;

    /**
     * A description over raw world-space particle positions (flattened), with no element:
     * add edges, cells and a surface with the setters.
     */
    constructor(positions?: Float32Array | number[]) {
        let flat = positions
            ? Float32Array.from(positions)
            : new Float32Array(0);
        this.generator = () => new RawSoftBodyBuilder(flat);
        this.setters = [];
        this.material = null;
        this.softness = null;
        this.particleMass = 1.0;
        this.mass = null;
        this.masses = null;
        this.pinnedParticles = new Uint32Array(0);
        this.cellModel = SoftBodyCellModel.Volume;
        this.volumePreservation = null;
        this.volumeFactor = 1.0;
        this.shapeMatching = null;
        this.selfContacts = false;
        this.oriented = null;
        this.particleRadius = null;
        this.surfaceCollider = ColliderDesc.ball(0.05);
        this.skinCollision = false;
        this.translation = null;
        this.linearDamping = 0.0;
        this.gravityScale = 1.0;
        this.additionalSolverIterations = 0;
        this.additionalPgsIterations = 3;
        this.canSleep = true;
        this.dominanceGroup = 0;
    }

    private static withGenerator(
        generator: () => RawSoftBodyBuilder | undefined,
    ): SoftBodyDesc | null {
        // Run it once to know whether the geometry is valid.
        let probe = generator();
        if (!probe) {
            return null;
        }
        probe.free();
        let desc = new SoftBodyDesc();
        desc.generator = generator as () => RawSoftBodyBuilder;
        return desc;
    }

    /**
     * A rope of `numParticles` particles from `start` to `end`.
     */
    public static rope(
        start: Vector,
        end: Vector,
        numParticles: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawStart = VectorOps.intoRaw(start);
            let rawEnd = VectorOps.intoRaw(end);
            let raw = RawSoftBodyBuilder.rope(rawStart, rawEnd, numParticles);
            rawStart.free();
            rawEnd.free();
            return raw;
        });
    }

    /**
     * A body filling the closed surface (segments in 2D, triangles in 3D) with cells of the
     * given size; `skinned` keeps the surface as a skin embedded in the cells. Returns
     * `null` if the surface cannot be meshed.
     *
     * @param vertices - The flattened world-space vertices of the surface.
     * @param indices - The flattened elements of the surface.
     */
    public static volumetric(
        vertices: Float32Array,
        indices: Uint32Array,
        cellSize: number,
        skinned: boolean = false,
    ): SoftBodyDesc | null {
        return SoftBodyDesc.withGenerator(() =>
            RawSoftBodyBuilder.volumetric(vertices, indices, cellSize, skinned),
        );
    }

    // #if DIM2
    /**
     * A polyline body: the vertices are particles linked by the segments (a line strip when
     * `indices` is not given). Returns `null` if the polyline is empty.
     */
    public static polyline(
        vertices: Float32Array,
        indices?: Uint32Array,
    ): SoftBodyDesc | null {
        let idx = indices ?? new Uint32Array(0);
        return SoftBodyDesc.withGenerator(() =>
            RawSoftBodyBuilder.polyline(vertices, idx),
        );
    }

    /**
     * A closed polygon of particles (flattened points) holding its area.
     */
    public static polygon(points: Float32Array | number[]): SoftBodyDesc {
        let flat = Float32Array.from(points);
        return SoftBodyDesc.withGenerator(() =>
            RawSoftBodyBuilder.polygon(flat),
        );
    }

    /**
     * A disk: a ring of `numParticles` particles holding its area (a pressurized blob).
     */
    public static disk(
        center: Vector,
        radius: number,
        numParticles: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawCenter = VectorOps.intoRaw(center);
            let raw = RawSoftBodyBuilder.disk(rawCenter, radius, numParticles);
            rawCenter.free();
            return raw;
        });
    }

    /**
     * A grid of `nx` by `ny` particles filled with triangle cells.
     */
    public static grid(
        center: Vector,
        halfExtents: Vector,
        nx: number,
        ny: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawCenter = VectorOps.intoRaw(center);
            let rawHalf = VectorOps.intoRaw(halfExtents);
            let raw = RawSoftBodyBuilder.grid(rawCenter, rawHalf, nx, ny);
            rawCenter.free();
            rawHalf.free();
            return raw;
        });
    }
    // #endif

    // #if DIM3
    /**
     * A triangle-mesh body without cells: the vertices are particles, the triangles the
     * surface, held by dihedral bending and shape matching. Returns `null` if the mesh is
     * empty.
     */
    public static trimesh(
        vertices: Float32Array,
        indices: Uint32Array,
    ): SoftBodyDesc | null {
        return SoftBodyDesc.withGenerator(() =>
            RawSoftBodyBuilder.trimesh(vertices, indices),
        );
    }

    /**
     * A cloth of `nu` by `nv` particles: particle `(i, j)` is at `origin + i * du + j * dv`.
     */
    public static cloth(
        origin: Vector,
        du: Vector,
        dv: Vector,
        nu: number,
        nv: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawOrigin = VectorOps.intoRaw(origin);
            let rawDu = VectorOps.intoRaw(du);
            let rawDv = VectorOps.intoRaw(dv);
            let raw = RawSoftBodyBuilder.cloth(rawOrigin, rawDu, rawDv, nu, nv);
            rawOrigin.free();
            rawDu.free();
            rawDv.free();
            return raw;
        });
    }

    /**
     * A tube of cloth around `axis`, from `radiusStart` at `origin` to `radiusEnd` at its
     * other end, with `numAround` particles per ring and `numAlong` rings.
     */
    public static clothTube(
        origin: Vector,
        axis: Vector,
        radiusStart: number,
        radiusEnd: number,
        numAround: number,
        numAlong: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawOrigin = VectorOps.intoRaw(origin);
            let rawAxis = VectorOps.intoRaw(axis);
            let raw = RawSoftBodyBuilder.clothTube(
                rawOrigin,
                rawAxis,
                radiusStart,
                radiusEnd,
                numAround,
                numAlong,
            );
            rawOrigin.free();
            rawAxis.free();
            return raw;
        });
    }

    /**
     * A cloth with different softness along `du` (warp), along `dv` (weft) and across the
     * diagonals (shear).
     */
    public static clothAnisotropic(
        origin: Vector,
        du: Vector,
        dv: Vector,
        nu: number,
        nv: number,
        warp: SpringCoefficients,
        weft: SpringCoefficients,
        shear: SpringCoefficients,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawOrigin = VectorOps.intoRaw(origin);
            let rawDu = VectorOps.intoRaw(du);
            let rawDv = VectorOps.intoRaw(dv);
            let raw = RawSoftBodyBuilder.clothAnisotropic(
                rawOrigin,
                rawDu,
                rawDv,
                nu,
                nv,
                warp.naturalFrequency,
                warp.dampingRatio,
                weft.naturalFrequency,
                weft.dampingRatio,
                shear.naturalFrequency,
                shear.dampingRatio,
            );
            rawOrigin.free();
            rawDu.free();
            rawDv.free();
            return raw;
        });
    }

    /**
     * A box of `nx` by `ny` by `nz` particles filled with tetrahedral cells.
     */
    public static cuboid(
        center: Vector,
        halfExtents: Vector,
        nx: number,
        ny: number,
        nz: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawCenter = VectorOps.intoRaw(center);
            let rawHalf = VectorOps.intoRaw(halfExtents);
            let raw = RawSoftBodyBuilder.cuboid(rawCenter, rawHalf, nx, ny, nz);
            rawCenter.free();
            rawHalf.free();
            return raw;
        });
    }

    /**
     * A hollow sphere: an icosphere surface with `subdivisions` refinement levels, holding
     * its volume (a balloon).
     */
    public static sphere(
        center: Vector,
        radius: number,
        subdivisions: number,
    ): SoftBodyDesc {
        return SoftBodyDesc.withGenerator(() => {
            let rawCenter = VectorOps.intoRaw(center);
            let raw = RawSoftBodyBuilder.sphere(
                rawCenter,
                radius,
                subdivisions,
            );
            rawCenter.free();
            return raw;
        });
    }
    // #endif

    /*
     * Chainable setters.
     */

    /**
     * Sets the material of the soft body.
     */
    public setMaterial(material: SoftBodyMaterial): SoftBodyDesc {
        this.material = material;
        return this;
    }

    /**
     * Sets a uniform softness for every constraint of the material.
     */
    public setSoftness(
        naturalFrequency: number,
        dampingRatio: number,
    ): SoftBodyDesc {
        this.softness = {naturalFrequency, dampingRatio};
        return this;
    }

    /**
     * Sets the uniform mass of the particles.
     */
    public setParticleMass(mass: number): SoftBodyDesc {
        this.particleMass = mass;
        this.masses = null;
        return this;
    }

    /**
     * Sets the total mass of the body, spread over its particles.
     */
    public setMass(mass: number): SoftBodyDesc {
        this.mass = mass;
        return this;
    }

    /**
     * Sets per-particle masses.
     */
    public setMasses(masses: Float32Array | number[]): SoftBodyDesc {
        this.masses = Float32Array.from(masses);
        return this;
    }

    /**
     * Pins the given particles in place.
     */
    public setPinnedParticles(pinned: Uint32Array | number[]): SoftBodyDesc {
        this.pinnedParticles = Uint32Array.from(pinned);
        return this;
    }

    /**
     * Replaces the structural edges (flattened particle pairs).
     */
    public setEdges(edges: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(edges);
        this.setters.push((raw) => raw.setEdges(flat));
        return this;
    }

    /**
     * Adds structural edges (flattened particle pairs).
     */
    public addEdges(edges: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(edges);
        this.setters.push((raw) => raw.addEdges(flat));
        return this;
    }

    /**
     * Replaces the bending edges (flattened particle pairs).
     */
    public setBendEdges(edges: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(edges);
        this.setters.push((raw) => raw.setBendEdges(flat));
        return this;
    }

    /**
     * Makes every edge resist stretching only (a rope or a net that folds freely).
     */
    public setTensionOnly(): SoftBodyDesc {
        this.setters.push((raw) => raw.setTensionOnly());
        return this;
    }

    // #if DIM3
    /**
     * Replaces the dihedral bending constraints (flattened quadruplets: the shared edge, then
     * the two opposite vertices).
     */
    public setDihedrals(dihedrals: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(dihedrals);
        this.setters.push((raw) => raw.setDihedrals(flat));
        return this;
    }

    /**
     * Sets the segments a body without surface collides through (a wire).
     */
    public setWire(segments: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(segments);
        this.setters.push((raw) => raw.setWire(flat));
        return this;
    }
    // #endif

    /**
     * Replaces the cells (flattened triangles in 2D, tetrahedra in 3D).
     */
    public setCells(cells: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(cells);
        this.setters.push((raw) => raw.setCells(flat));
        return this;
    }

    /**
     * Replaces the boundary elements (flattened segments in 2D, triangles in 3D), oriented
     * outward.
     */
    public setSurface(surface: Uint32Array | number[]): SoftBodyDesc {
        let flat = Uint32Array.from(surface);
        this.setters.push((raw) => raw.setSurface(flat));
        return this;
    }

    /**
     * Sets a skin: a finer mesh (flattened world-space vertices and elements) embedded in the
     * cells, which follows them.
     */
    public setSkin(
        vertices: Float32Array | number[],
        indices: Uint32Array | number[],
    ): SoftBodyDesc {
        let flatVertices = Float32Array.from(vertices);
        let flatIndices = Uint32Array.from(indices);
        this.setters.push((raw) => raw.setSkin(flatVertices, flatIndices));
        return this;
    }

    /**
     * Makes the body collide through its skin rather than through its cells' boundary.
     */
    public setSkinCollision(enabled: boolean): SoftBodyDesc {
        this.skinCollision = enabled;
        return this;
    }

    /**
     * Overrides the softness of the given edges.
     */
    public setEdgeSoftness(
        edges: Uint32Array | number[],
        softness: SpringCoefficients[],
    ): SoftBodyDesc {
        let flatEdges = Uint32Array.from(edges);
        let frequencies = Float32Array.from(
            softness.map((s) => s.naturalFrequency),
        );
        let dampings = Float32Array.from(softness.map((s) => s.dampingRatio));
        this.setters.push((raw) =>
            raw.setEdgeSoftness(flatEdges, frequencies, dampings),
        );
        return this;
    }

    /**
     * Multiplies the tear thresholds of the given edges.
     */
    public setEdgeTearResistance(
        edges: Uint32Array | number[],
        resistances: Float32Array | number[],
    ): SoftBodyDesc {
        let flatEdges = Uint32Array.from(edges);
        let flatResistances = Float32Array.from(resistances);
        this.setters.push((raw) =>
            raw.setEdgeTearResistance(flatEdges, flatResistances),
        );
        return this;
    }

    /**
     * Sets the constitutive model of the cells.
     */
    public setCellModel(model: SoftBodyCellModel): SoftBodyDesc {
        this.cellModel = model;
        return this;
    }

    /**
     * Enables the preservation of the area/volume enclosed by the body's closed surfaces.
     */
    public setVolumePreservation(enabled: boolean): SoftBodyDesc {
        this.volumePreservation = enabled;
        return this;
    }

    /**
     * Sets the target volume multiplier (`> 1` inflates the body); this also enables the
     * volume preservation.
     */
    public setVolumeFactor(factor: number): SoftBodyDesc {
        this.volumeFactor = factor;
        this.volumePreservation = true;
        return this;
    }

    /**
     * Holds the body's shape by shape matching.
     */
    public setShapeMatching(enabled: boolean): SoftBodyDesc {
        this.shapeMatching = enabled;
        return this;
    }

    /**
     * Makes the body's surface collide with itself.
     */
    public setSelfContacts(enabled: boolean): SoftBodyDesc {
        this.selfContacts = enabled;
        return this;
    }

    /**
     * Sets whether the shape of the body's collision surface is built with the `ORIENTED`
     * flag, like a polyline or mesh. Left unset, it is whenever the surface is closed, which
     * is what a solid body wants: an oriented closed surface encloses matter, so nothing is
     * held inside it. Set it to `false` for a shell, whose inner side holds the bodies inside
     * it.
     */
    public setOriented(oriented: boolean): SoftBodyDesc {
        this.oriented = oriented;
        return this;
    }

    /**
     * Sets the thickness of the particles.
     */
    public setParticleRadius(radius: number): SoftBodyDesc {
        this.particleRadius = radius;
        return this;
    }

    /**
     * Sets the template of the body's colliders: its friction, restitution, groups, events
     * and other settings are kept, its shape is replaced by the body's deformable surface.
     */
    public setSurfaceCollider(collider: ColliderDesc): SoftBodyDesc {
        this.surfaceCollider = collider;
        return this;
    }

    /**
     * Removes the body's colliders: it will not collide with anything.
     */
    public setNoSurfaceCollider(): SoftBodyDesc {
        this.surfaceCollider = null;
        return this;
    }

    /**
     * Translates every particle.
     */
    public setTranslation(translation: Vector): SoftBodyDesc {
        this.translation = translation;
        return this;
    }

    /**
     * Sets the linear damping of the particles.
     */
    public setLinearDamping(damping: number): SoftBodyDesc {
        this.linearDamping = damping;
        return this;
    }

    /**
     * Sets the gravity scale of the particles.
     */
    public setGravityScale(scale: number): SoftBodyDesc {
        this.gravityScale = scale;
        return this;
    }

    /**
     * Sets the extra solver substeps requested for the body and everything it touches.
     */
    public setAdditionalSolverIterations(iterations: number): SoftBodyDesc {
        this.additionalSolverIterations = iterations;
        return this;
    }

    /**
     * Sets the extra internal PGS iterations per substep for the body and everything it
     * touches (default: `3`).
     */
    public setAdditionalPgsIterations(iterations: number): SoftBodyDesc {
        this.additionalPgsIterations = iterations;
        return this;
    }

    /**
     * Sets whether the body may fall asleep.
     */
    public setCanSleep(canSleep: boolean): SoftBodyDesc {
        this.canSleep = canSleep;
        return this;
    }

    /**
     * Sets the dominance group of the body.
     */
    public setDominanceGroup(group: number): SoftBodyDesc {
        this.dominanceGroup = group;
        return this;
    }

    /**
     * Sets the user data associated with the soft body.
     */
    public setUserData(data: unknown): SoftBodyDesc {
        this.userData = data;
        return this;
    }

    /**
     * Appends the particles and elements of another description to this one.
     */
    public append(other: SoftBodyDesc): SoftBodyDesc {
        this.setters.push((raw) => {
            let rawOther = other.intoRaw();
            raw.append(rawOther);
            rawOther.free();
        });
        return this;
    }

    /**
     * Builds the WASM-side builder this description stands for. The result must be freed.
     * @internal
     */
    public intoRaw(): RawSoftBodyBuilder {
        let raw = this.generator();
        for (let setter of this.setters) {
            setter(raw);
        }
        if (this.material) {
            let rawMaterial = this.material.intoRaw();
            raw.setMaterial(rawMaterial);
            rawMaterial.free();
        }
        if (this.softness) {
            raw.setSoftness(
                this.softness.naturalFrequency,
                this.softness.dampingRatio,
            );
        }
        if (this.masses) {
            raw.setMasses(this.masses);
        } else {
            raw.setParticleMass(this.particleMass);
        }
        if (this.mass !== null) {
            raw.setMass(this.mass);
        }
        raw.setPinnedParticles(this.pinnedParticles);
        raw.setCellModel(this.cellModel as number as RawSoftBodyCellModel);
        // The settings a generator may have chosen are only overridden when set here.
        if (this.volumePreservation !== null) {
            raw.setVolumePreservation(this.volumePreservation);
        }
        raw.setVolumeFactor(this.volumeFactor);
        if (this.shapeMatching !== null) {
            raw.setShapeMatching(this.shapeMatching);
        }
        raw.setSelfContacts(this.selfContacts);
        if (this.oriented !== null) {
            raw.setOriented(this.oriented);
        }
        if (this.particleRadius !== null) {
            raw.setParticleRadius(this.particleRadius);
        }
        raw.setSkinCollision(this.skinCollision);
        if (this.surfaceCollider) {
            let c = this.surfaceCollider;
            raw.setSurfaceCollider(
                c.friction,
                c.restitution,
                c.frictionCombineRule,
                c.restitutionCombineRule,
                c.isSensor,
                c.collisionGroups,
                c.solverGroups,
                c.activeCollisionTypes,
                c.activeHooks,
                c.activeEvents,
                c.contactForceEventThreshold,
                c.contactSkin,
            );
        } else {
            raw.setNoSurfaceCollider();
        }
        if (this.translation) {
            let rawTra = VectorOps.intoRaw(this.translation);
            raw.translated(rawTra);
            rawTra.free();
        }
        raw.setLinearDamping(this.linearDamping);
        raw.setGravityScale(this.gravityScale);
        raw.setAdditionalSolverIterations(this.additionalSolverIterations);
        raw.setAdditionalPgsIterations(this.additionalPgsIterations);
        raw.setCanSleep(this.canSleep);
        raw.setDominanceGroup(this.dominanceGroup);
        return raw;
    }

    /**
     * The number of particles the description currently generates.
     */
    public numParticles(): number {
        let raw = this.intoRaw();
        let n = raw.numParticles();
        raw.free();
        return n;
    }

    /**
     * The flattened world-space positions of the particles the description generates.
     */
    public particlePositions(): Float32Array {
        let raw = this.intoRaw();
        let positions = raw.particlePositions();
        raw.free();
        return positions;
    }
}

/**
 * The record of a soft body tearing: the elements it lost, the particles the tear split,
 * the pieces the tear separated into soft bodies of their own, and the clusters and joints
 * that moved with them.
 *
 * Events drained from an `EventQueue` are only valid inside the draining closure; events
 * returned by `World.tearSoftBody` and `World.cutSoftBody` must be freed with `.free()`.
 */
export class SoftBodyTearEvent {
    raw: RawSoftBodyTearEvent;

    constructor(raw?: RawSoftBodyTearEvent) {
        this.raw = raw;
    }

    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    /**
     * The soft body that tore.
     */
    public softBody(): SoftBodyHandle {
        return this.raw.softBody();
    }

    /**
     * The particle pairs of the edges that tore, flattened.
     */
    public tornEdges(): Uint32Array {
        return this.raw.tornEdges();
    }

    /**
     * The particles of the cells that tore, flattened.
     */
    public tornCells(): Uint32Array {
        return this.raw.tornCells();
    }

    /**
     * The particle pairs of the edges the tear removed, flattened.
     */
    public removedEdges(): Uint32Array {
        return this.raw.removedEdges();
    }

    /**
     * The particles the tear split, flattened as `(original, copy)` pairs.
     */
    public splitParticles(): Uint32Array {
        return this.raw.splitParticles();
    }

    /**
     * The particles the tear inserted.
     */
    public insertedParticles(): Uint32Array {
        return this.raw.insertedParticles();
    }

    /**
     * The particle pairs (flattened) the tear started from.
     */
    public seeds(): Uint32Array {
        return this.raw.seeds();
    }

    /**
     * The number of pieces the tear split off into soft bodies of their own.
     */
    public numPieces(): number {
        return this.raw.numPieces();
    }

    /**
     * The soft body the `i`-th piece became.
     */
    public pieceSoftBody(i: number): SoftBodyHandle {
        return this.raw.pieceSoftBody(i);
    }

    /**
     * The particles (indices in the torn body) that went into the `i`-th piece.
     */
    public pieceParticles(i: number): Uint32Array {
        return this.raw.pieceParticles(i);
    }

    /**
     * The clusters that moved into the `i`-th piece, flattened as `(source, destination)`
     * index pairs.
     */
    public pieceClusters(i: number): Uint32Array {
        return this.raw.pieceClusters(i);
    }

    /**
     * The number of clusters the tear split.
     */
    public numClusterSplits(): number {
        return this.raw.numClusterSplits();
    }

    /**
     * The cluster of the torn body the `i`-th split came from.
     */
    public clusterSplitSource(i: number): number {
        return this.raw.clusterSplitSource(i);
    }

    /**
     * The soft body holding the cluster the `i`-th split created.
     */
    public clusterSplitSoftBody(i: number): SoftBodyHandle {
        return this.raw.clusterSplitSoftBody(i);
    }

    /**
     * The index of the cluster the `i`-th split created.
     */
    public clusterSplitCluster(i: number): number {
        return this.raw.clusterSplitCluster(i);
    }

    /**
     * The proxy rigid body of the cluster the `i`-th split created.
     */
    public clusterSplitProxy(i: number): RigidBodyHandle {
        return this.raw.clusterSplitProxy(i);
    }

    /**
     * Does the cluster the `i`-th split created keep the source cluster's proxy?
     */
    public clusterSplitKeepsProxy(i: number): boolean {
        return this.raw.clusterSplitKeepsProxy(i);
    }

    /**
     * The number of impulse joints the tear moved to another proxy.
     */
    public numMovedJoints(): number {
        return this.raw.numMovedJoints();
    }

    /**
     * The `i`-th moved impulse joint.
     */
    public movedJoint(i: number): number {
        return this.raw.movedJoint(i);
    }

    /**
     * The proxy the `i`-th moved joint was attached to before the tear.
     */
    public movedJointFrom(i: number): RigidBodyHandle {
        return this.raw.movedJointFrom(i);
    }

    /**
     * The proxy the `i`-th moved joint is attached to after the tear.
     */
    public movedJointTo(i: number): RigidBodyHandle {
        return this.raw.movedJointTo(i);
    }

    /**
     * Where a particle of the torn body is after the tear: its soft body and index there, or
     * `null` if the particle was removed.
     */
    public particleDestination(
        particle: number,
    ): {softBody: SoftBodyHandle; particle: number} | null {
        let body = this.raw.particleDestinationBody(particle);
        let index = this.raw.particleDestinationIndex(particle);
        if (body === undefined || index === undefined) {
            return null;
        }
        return {softBody: body, particle: index};
    }
}
