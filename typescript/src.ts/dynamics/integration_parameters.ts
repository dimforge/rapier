import {RawIntegrationParameters} from "../raw";

export class IntegrationParameters {
    raw: RawIntegrationParameters;

    constructor(raw?: RawIntegrationParameters) {
        this.raw = raw || new RawIntegrationParameters();
    }

    /**
     * Free the WASM memory used by these integration parameters.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    /**
     * The timestep length (default: `1.0 / 60.0`)
     */
    get dt(): number {
        return this.raw.dt;
    }

    /**
     * The Error Reduction Parameter in `[0, 1]` is the proportion of
     * the positional error to be corrected at each time step (default: `0.2`).
     */
    get contact_erp(): number {
        return this.raw.contact_erp;
    }

    get lengthUnit(): number {
        return this.raw.lengthUnit;
    }

    /**
     * Normalized amount of penetration the engine won’t attempt to correct (default: `0.001m`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedAllowedLinearError(): number {
        return this.raw.normalizedAllowedLinearError;
    }

    /**
     * The maximal normalized distance separating two objects that will generate predictive contacts (default: `0.002`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedPredictionDistance(): number {
        return this.raw.normalizedPredictionDistance;
    }

    /**
     * The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     */
    get numSolverIterations(): number {
        return this.raw.numSolverIterations;
    }

    /**
     * Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     */
    get numInternalPgsIterations(): number {
        return this.raw.numInternalPgsIterations;
    }

    /**
     * Minimum number of dynamic bodies in each active island (default: `128`).
     */
    get minIslandSize(): number {
        return this.raw.minIslandSize;
    }

    /**
     * Maximum number of substeps performed by the  solver (default: `1`).
     */
    get maxCcdSubsteps(): number {
        return this.raw.maxCcdSubsteps;
    }

    set dt(value: number) {
        this.raw.dt = value;
    }

    set contact_natural_frequency(value: number) {
        this.raw.contact_natural_frequency = value;
    }

    set lengthUnit(value: number) {
        this.raw.lengthUnit = value;
    }

    set normalizedAllowedLinearError(value: number) {
        this.raw.normalizedAllowedLinearError = value;
    }

    set normalizedPredictionDistance(value: number) {
        this.raw.normalizedPredictionDistance = value;
    }

    /**
     * Sets the number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     */
    set numSolverIterations(value: number) {
        this.raw.numSolverIterations = value;
    }

    /**
     * Sets the number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     */
    set numInternalPgsIterations(value: number) {
        this.raw.numInternalPgsIterations = value;
    }

    set minIslandSize(value: number) {
        this.raw.minIslandSize = value;
    }

    set maxCcdSubsteps(value: number) {
        this.raw.maxCcdSubsteps = value;
    }
}
