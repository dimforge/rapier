#include "snippets.h"

int main(void) {
    snippets_init();
    R3World *world = r3NewWorld();

    // DOCUSAURUS: IntegrationParameters start
    /* Copy the integration parameters of the world, modify them, then apply them. */
    R3IntegrationParameters params = r3IntegrationParameters(world);
    params.numSolverIterations = 12;
    params.warmstartJoints = 1;
    r3SetIntegrationParameters(world, &params);
    // DOCUSAURUS: IntegrationParameters stop

    // DOCUSAURUS: Setters start
    /* Each parameter can also be read or modified on its own. */
    r3SetTimeStep(world, 1.0 / 120.0);
    r3SetNumSolverIterations(world, 8);
    r3SetWarmstartJoints(world, 1);
    printf("dt = %f, %zu solver iterations\n", (double)r3TimeStep(world), r3NumSolverIterations(world));
    // DOCUSAURUS: Setters stop

    // DOCUSAURUS: DefaultParameters start
    /* The default values, e.g., to reset a world to its initial settings. */
    R3IntegrationParameters defaults = r3DefaultIntegrationParameters();
    r3SetIntegrationParameters(world, &defaults);
    // DOCUSAURUS: DefaultParameters stop

    // DOCUSAURUS: LengthUnit start
    /* The simulation is measured in centimeters. */
    r3SetLengthUnit(world, 100.0);
    /* The gravity isn't scaled by the length unit: it must be given in centimeters too. */
    r3SetGravity(world, r3Vector(0.0, -981.0, 0.0));
    // DOCUSAURUS: LengthUnit stop

    // DOCUSAURUS: ContactSoftness start
    /* Contacts against fixed rigid-bodies are stiffer by default: make them as soft as the others. */
    R3SpringCoefficients softness = r3ContactSoftness(world);
    printf("%f Hz, damping ratio %f\n", (double)softness.natural_frequency, (double)softness.damping_ratio);
    r3SetStaticContactSoftness(world, softness);
    // DOCUSAURUS: ContactSoftness stop

    // DOCUSAURUS: FrictionModel start
    /* Solve one Coulomb friction constraint per contact point. */
    r3SetFrictionModel(world, R3_FRICTION_MODEL_COULOMB);
    // DOCUSAURUS: FrictionModel stop

    r3Step(world, NULL, NULL);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
