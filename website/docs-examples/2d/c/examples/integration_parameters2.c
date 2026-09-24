#include "snippets.h"

int main(void) {
    snippets_init();
    R2World *world = r2NewWorld();

    // DOCUSAURUS: IntegrationParameters start
    /* Copy the integration parameters of the world, modify them, then apply them. */
    R2IntegrationParameters params = r2IntegrationParameters(world);
    params.numSolverIterations = 12;
    params.warmstartJoints = 1;
    r2SetIntegrationParameters(world, &params);
    // DOCUSAURUS: IntegrationParameters stop

    // DOCUSAURUS: Setters start
    /* Each parameter can also be read or modified on its own. */
    r2SetTimeStep(world, 1.0 / 120.0);
    r2SetNumSolverIterations(world, 8);
    r2SetWarmstartJoints(world, 1);
    printf("dt = %f, %zu solver iterations\n", (double)r2TimeStep(world), r2NumSolverIterations(world));
    // DOCUSAURUS: Setters stop

    // DOCUSAURUS: DefaultParameters start
    /* The default values, e.g., to reset a world to its initial settings. */
    R2IntegrationParameters defaults = r2DefaultIntegrationParameters();
    r2SetIntegrationParameters(world, &defaults);
    // DOCUSAURUS: DefaultParameters stop

    // DOCUSAURUS: LengthUnit start
    /* The simulation is measured in centimeters. */
    r2SetLengthUnit(world, 100.0);
    /* The gravity isn't scaled by the length unit: it must be given in centimeters too. */
    r2SetGravity(world, r2Vector(0.0, -981.0));
    // DOCUSAURUS: LengthUnit stop

    // DOCUSAURUS: ContactSoftness start
    /* Contacts against fixed rigid-bodies are stiffer by default: make them as soft as the others. */
    R2SpringCoefficients softness = r2ContactSoftness(world);
    printf("%f Hz, damping ratio %f\n", (double)softness.natural_frequency, (double)softness.damping_ratio);
    r2SetStaticContactSoftness(world, softness);
    // DOCUSAURUS: ContactSoftness stop

    r2Step(world, NULL, NULL);
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
