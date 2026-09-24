/* Port of examples3d/joints3.rs: run_impulse_joints. */
#include "testbed.h"
void joints3Run(Testbed *testbed, int useArticulations);

void tbJoints3RunImpulseJoints(Testbed *testbed) {
    joints3Run(testbed, 0);
}
