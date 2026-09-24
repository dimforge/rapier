/* Port of examples3d/joints3.rs: run_multibody_joints. */
#include "testbed.h"
void joints3Run(Testbed *testbed, int useArticulations);

void tbJoints3RunMultibodyJoints(Testbed *testbed) {
    joints3Run(testbed, 1);
}
