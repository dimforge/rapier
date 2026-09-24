/* Port of examples3d/convex_decomposition3.rs. */
#include "testbed.h"
void dynamicTrimesh3RunImpl(Testbed *testbed, int useConvexDecomposition);

void tbConvexDecomposition3(Testbed *testbed) {
    dynamicTrimesh3RunImpl(testbed, 1);
}
