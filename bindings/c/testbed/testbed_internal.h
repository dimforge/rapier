#ifndef RAPIER_TESTBED_INTERNAL_H
#define RAPIER_TESTBED_INTERNAL_H
#include "testbed.h"
void tbCheck(Testbed *, RAPIER_TYPE(Status), const char *, const char *, int);
/* Checked viewer operations return to a C-only recovery boundary. */
#define TB(t, call) tbCheck((t), (call), #call, __FILE__, __LINE__)
#endif
