#ifndef RAPIER_TESTBED_GRAB_H
#define RAPIER_TESTBED_GRAB_H
#include "testbed.h"
#include <stdbool.h>

/* Mirrors src_testbed/grab.rs: a motor joint pulls a body or one soft particle. */
typedef struct TbGrab {
    bool active, soft;
    RAPIER_TYPE(RigidBodyHandle) body, mouseBody;
    RAPIER_TYPE(ImpulseJointHandle) joint;
    RAPIER_TYPE(Vector) planePoint;
} TbGrab;

RAPIER_TYPE(Status) tbGrabBegin(Testbed *, TbGrab *, RAPIER_TYPE(Real) pickRadius);
RAPIER_TYPE(Status) tbGrabUpdate(Testbed *, TbGrab *, RAPIER_TYPE(Vector) cameraForward);
RAPIER_TYPE(Status) tbGrabRelease(Testbed *, TbGrab *);
void tbGrabDrawCue(Testbed *, const TbGrab *);
#endif
