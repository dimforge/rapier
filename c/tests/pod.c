#include "rapier.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define OK(call)                                                                                   \
    do {                                                                                           \
        RAPIER_TYPE(Status) status = (call);                                                       \
        if (status != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

static RAPIER_TYPE(Bool) RAPIER_CALL reject(void *data, const RAPIER_TYPE(ReadContext) *read,
                                            RAPIER_TYPE(ColliderHandle) handle) {
    (void)read;
    (void)handle;
    ++*(unsigned *)data;
    return 0;
}

static void test_world_and_deformable_bindings(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(RigidBodyHandle) a, b;
    RAPIER_TYPE(ColliderHandle) colliderHandle;
    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
    body.additionalMass = 1;
    a = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    body.position.translation.x = 2;
    b = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)(0.5);
    colliderHandle = RAPIER_FN(InsertCollider)(a, &collider);
    OK(RAPIER_FN(LastStatus)());
    collider.position.translation.y = -10;
    colliderHandle = RAPIER_FN(InsertColliderWithoutParent)(world, &collider);
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(ImpulseJointHandle) impulse;
    RAPIER_TYPE(MultibodyJointHandle) multi;
    RAPIER_TYPE(JointDesc) joint = RAPIER_FN(FixedJointDesc)();
    impulse = RAPIER_FN(InsertImpulseJoint)(a, b, &joint);
    OK(RAPIER_FN(LastStatus)());
    assert(impulse.index != UINT32_MAX);
    multi = RAPIER_FN(InsertMultibodyJoint)(a, b, &joint);
    OK(RAPIER_FN(LastStatus)());
    assert(multi.index != UINT32_MAX);

    RAPIER_TYPE(SoftBodyDesc) soft;
    RAPIER_TYPE(SoftBodyHandle) softHandle;
    RAPIER_TYPE(Vector) vertices[3] = {{0}, {0}, {0}};
    vertices[1].x = 1;
    vertices[2].y = 1;
    soft = RAPIER_FN(DefaultSoftBodyDesc)();
    soft.positions = (RAPIER_TYPE(VectorView)){vertices, 3};
    soft.collisionEnabled = 0;
    softHandle = RAPIER_FN(InsertSoftBody)(world, &soft);
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(RigidBodyHandle) root;
    OK(RAPIER_FN(SoftBody_ValidateHandle)(softHandle));
    root = RAPIER_FN(SoftBody_RootBody)(softHandle);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(SoftBodyMaterial) material = RAPIER_FN(SoftBody_Material)(softHandle);
    OK(RAPIER_FN(LastStatus)());
    material.edgeSoftness.natural_frequency = 45;
    material.tearStrain = (RAPIER_TYPE(OptionalReal)){1, 0.5};
    OK(RAPIER_FN(SoftBody_SetMaterial)(softHandle, &material));
    material = RAPIER_FN(SoftBody_Material)(softHandle);
    OK(RAPIER_FN(LastStatus)());
    assert(material.edgeSoftness.natural_frequency == 45 && material.tearStrain.enabled);

    collider = RAPIER_FN(DefaultColliderDesc)();
    collider.isSensor = 1;
    collider.shape.vertices = (RAPIER_TYPE(VectorView)){vertices, 3};
#ifdef RAPIER_DIM3
    uint32_t indices[] = {0, 1, 2};
    collider.shape.kind = RAPIER_CONST(SHAPE_DESC_TRIMESH);
    collider.shape.flags = RAPIER_CONST(TRIMESH_DEFORMABLE);
#else
    uint32_t indices[] = {0, 1, 1, 2};
    collider.shape.kind = RAPIER_CONST(SHAPE_DESC_POLYLINE);
    collider.shape.flags = RAPIER_CONST(POLYLINE_DEFORMABLE);
#endif
#ifdef RAPIER_DIM3
    collider.shape.triangles =
        (RAPIER_TYPE(TriangleView)){(const RAPIER_TYPE(Triangle) *)indices, 1};
#else
    collider.shape.edges = (RAPIER_TYPE(EdgeView)){(const RAPIER_TYPE(Edge) *)indices, 2};
#endif
    RAPIER_TYPE(SoftMeshBindingDesc) binding = RAPIER_FN(DefaultSoftMeshBindingDesc)();
    uint32_t particleIndices[] = {0, 1, 2};
    binding.kind = RAPIER_CONST(SOFT_BINDING_DIRECT);
    binding.particles = (RAPIER_TYPE(IndexView)){particleIndices, 3};
    colliderHandle = RAPIER_FN(InsertDeformableCollider)(&collider, &binding, root);
    OK(RAPIER_FN(LastStatus)());
    particleIndices[2] = 999;
    vertices[1].x = 100;
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    OK(RAPIER_FN(SoftBody_ValidateHandle)(softHandle));
    size_t count;
    RAPIER_TYPE(Vector) copied[3];
    count = RAPIER_FN(SoftBody_MeshVertices)(softHandle, colliderHandle, copied, 3);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 3 && copied[1].x < 2);
    OK(RAPIER_FN(FreeWorld)(world));
}

int main(void) {
    test_world_and_deformable_bindings();
    RAPIER_TYPE(PodLayout) layout = RAPIER_FN(PodLayout)();
    assert(layout.rigidBodyDesc == sizeof(RAPIER_TYPE(RigidBodyDesc)));
    assert(layout.colliderDesc == sizeof(RAPIER_TYPE(ColliderDesc)));
    assert(layout.shapeDesc == sizeof(RAPIER_TYPE(ShapeDesc)));
    assert(layout.jointDesc == sizeof(RAPIER_TYPE(JointDesc)));
    assert(layout.softBodyMaterial == sizeof(RAPIER_TYPE(SoftBodyMaterial)));
    assert(layout.integrationParameters == sizeof(RAPIER_TYPE(IntegrationParameters)));
    assert(layout.softBodyDesc == sizeof(RAPIER_TYPE(SoftBodyDesc)));
    assert(layout.softMeshBindingDesc == sizeof(RAPIER_TYPE(SoftMeshBindingDesc)));
    assert(layout.queryOptions == sizeof(RAPIER_TYPE(QueryOptions)));

    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)(0.5);
    body.canSleep = 0;
    body.position.translation.y = 5;
    body.userData.low = 42;
    RAPIER_TYPE(RigidBodyHandle) first, second;
    RAPIER_TYPE(ColliderHandle) firstCollider;
    first = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    firstCollider = RAPIER_FN(InsertCollider)(first, &collider);
    OK(RAPIER_FN(LastStatus)());
    body.position.translation.x = 2;
    second = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(second, &collider);
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(Vector) position;
    OK(RAPIER_FN(RigidBody_ValidateHandle)(first));
    position = RAPIER_FN(RigidBody_Translation)(first);
    OK(RAPIER_FN(LastStatus)());
    assert(position.x == 0 && position.y == 5);
    RAPIER_TYPE(UserData) userData = RAPIER_FN(RigidBody_UserData)(first);
    OK(RAPIER_FN(LastStatus)());
    assert(userData.low == 42);

    RAPIER_TYPE(JointDesc) joint = RAPIER_FN(SpringJointDesc)(2, 100, 1);
    RAPIER_TYPE(ImpulseJointHandle)
    jointHandle = RAPIER_FN(InsertImpulseJoint)(first, second, &joint);
    OK(RAPIER_FN(LastStatus)());
    assert(jointHandle.index != UINT32_MAX);
    joint.motors[0].stiffness = -1;
    RAPIER_FN(InsertImpulseJoint)(first, second, &joint);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_ARGUMENT));

    RAPIER_TYPE(IntegrationParameters) settings = RAPIER_FN(IntegrationParameters)(world);
    OK(RAPIER_FN(LastStatus)());
    settings.dt = (RAPIER_TYPE(Real))(1.0 / 120.0);
    OK(RAPIER_FN(SetIntegrationParameters)(world, &settings));
    settings.dt = -1;
    assert(RAPIER_FN(SetIntegrationParameters)(world, &settings) == RAPIER_CONST(INVALID_ARGUMENT));
    settings = RAPIER_FN(IntegrationParameters)(world);
    OK(RAPIER_FN(LastStatus)());
    assert(settings.dt == (RAPIER_TYPE(Real))(1.0 / 120.0));

    OK(RAPIER_FN(Step)(world, NULL, NULL));
    RAPIER_TYPE(QueryOptions) query = RAPIER_FN(DefaultQueryOptions)();
    RAPIER_TYPE(Vector) origin = {0}, direction = {0};
    origin.y = 10;
    direction.y = -1;
    RAPIER_TYPE(Bool) found = 0;
    RAPIER_TYPE(RayHit) hit;
    RAPIER_TYPE(OptionalRayHit)
    tryCastRayResult2 = RAPIER_FN(TryCastRay)(world, &query, origin, direction, 20, 1);
    hit = tryCastRayResult2.hit;
    found = tryCastRayResult2.found;
    OK(RAPIER_FN(LastStatus)());
    assert(found && hit.collider.index == firstCollider.index);
    unsigned calls = 0;
    query.predicate = reject;
    query.userData = &calls;
    RAPIER_TYPE(OptionalRayHit)
    tryCastRayResult3 = RAPIER_FN(TryCastRay)(world, &query, origin, direction, 20, 1);
    hit = tryCastRayResult3.hit;
    found = tryCastRayResult3.found;
    OK(RAPIER_FN(LastStatus)());
    assert(!found && calls);
    query.predicate = NULL;
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    RAPIER_TYPE(OptionalRayHit)
    tryCastRayResult4 = RAPIER_FN(TryCastRay)(world, &query, origin, direction, 20, 1);
    hit = tryCastRayResult4.hit;
    found = tryCastRayResult4.found;
    OK(RAPIER_FN(LastStatus)());
    assert(found);

    /* A copied description borrows arrays until insertion. The world then owns copies. */
    RAPIER_TYPE(SoftBodyDesc) soft = RAPIER_FN(DefaultSoftBodyDesc)();
    RAPIER_TYPE(Vector) particles[2] = {{0}, {0}};
    particles[0].y = particles[1].y = 10;
    particles[1].x = 1;
    RAPIER_TYPE(Edge) edges[] = {{0, 1}};
    soft.positions = (RAPIER_TYPE(VectorView)){particles, 2};
    soft.edges = (RAPIER_TYPE(EdgeView)){edges, 1};
    soft.collisionEnabled = 0;
    soft.canSleep = 0;
    soft.material.edgeSoftness.natural_frequency = 40;
    RAPIER_TYPE(SoftBodyHandle) softHandle = RAPIER_FN(InsertSoftBody)(world, &soft);
    OK(RAPIER_FN(LastStatus)());
    particles[1].x = 100;
    edges[0].b = 999;

    OK(RAPIER_FN(SoftBody_ValidateHandle)(softHandle));
    position = RAPIER_FN(SoftBody_ParticlePosition)(softHandle, 1);
    OK(RAPIER_FN(LastStatus)());
    assert(position.x == 1);
    RAPIER_TYPE(SoftBodyHandle) sentinel = {NULL, 123, 456};
    sentinel = RAPIER_FN(InsertSoftBody)(world, &soft);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_ARGUMENT));
    assert(sentinel.index == UINT32_MAX && sentinel.generation == UINT32_MAX);

    /* Retained shared shapes remain usable after the caller releases its reference. */
    RAPIER_TYPE(SharedShape) *shape = RAPIER_FN(BallSharedShape)(0.75);
    OK(RAPIER_FN(LastStatus)());
    collider = RAPIER_FN(DefaultColliderDesc)();
    collider.shape.kind = RAPIER_CONST(SHAPE_DESC_SHARED);
    collider.shape.sharedShape = shape;
    collider.position.translation.x = 10;
    RAPIER_FN(InsertColliderWithoutParent)(world, &collider);
    OK(RAPIER_FN(LastStatus)());
    OK(RAPIER_FN(FreeSharedShape)(shape));
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    origin.x = 10;
    RAPIER_TYPE(OptionalRayHit)
    tryCastRayResult5 = RAPIER_FN(TryCastRay)(world, &query, origin, direction, 20, 1);
    hit = tryCastRayResult5.hit;
    found = tryCastRayResult5.found;
    OK(RAPIER_FN(LastStatus)());
    assert(found && isfinite(hit.time_of_impact));
    /* The only remaining owner is world. POD values and borrowed views need no cleanup. */
    OK(RAPIER_FN(FreeWorld)(world));
    return 0;
}
