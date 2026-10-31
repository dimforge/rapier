#include "rapier.hpp"
#include <cassert>
#include <cstring>
#include <type_traits>

int main() {
    static_assert(std::is_standard_layout<RAPIER_TYPE(Pose)>::value, "POD pose");
    static_assert(sizeof(RAPIER_TYPE(RigidBodyHandle)) == sizeof(void *) + 2 * sizeof(uint32_t), "handle ABI");
    static_assert(std::is_trivially_copyable<RAPIER_TYPE(RigidBodyDesc)>::value, "POD body");
    static_assert(std::is_standard_layout<RAPIER_TYPE(ColliderDesc)>::value, "POD collider");
    static_assert(std::is_trivially_copyable<RAPIER_TYPE(SoftBodyDesc)>::value, "POD soft recipe");
    static_assert(std::is_trivially_copyable<RAPIER_TYPE(JointDesc)>::value, "POD joint");
    auto world = rapier::make_world();
    auto body = rapier::rigid_body();
    auto collider = rapier::ball(0.5);
    body.position.translation.y = 5;
    auto bodyHandle = RAPIER_FN(InsertRigidBody)(world.get(), &body);
    rapier::check(RAPIER_FN(LastStatus)());
    auto colliderHandle = RAPIER_FN(InsertCollider)(bodyHandle, &collider);
    rapier::check(RAPIER_FN(LastStatus)());
    auto query = rapier::queryOptions();
    assert(!query.predicate && !query.userData);
    auto moved = std::move(world);
    assert(!world && moved);
    rapier::check(RAPIER_FN(Step)(moved.get(), nullptr, nullptr));
    {
        rapier::Shape shape(RAPIER_FN(Collider_CloneShape)(colliderHandle));
        rapier::check(RAPIER_FN(LastStatus)());
        // The owned clone must remain usable after its source collider is removed.
        rapier::check(RAPIER_FN(RemoveCollider)(colliderHandle, 1));
        rapier::ShapeMesh mesh(RAPIER_FN(SharedShape_Tessellate)(shape.get(), 8));
        rapier::EventCollector events(RAPIER_FN(NewEventCollector)());
        rapier::KinematicCharacterController character(
            RAPIER_FN(NewKinematicCharacterController)());
        rapier::PidController pid(RAPIER_FN(NewPidController)());
        rapier::Snapshot snapshot(RAPIER_FN(SerializeWorld)(moved.get()));
        rapier::SoftBodyTearEvent tear;
        assert(shape && mesh && events && character && pid && snapshot);
        // Bootstrap a replica with a full delta; it acknowledges the delta's tick.
        rapier::check(RAPIER_FN(EnableChangeTracking)(moved.get()));
        rapier::WorldDelta full(RAPIER_FN(SerializeWorldDelta)(moved.get(), nullptr, 0));
        rapier::check(RAPIER_FN(LastStatus)());
        auto replica = rapier::make_world();
        auto fullDelta = RAPIER_FN(Bytes_Data)(full.delta.get());
        rapier::Bytes acked(
            RAPIER_FN(ApplyWorldDelta)(replica.get(), fullDelta.data, fullDelta.count));
        rapier::check(RAPIER_FN(LastStatus)());
        auto ackedTick = RAPIER_FN(Bytes_Data)(acked.get());
        auto fullTick = RAPIER_FN(Bytes_Data)(full.tick.get());
        assert(ackedTick.count == fullTick.count &&
               std::memcmp(ackedTick.data, fullTick.data, fullTick.count) == 0);
        // A replica restored from a snapshot declares the tick issued right before it.
        rapier::Bytes tick = rapier::change_tick(moved.get());
        rapier::Snapshot tracked(RAPIER_FN(SerializeWorld)(moved.get()));
        auto trackedBytes = RAPIER_FN(Bytes_Data)(tracked.get());
        rapier::World restored(
            RAPIER_FN(DeserializeWorld)(trackedBytes.data, trackedBytes.count));
        rapier::check(RAPIER_FN(LastStatus)());
        rapier::set_delta_baseline(restored.get(), tick);
        rapier::WorldDelta failed(RAPIER_FN(SerializeWorldDelta)(nullptr, nullptr, 0));
        assert(!failed.delta && !failed.tick &&
               RAPIER_FN(LastStatus)() == RAPIER_CONST(NULL_POINTER));
        auto transferred = std::move(mesh);
        assert(transferred && !mesh);
#if defined(RAPIER_DIM3)
        rapier::TriMeshData triangles(RAPIER_FN(SharedShape_ToTrimesh)(shape.get(), 8, 8));
        assert(triangles);
        auto chassisDesc = RAPIER_FN(DynamicRigidBodyDesc)();
        auto chassis = RAPIER_FN(InsertRigidBody)(moved.get(), &chassisDesc);
        rapier::DynamicRayCastVehicleController vehicle(
            RAPIER_FN(NewDynamicRayCastVehicleController)(chassis));
        assert(vehicle);
#endif
#if defined(RAPIER_ROBOTICS) && defined(RAPIER_DIM3) && defined(RAPIER_F32)
        static_assert(std::is_trivially_copyable<R3UrdfLoaderOptions>::value, "POD URDF options");
        static_assert(std::is_trivially_copyable<R3MjcfLoaderOptions>::value, "POD MJCF options");
        const auto layout = r3PodLayout();
        assert(layout.urdfLoaderOptions == sizeof(R3UrdfLoaderOptions));
        assert(layout.mjcfLoaderOptions == sizeof(R3MjcfLoaderOptions));
        const auto urdf = r3DefaultUrdfLoaderOptions();
        const auto mjcf = r3DefaultMjcfLoaderOptions();
        assert(urdf.rigidBodyBlueprint.enabled && mjcf.rigidBodyBlueprint.enabled);
        assert(urdf.colliderBlueprint.density == 0 && mjcf.colliderBlueprint.density == 0);
        rapier::UrdfRobot urdfRobot;
        rapier::UrdfRobotHandles urdfHandles;
        rapier::MjcfRobot mjcfRobot;
        rapier::MjcfRobotHandles mjcfHandles;
#endif
    }
    bool caught = false;
    try {
        RAPIER_FN(BallSharedShape)(-1);
        rapier::check(RAPIER_FN(LastStatus)());
    } catch (const std::runtime_error &) {
        caught = true;
    }
    assert(caught);
}
