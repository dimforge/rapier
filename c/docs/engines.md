# Engine and language integration

Keep one physics world per scene/simulation, and store generational handles in
engine components. Build shapes once and share them across colliders. Stepping
and entity creation/removal belong on an exclusive physics thread or behind the
engine's synchronization boundary. Copy transforms and events into engine-owned
buffers before exposing them to other threads; never persist borrowed body or
collider pointers across a frame.

## Unity / C#

`examples/RapierNative.cs` is a small executable 3D/f32 P/Invoke example. Native
structs use `LayoutKind.Sequential`, uint for Rapier booleans/enums, and pointer-
sized `UIntPtr` for `size_t`. Always declare `CallingConvention.Cdecl`. Use
`SafeHandle` for owned worlds; borrowed set/element pointers must not have owning
finalizers. Keep the owning SafeHandle alive while using borrowed pointers.

Build the matching native target and place the DLL/shared library in the project's
native plugin location for that platform/architecture. Configure Unity's plugin
importer accordingly. For iOS/static builds, an application may need `__Internal`
as its import library and platform-specific native-link setup. The provided
example does not perform platform packaging or editor configuration.

Call physics at a fixed timestep and set Rapier's `IntegrationParameters.dt` to
that interval. Synchronize dynamic body results into Transform objects after
stepping; author kinematic targets before stepping. Choose an explicit basis and
unit mapping. For a Unity mapping that reflects Z, convert positions with
`(x,y,-z)` and quaternion components with `(-x,-y,z,w)` in both directions. The
sample only uses vertical translation and does not impose an engine-wide mapping.
When rendering soft bodies, copy collision-mesh vertices and indices, and rebuild
topology when its version changes. Process tear piece/particle remaps to preserve
render attributes attached to particles.

Keep delegates rooted for the whole step when using hooks. Parallel builds can
invoke hooks on worker threads: do not access Unity objects from those callbacks.
Event polling after the step is usually simpler. A full generated C# wrapper can
be generated from `rapier.h`; this example intentionally declares only its used
entry points.

## Unreal / C++

Add the installed include directory, link the selected static or import library
from the module's Build.cs, and stage the shared runtime library through Unreal's
normal runtime dependency mechanism if dynamically linked. Define
`RAPIER_DIM3`, `RAPIER_F32` or `RAPIER_F64`, and `RAPIER_STATIC` for static linkage.
The C API does not require C++ exceptions; Unreal projects with exceptions disabled
can check statuses directly instead of calling the throwing `rapier::check` helper.

Use a subsystem/scene object to own the world. Store `R3RigidBodyHandle` and
`R3ColliderHandle` in components, and translate Rapier user data into engine IDs.
Do not store an engine UObject pointer in a physics object without an engine-side
lifetime strategy. Event collection allows game-thread dispatch without invoking
Unreal object APIs from solver callbacks.

Choose a unit and basis conversion consistently for positions, normals, forces,
velocities, rotations and inertia. Rapier defaults to meters and gravity along -Y;
Unreal commonly uses centimeters and Z-up. You can keep physics in meters and
convert at the boundary, or configure `length_unit` and gravity for your selected
units. A reflection changes angular-vector handedness as well as positions; use
a basis transform for rotations rather than merely swapping quaternion fields.

## Build/ABI checklist for wrapper authors

1. Select one dimension and scalar precision. Check `r3CheckAbi` before passing
   POD math structures; reject an incompatible runtime library.
2. Generate bindings from the preprocessed header with those definitions. Optional
   FEM and parallel declarations also require `RAPIER_FEM` / `RAPIER_PARALLEL`.
3. Preserve C layout, pointer width, constness, and the documented pointer lifetimes.
   Do not use a language's default bool marshaling for `R3Bool`.
4. Copy the thread-local error text before another API call. Treat query misses and
   buffer sizing distinctly from fatal errors.
5. Make owned/borrowed pointer distinctions visible in the wrapper. Prefer handles
   and reacquisition for long-lived engine components.
6. Convert coordinate systems and units in one shared layer. Cover rotations,
   angular velocities, inertia and soft meshes as well as translations.

The repository tests exercise the C ABI and C++ ownership helpers. Engine editor,
IL2CPP/AOT, consoles, mobile signing, and Unreal build integration need validation
in the target projects.
