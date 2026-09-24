// Minimal standalone C# / Unity-compatible 3D-f32 example. Uses only blittable C ABI types.
using System;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class RapierNative
{
    const string Library = "rapier3d_ffi";
    [StructLayout(LayoutKind.Sequential)] public struct Vector { public float x, y, z; }
    [StructLayout(LayoutKind.Sequential)] public struct Rotation { public float x, y, z, w; }
    [StructLayout(LayoutKind.Sequential)] public struct Pose { public Vector translation; public Rotation rotation; }
    [StructLayout(LayoutKind.Sequential)] public struct BodyHandle { public IntPtr world; public uint index, generation; }
    [StructLayout(LayoutKind.Sequential)] public struct UserData { public ulong low, high; }
    [StructLayout(LayoutKind.Sequential)] public struct MassProperties
    {
        public Vector localCom;
        public float mass;
        public Vector principalInertia;
        public Rotation principalInertiaLocalFrame;
    }
    // R3Bool is uint32_t. Do not marshal these fields as C# bool.
    [StructLayout(LayoutKind.Sequential)] public struct RigidBodyDesc
    {
        public Pose position;
        public Vector linvel, angvel;
        public uint bodyType;
        public float gravityScale, linearDamping, angularDamping, additionalMass;
        public uint useAdditionalMassProperties;
        public MassProperties additionalMassProperties;
        public byte lockedAxes;
        public uint canSleep, sleeping, ccdEnabled;
        public float softCcdPrediction;
        public uint allowFastRotation, enabled;
        public sbyte dominanceGroup;
        public UIntPtr additionalSolverIterations, additionalPgsIterations;
        public uint gyroscopicForcesEnabled;
        public UserData userData;
    }
    [StructLayout(LayoutKind.Sequential)] struct PodLayout
    {
        public UIntPtr rigidBodyDesc, colliderDesc, shapeDesc, jointDesc;
        public UIntPtr softBodyMaterial, integrationParameters, softBodyDesc;
        public UIntPtr softMeshBindingDesc, queryOptions, urdfLoaderOptions, mjcfLoaderOptions;
    }
    public sealed class World : SafeHandleZeroOrMinusOneIsInvalid
    {
        public World() : base(true) { }
        protected override bool ReleaseHandle() { return r3FreeWorld(handle) == 0; }
    }
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern uint r3CheckAbi(uint version,uint dimension,UIntPtr realSize,UIntPtr vectorSize,UIntPtr poseSize,uint features);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern IntPtr r3LastError();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern uint r3LastStatus();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern World r3NewWorld();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern uint r3FreeWorld(IntPtr world);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern uint r3Step(World world,IntPtr hooks,IntPtr events);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern PodLayout r3PodLayout();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern RigidBodyDesc r3DynamicRigidBodyDesc();
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern BodyHandle r3InsertRigidBody(World world,in RigidBodyDesc desc);
    [DllImport(Library, CallingConvention=CallingConvention.Cdecl, ExactSpelling=true)] static extern Vector r3RigidBody_Translation(BodyHandle handle);
    static void Check(uint status)
    {
        if(status!=0) throw new InvalidOperationException(Marshal.PtrToStringUTF8(r3LastError()));
    }
    // Call from Unity's main thread, or any thread with exclusive access to this world.
    // The native library must be installed for the process architecture before invoking this method.
    public static Vector SimulateOneSecond()
    {
        Check(r3CheckAbi(1,3,(UIntPtr)4,(UIntPtr)Marshal.SizeOf<Vector>(),(UIntPtr)Marshal.SizeOf<Pose>(),0));
        PodLayout layout = r3PodLayout();
        if (layout.rigidBodyDesc != (UIntPtr)Marshal.SizeOf<RigidBodyDesc>())
            throw new InvalidOperationException("RigidBodyDesc layout does not match the native library.");
        World world = r3NewWorld();
        Check(r3LastStatus());
        using(world)
        {
            RigidBodyDesc body = r3DynamicRigidBodyDesc();
            body.additionalMass = 1;
            body.position.translation.y = 5;
            BodyHandle handle = r3InsertRigidBody(world,in body);
            Check(r3LastStatus());
            // The description is a value. Only the world needs disposal.
            for(int i=0;i<60;++i) Check(r3Step(world,IntPtr.Zero,IntPtr.Zero));
            Vector position = r3RigidBody_Translation(handle);
            // Handles contain a raw pointer; keep the owning SafeHandle alive through the call.
            GC.KeepAlive(world);
            Check(r3LastStatus());
            return position;
        }
    }
}
