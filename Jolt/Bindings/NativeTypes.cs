using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Jolt
{
    public unsafe partial struct JPH_PhysicsSystem
    {
        public int _unused;
    }

    public unsafe partial struct JPH_BodyInterface
    {
        public int _unused;
    }

    public unsafe partial struct JPH_NarrowPhaseQuery
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Body
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Shape
    {
        public int _unused;
    }

    public unsafe partial struct JPH_PhysicsSystemState
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Constraint
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Plane
    {
        public float3 normal;
        public float distance;
    }

    public enum JPH_MotionType
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2,
    }

    public enum JPH_MotionQuality
    {
        Discrete = 0,
        LinearCast = 1,
    }

    public enum JPH_Activation
    {
        Activate = 0,
        DontActivate = 1,
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_BodyCreationSettings
    {
        public JPH_Shape* shape;
        public float3 position;
        public quaternion rotation;
        public float3 linearVelocity;
        public float3 angularVelocity;
        public ulong userData;
        public uint objectLayer;
        public byte motionType;
        public byte motionQuality;
        public byte isSensor;
        public byte allowSleeping;
        public float linearDamping;
        public float angularDamping;
        public float maxLinearVelocity;
        public float maxAngularVelocity;
        public float gravityFactor;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_RayCast
    {
        public float3 origin;
        public float3 direction;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_RayCastResult
    {
        public uint bodyID;
        public float fraction;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_Triangle
    {
        public float3 v1;
        public float3 v2;
        public float3 v3;
        public uint materialIndex;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_IndexedTriangle
    {
        public uint i1;
        public uint i2;
        public uint i3;
        public uint materialIndex;
    }
}
