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

    public unsafe partial struct JPH_CharacterVirtual
    {
        public int _unused;
    }

    public unsafe partial struct JPH_CharacterVirtualState
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

    public unsafe partial struct JPH_StateRecorderFilter
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Constraint
    {
        public int _unused;
    }

    public unsafe partial struct JPH_BroadPhaseLayerFilter
    {
        public int _unused;
    }

    public unsafe partial struct JPH_ObjectLayerFilter
    {
        public int _unused;
    }

    public unsafe partial struct JPH_BodyFilter
    {
        public int _unused;
    }

    public unsafe partial struct JPH_ShapeFilter
    {
        public int _unused;
    }

    public unsafe partial struct JPH_BlobBuilder
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

    public enum JPH_OverrideMassProperties
    {
        CalculateMassAndInertia = 0,
        CalculateInertia = 1,
        MassAndInertiaProvided = 2,
    }

    public enum JPH_BackFaceMode
    {
        IgnoreBackFaces = 0,
        CollideWithBackFaces = 1,
    }

    public enum JPH_GroundState
    {
        OnGround = 0,
        OnSteepGround = 1,
        NotSupported = 2,
        InAir = 3,
    }

    public enum JPH_CollisionCollectorType
    {
        ClosestHit = 0,
        AllHit = 1,
        AnyHit = 2,
    }

    public enum JPH_Activation
    {
        Activate = 0,
        DontActivate = 1,
    }

    public enum JPH_ConstraintKind : byte
    {
        Fixed = 1,
        Distance = 2,
    }

    public enum JPH_ConstraintSpace : byte
    {
        LocalToBodyCOM = 0,
        WorldSpace = 1,
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
    public struct JPH_ConstraintCreationSettings
    {
        public byte kind;
        public byte enabled;
        public byte space;
        public byte autoDetectPoint;
        public uint priority;
        public uint numVelocityStepsOverride;
        public uint numPositionStepsOverride;
        public float minDistance;
        public float maxDistance;
        public ulong userData;
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
        public uint subShapeID2;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_RayCastSettings
    {
        public JPH_BackFaceMode backFaceModeTriangles;
        public JPH_BackFaceMode backFaceModeConvex;
        public byte treatConvexAsSolid;
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
