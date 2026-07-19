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

    public unsafe partial struct JPH_PhysicsSystemState
    {
        public int _unused;
    }

    public unsafe partial struct JPH_Constraint
    {
        public int _unused;
    }

    [System.Flags]
    public enum JPH_DebugDrawBodyFlags : uint
    {
        None = 0,
        Shape = 1u << 0,
        BoundingBox = 1u << 1,
        CenterOfMassTransform = 1u << 2,
        WorldTransform = 1u << 3,
        Velocity = 1u << 4,
    }

    public enum JPH_DebugDrawShapeColor : uint
    {
        Instance = 0,
        ShapeType = 1,
        MotionType = 2,
        Sleep = 3,
        Island = 4,
        Material = 5,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DebugDrawSettings
    {
        public JPH_DebugDrawBodyFlags flags;
        public uint shapeColor;
        public float3 cameraPosition;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DebugDrawLine
    {
        public float3 from;
        public float3 to;
        public uint color;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_DODBuffer
    {
        public void* data;
        public uint stride;
        public uint count;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_WorldSyncIn
    {
        public JPH_DODBuffer rigidBodies;
        public JPH_DODBuffer motionDatas;
        public JPH_DODBuffer motionVelocities;
        public JPH_DODBuffer bodyEntries;
        public JPH_DODBuffer joints;
        public JPH_DODBuffer jointEntries;
        public JPH_DODBuffer jointHandlesByEntry;
        public uint dynamicBodyStartIndex;
        public uint dynamicBodyCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_WorldSyncOut
    {
        public JPH_DODBuffer rigidBodies;
        public JPH_DODBuffer motionDatas;
        public JPH_DODBuffer motionVelocities;
        public JPH_DODBuffer outputMotionDatas;
        public JPH_DODBuffer outputMotionVelocities;
        public uint dynamicBodyStartIndex;
        public uint bodyStartIndex;
        public uint outputStartIndex;
        public uint count;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_EntityRayCastResult
    {
        public ulong entityID;
        public float fraction;
        public uint subShapeID2;
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

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_ObjectLayerFilter_Procs
    {
        public System.IntPtr ShouldCollide;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_BodyFilter_Procs
    {
        public System.IntPtr ShouldCollide;
        public System.IntPtr ShouldCollideLocked;
    }

    public unsafe partial struct JPH_ShapeFilter
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

    public enum JPH_CharacterGroundState
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
    public struct JPH_ColliderBlob
    {
        public byte type;
        public byte flags;
        public ushort payloadSize;
        public uint version;
        public ulong payload0;
        public ulong payload1;
        public ulong payload2;
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
    public struct JPH_CharacterVirtualCreation
    {
        public float capsuleHalfHeightOfCylinder;
        public float capsuleRadius;
        public float3 up;
        public float3 shapeOffset;
        public float supportingVolumeConstant;
        public float maxSlopeAngle;
        public float mass;
        public float maxStrength;
        public float predictiveContactDistance;
        public uint maxCollisionIterations;
        public uint maxConstraintIterations;
        public float minTimeRemaining;
        public float collisionTolerance;
        public float characterPadding;
        public uint maxNumHits;
        public float hitReductionCosMaxAngle;
        public float penetrationRecoverySpeed;
        public uint characterID;
        public ulong userData;
        public JPH_CharacterVirtualState state;
    }

    [StructLayout(LayoutKind.Sequential)]
    public partial struct JPH_CharacterVirtualState
    {
        public float3 position;
        public quaternion rotation;
        public float3 linearVelocity;
        public float3 groundNormal;
        public float3 groundVelocity;
        public JPH_CharacterGroundState groundState;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_CharacterVirtualUpdateSettings
    {
        public float3 stickToFloorStepDown;
        public float3 walkStairsStepUp;
        public float walkStairsMinStepForward;
        public float walkStairsStepForwardTest;
        public float walkStairsCosAngleForwardContact;
        public float3 walkStairsStepDownExtra;
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
