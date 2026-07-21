using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Jolt
{
    public unsafe partial struct JPH_PhysicsSystem
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
    public struct JPH_DODContinuationCounts
    {
        public uint bodyContinuationCount;
        public uint constraintContinuationCount;
        public uint characterContinuationCount;
        public uint characterContactContinuationCount;
        public uint bodyPairContinuationCount;
        public uint manifoldContinuationCount;
        public uint contactPointContinuationCount;
        public uint reserved0;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODGlobalContinuation
    {
        public uint version;
        public float previousStepDeltaTime;
        public float3 gravity;
        public uint reserved0;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODContinuationSleepSphere
    {
        public float3 center;
        public float radius;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODRigidTransform
    {
        public quaternion rotation;
        public float3 position;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODBodyContinuation
    {
        public ulong entityID;
        public ulong topologyHash;
        public float3 accumulatedForce;
        public float3 accumulatedTorque;
        public float3 sleepTestOffset;
        public JPH_DODContinuationSleepSphere sleepTestSphere0;
        public JPH_DODContinuationSleepSphere sleepTestSphere1;
        public JPH_DODContinuationSleepSphere sleepTestSphere2;
        public float sleepTestTimer;
        public uint activeIndex;
        public byte active;
        public byte reserved0;
        public byte reserved1;
        public byte reserved2;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODConstraintContinuation
    {
        public ulong entityID;
        public ulong topologyHash;
        public float3 positionLambda;
        public float3 rotationLambda;
        public float3 worldSpaceNormal;
        public float scalarLambda;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODCharacterContinuation
    {
        public ulong entityID;
        public ulong topologyHash;
        public ulong groundEntityID;
        public JPH_DODRigidTransform worldFromCharacter;
        public float3 linearVelocity;
        public float3 groundPosition;
        public float3 groundNormal;
        public float3 groundVelocity;
        public float lastDeltaTime;
        public uint groundSubShapeID;
        public uint firstContact;
        public uint contactCount;
        public byte groundState;
        public byte maxHitsExceeded;
        public ushort reserved0;
    }

    [StructLayout(LayoutKind.Sequential, Size = 88)]
    public struct JPH_DODCharacterContactContinuation
    {
        public ulong bodyEntityID;
        public ulong characterEntityID;
        public float3 position;
        public float3 linearVelocity;
        public float3 contactNormal;
        public float3 surfaceNormal;
        public float distance;
        public float fraction;
        public uint subShapeID;
        public byte motionType;
        public byte isSensor;
        public byte hadCollision;
        public byte wasDiscarded;
        public byte canPushCharacter;
        public byte reserved0;
        public byte reserved1;
        public byte reserved2;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODBodyPairContinuation
    {
        public ulong entityA;
        public ulong entityB;
        public float3 deltaPosition;
        public float3 deltaRotation;
        public uint firstManifold;
        public uint manifoldCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODManifoldContinuation
    {
        public ulong entity1;
        public ulong entity2;
        public uint subShapeID1;
        public uint subShapeID2;
        public float3 contactNormal;
        public uint firstContactPoint;
        public ushort contactPointCount;
        public byte isCCD;
        public byte reserved0;
        public uint reserved1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODContactPointContinuation
    {
        public float3 position1;
        public float3 position2;
        public float nonPenetrationLambda;
        public float frictionLambda1;
        public float frictionLambda2;
    }

    public enum JPH_FullStateSyncStatus
    {
        Success = 0,
        InvalidArgument = 1,
        InvalidLayout = 2,
        NonCanonicalOrder = 3,
        CapacityExceeded = 4,
        TopologyMismatch = 5,
        GlobalFailed = 6,
        BodiesFailed = 7,
        BroadPhaseFailed = 8,
        ContactsFailed = 9,
        ConstraintsFailed = 10,
        CharactersFailed = 11,
        InternalError = 12,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DODCharacterDefinition
    {
        public ulong entityID;
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
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_DODWorldDefinition
    {
        public JPH_DODBuffer rigidBodies;
        public JPH_DODBuffer motionDatas;
        public JPH_DODBuffer motionVelocities;
        public JPH_DODBuffer joints;
        public JPH_DODBuffer characterDefinitions;
        public uint secondaryBodyCount;
        public uint dynamicBodyCount;
        public uint staticBodyCount;
        public uint reserved0;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_DODContinuationState
    {
        public JPH_DODGlobalContinuation global;
        public JPH_DODBuffer bodies;
        public JPH_DODBuffer constraints;
        public JPH_DODBuffer characters;
        public JPH_DODBuffer characterContacts;
        public JPH_DODBuffer bodyPairs;
        public JPH_DODBuffer manifolds;
        public JPH_DODBuffer contactPoints;
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_DODFullStateIn
    {
        public JPH_DODWorldDefinition definition;
        public JPH_DODContinuationState continuation;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_EntityRayCastResult
    {
        public ulong entityID;
        public float fraction;
        public uint subShapeID2;
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
