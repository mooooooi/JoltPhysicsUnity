using System;
using System.Runtime.InteropServices;
using Higo.Gameplay.Runtime;
using Unity.Collections;
using Unity.Mathematics;
using LowLevel = Jolt.LowLevel;

namespace Jolt
{
    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_PhysicsSystemSettings
    {
        public uint maxBodies;
        public uint numBodyMutexes;
        public uint maxBodyPairs;
        public uint maxContactConstraints;
        public void* broadPhaseLayerInterface;
        public void* objectLayerPairFilter;
        public void* objectVsBroadPhaseLayerFilter;
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.BodyId.")]
    public unsafe partial struct Body
    {
        internal JPH_Body* Ptr;
        private uint bodyId;

        public uint GetID() => bodyId;
        public MotionProperties GetMotionProperties() => default;
        public void GetPosition(rvec3* position)
        {
            if (position != null)
                *position = rvec3.zero;
        }

        public void GetRotation(quaternion* rotation)
        {
            if (rotation != null)
                *rotation = quaternion.identity;
        }

        public bool IsKinematic() => false;
        public void SetMotionType(JPH_MotionType motionType) { }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_ExtendedUpdateSettings
    {
        public float3 stickToFloorStepDown;
        public float3 walkStairsStepUp;
        public float walkStairsMinStepForward;
        public float walkStairsStepForwardTest;
        public float walkStairsCosAngleForwardContact;
        public float3 walkStairsStepDownExtra;
    }

    [Obsolete("Legacy compatibility shell. CharacterVirtual is disabled for the minimal glue path.")]
    public unsafe partial struct CharacterVirtual
    {
        internal JPH_CharacterVirtual* Ptr;

        public JPH_CharacterVirtual* ToUnsafePtr() => Ptr;

        public void Destroy() { }
        public void SaveAlignedState(void* builder) { }
        public void RestoreAlignedState(void* payload, uint size) { }
        public void SetPosition(rvec3 position) { }
        public void SetPosition(float3 position) { }
        public void GetPosition(rvec3* position)
        {
            if (position != null)
                *position = rvec3.zero;
        }

        public void SetRotation(quaternion rotation) { }
        public void GetRotation(quaternion* rotation)
        {
            if (rotation != null)
                *rotation = quaternion.identity;
        }

        public void SetLinearVelocity(float3 velocity) { }
        public void GetLinearVelocity(float3* velocity)
        {
            if (velocity != null)
                *velocity = default;
        }

        public void GetGroundNormal(float3* normal)
        {
            if (normal != null)
                *normal = math.up();
        }

        public void GetGroundVelocity(float3* velocity)
        {
            if (velocity != null)
                *velocity = default;
        }

        public JPH_GroundState GetGroundState() => JPH_GroundState.InAir;
        public bool IsSupported() => false;

        public void ExtendedUpdate(
            float deltaTime,
            JPH_ExtendedUpdateSettings* settings,
            uint objectLayer,
            JPH_PhysicsSystem* physicsSystem,
            JPH_BodyFilter* bodyFilter,
            JPH_ShapeFilter* shapeFilter)
        {
        }
    }

    [Obsolete("Legacy compatibility shell. New code should use explicit motion data.")]
    public struct MotionProperties
    {
        public float GetInverseMassUnchecked() => 1f;
        public void SetInverseMass(float value) { }
        public float GetLinearDamping() => 0f;
        public void SetLinearDamping(float value) { }
        public float GetAngularDamping() => 0f;
        public void SetAngularDamping(float value) { }
        public void SetInverseInertia(float3 diagonal, quaternion rotation) { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe partial struct PhysicsSystem
    {
        internal JPH_PhysicsSystem* Ptr;

        public bool IsCreated => Ptr != null;
        public JPH_PhysicsSystem* ToUnsafePtr() => Ptr;

        public static PhysicsSystem Create(uint maxBodies, uint numBodyMutexes, uint maxBodyPairs, uint maxContactConstraints)
        {
            JoltCore.Init();
            return new PhysicsSystem
            {
                Ptr = UnsafeBindings.JPH_PhysicsSystem_Create(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints)
            };
        }

        public BodyInterface GetBodyInterface()
        {
            return new BodyInterface
            {
                Ptr = Ptr != null ? UnsafeBindings.JPH_PhysicsSystem_GetBodyInterface(Ptr) : null
            };
        }

        public void Destroy()
        {
            if (Ptr != null)
            {
                UnsafeBindings.JPH_PhysicsSystem_Destroy(Ptr);
                Ptr = null;
            }
        }

        public uint Update(float deltaTime, int collisionSteps)
        {
            return Ptr != null ? UnsafeBindings.JPH_PhysicsSystem_Update(Ptr, deltaTime, collisionSteps) : 0xffffffffu;
        }

        public float3 GetGravity() => new float3(0f, -9.81f, 0f);
        public void GetGravity(float3* gravity)
        {
            if (gravity != null)
                *gravity = GetGravity();
        }
        public uint Update(float deltaTime, int collisionSteps, void* jobSystem) => Update(deltaTime, collisionSteps);
        public bool RestoreAlignedState(void* data, void* filter) => false;
        public void SaveAlignedState(void* builder, JPH_StateRecorderState state, void* filter) { }
        public void DrawBodies(JPH_DrawSettings* settings, void* debugRenderer, void* bodyFilter) { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe partial struct BodyInterface
    {
        internal JPH_BodyInterface* Ptr;

        public JPH_BodyInterface* ToUnsafePtr() => Ptr;
        public void AddBody(uint bodyId, JPH_Activation activation) { }
        public void DeactivateBody(uint bodyId) { }
        public void RemoveBody(uint bodyId) { }
        public void DestroyBody(uint bodyId) { }
        public void MoveKinematic(uint bodyId, rvec3 targetPosition, quaternion targetRotation, float deltaTime) { }
        public float GetGravityFactor(uint bodyId) => 1f;
        public void SetGravityFactor(uint bodyId, float value) { }
    }

    public enum JPH_StateRecorderState : uint
    {
        None = 0,
        Global = 1 << 0,
        Bodies = 1 << 1,
        Contacts = 1 << 2,
        Constraints = 1 << 3,
        All = 0xffffffff,
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct JPH_CollisionGroup
    {
        public void* groupFilter;
        public uint groupID;
        public uint subGroupID;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_ObjectLayerFilter_Procs
    {
        public IntPtr ShouldCollide;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_BodyFilter_Procs
    {
        public IntPtr ShouldCollide;
        public IntPtr ShouldCollideLocked;
    }

    public static class JoltCore
    {
        public static void Init() => UnsafeBindings.JPH_Init();
        public static void Shutdown() => UnsafeBindings.JPH_Shutdown();
    }

    [Obsolete("Legacy compatibility shell. New code should serialize explicit sync output.")]
    public unsafe partial struct JPH_PhysicsSystemState :
        IPackedSerializable,
        IPackedDeltaSerializable<JPH_PhysicsSystemState>
    {
        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model) { }
        public void Serialize(ref JPH_PhysicsSystemState baseline, ref DataStreamWriter writer, in StreamCompressionModel model) { }
        public void Deserialize(ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(ref JPH_PhysicsSystemState baseline, ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(ref JPH_PhysicsSystemState baseline, NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model) { }
    }

    [Obsolete("Legacy compatibility shell. New code should serialize explicit sync output.")]
    public unsafe partial struct JPH_CharacterVirtualState :
        IPackedSerializable,
        IPackedDeltaSerializable<JPH_CharacterVirtualState>
    {
        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model) { }
        public void Serialize(ref JPH_CharacterVirtualState baseline, ref DataStreamWriter writer, in StreamCompressionModel model) { }
        public void Deserialize(ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(ref JPH_CharacterVirtualState baseline, ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model) { }
        public void Deserialize(ref JPH_CharacterVirtualState baseline, NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model) { }
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public unsafe delegate void JPH_CastRayResultCallback(void* context, JPH_RayCastResult* result);

    public static unsafe partial class UnsafeBindings
    {
        public static byte JPH_NarrowPhaseQuery_CastRay(
            JPH_NarrowPhaseQuery* query,
            rvec3* origin,
            float3* displacement,
            JPH_RayCastResult* hit,
            JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter,
            JPH_ObjectLayerFilter* objectLayerFilter,
            JPH_BodyFilter* bodyFilter)
        {
            return 0;
        }

        public static void JPH_NarrowPhaseQuery_CastRay3(
            JPH_NarrowPhaseQuery* query,
            rvec3* origin,
            float3* displacement,
            JPH_RayCastSettings* settings,
            JPH_CollisionCollectorType collectorType,
            IntPtr callback,
            void* context,
            JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter,
            JPH_ObjectLayerFilter* objectLayerFilter,
            JPH_BodyFilter* bodyFilter,
            JPH_ShapeFilter* shapeFilter)
        {
        }

        public static JPH_BlobBuilder* JPH_BlobBuilder_Create(uint initialCapacity) => null;
        public static void JPH_BlobBuilder_Destroy(JPH_BlobBuilder* builder) { }
        public static uint JPH_BlobBuilder_GetRequiredByteCount(JPH_BlobBuilder* builder) => 0;
        public static void JPH_BlobBuilder_Flush(JPH_BlobBuilder* builder, void* destination, uint size) { }
        public static void JPH_BlobBuilder_Reset(JPH_BlobBuilder* builder) { }

        public static void JPH_PhysicsSystem_OptimizeBroadPhase(JPH_PhysicsSystem* physicsSystem) { }
        public static void JPH_BodyInterface_InvalidateContactCache(JPH_BodyInterface* bodyInterface, uint bodyId) { }
        public static void JPH_BodyInterface_SetLinearVelocity(JPH_BodyInterface* bodyInterface, uint bodyId, float3* velocity) { }
        public static void JPH_BodyInterface_SetAngularVelocity(JPH_BodyInterface* bodyInterface, uint bodyId, float3* velocity) { }
        public static byte JPH_BodyInterface_SetLinearAndAngularVelocity(JPH_BodyInterface* bodyInterface, uint bodyId, float3* linearVelocity, float3* angularVelocity) => 0;
        public static byte JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface* bodyInterface, uint bodyId, rvec3* position, quaternion* rotation, JPH_Activation activation) => 0;
        public static void JPH_BodyInterface_MoveKinematic(JPH_BodyInterface* bodyInterface, uint bodyId, rvec3* targetPosition, quaternion* targetRotation, float deltaTime) { }
        public static void JPH_BodyInterface_RemoveBody(JPH_BodyInterface* bodyInterface, uint bodyId) { }
        public static void JPH_BodyInterface_DestroyBody(JPH_BodyInterface* bodyInterface, uint bodyId) { }

        public static void JPH_BodyInterface_AddBody(JPH_BodyInterface* bodyInterface, uint bodyId, JPH_Activation activation) { }
        public static void JPH_BodyInterface_SetObjectLayer(JPH_BodyInterface* bodyInterface, uint bodyId, uint objectLayer) { }
        public static void JPH_BodyInterface_SetMotionType(JPH_BodyInterface* bodyInterface, uint bodyId, JPH_MotionType motionType, JPH_Activation activation) { }
        public static void JPH_BodyInterface_SetMotionQuality(JPH_BodyInterface* bodyInterface, uint bodyId, JPH_MotionQuality motionQuality) { }
        public static void JPH_BodyInterface_SetCollisionGroup(JPH_BodyInterface* bodyInterface, uint bodyId, JPH_CollisionGroup* collisionGroup) { }
        public static void JPH_BodyInterface_SetUseManifoldReduction(JPH_BodyInterface* bodyInterface, uint bodyId, byte value) { }
        public static void JPH_BodyInterface_SetFriction(JPH_BodyInterface* bodyInterface, uint bodyId, float value) { }
        public static void JPH_BodyInterface_SetRestitution(JPH_BodyInterface* bodyInterface, uint bodyId, float value) { }
        public static void JPH_BodyInterface_SetGravityFactor(JPH_BodyInterface* bodyInterface, uint bodyId, float value) { }
        public static void JPH_BodyInterface_SetUserData(JPH_BodyInterface* bodyInterface, uint bodyId, ulong userData) { }
        public static void JPH_BodyInterface_SetPositionRotationAndVelocity(
            JPH_BodyInterface* bodyInterface,
            uint bodyId,
            rvec3* position,
            quaternion* rotation,
            float3* linearVelocity,
            float3* angularVelocity) { }
        public static void JPH_BodyInterface_GetPosition(JPH_BodyInterface* bodyInterface, uint bodyId, rvec3* position)
        {
            if (position != null)
                *position = rvec3.zero;
        }

        public static uint JPH_CharacterVirtual_GetInnerBodyID(JPH_CharacterVirtual* character) => 0xffffffffu;
        public static void JPH_CharacterVirtual_SaveAlignedState(JPH_CharacterVirtual* character, void* builder) { }
        public static byte JPH_CharacterVirtual_RestoreAlignedState(JPH_CharacterVirtual* character, void* payload, uint size) => 0;

        public static void JPH_PhysicsSystem_SaveAlignedState(
            JPH_PhysicsSystem* physicsSystem,
            JPH_BlobBuilder* builder,
            JPH_StateRecorderState state,
            JPH_StateRecorderFilter* filter)
        {
        }

        public static byte JPH_PhysicsSystem_RestoreAlignedState(
            JPH_PhysicsSystem* physicsSystem,
            void* payload,
            JPH_StateRecorderFilter* filter)
        {
            return 0;
        }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe struct JobSystem
    {
        public bool IsCreated => false;
        public void* ToUnsafePtr() => null;
        public void Destroy() { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe struct JobSystemThreadPoolConfig
    {
        public int _unused;
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public static class JobSystemThreadPool
    {
        public static unsafe JobSystem Create(JobSystemThreadPoolConfig* config) => default;
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe struct ObjectLayerPairFilter
    {
        public static ObjectLayerPairFilter Create(uint objectLayerCount) => default;
        public void* ToUnsafePtr() => null;
        public void EnableCollision(uint layer1, uint layer2) { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe struct BroadPhaseLayerInterface
    {
        public static BroadPhaseLayerInterface Create(int objectLayerCount, int broadPhaseLayerCount) => default;
        public void* ToUnsafePtr() => null;
        public void MapObjectToBroadPhaseLayer(uint objectLayer, byte broadPhaseLayer) { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Jolt.LowLevel.World.")]
    public unsafe struct ObjectVsBroadPhaseLayerFilter
    {
        public static ObjectVsBroadPhaseLayerFilter Create(void* broadPhaseLayerInterface, int broadPhaseLayerCount, void* objectLayerPairFilter, int objectLayerCount) => default;
        public void* ToUnsafePtr() => null;
    }

    [Obsolete("Legacy compatibility shell. New code should use the DOD registry SyncIn/SyncOut path.")]
    public unsafe struct StateRecorderImpl
    {
        public bool IsCreated => false;
        public void* ToUnsafePtr() => null;
        public static StateRecorderImpl Create() => default;
        public void Destroy() { }
        public uint GetDataSize() => 0;
        public void ReadBytes(void* destination, uint size) { }
        public void Clear() { }
    }

    [Obsolete("Legacy compatibility shell. New code should use the DOD registry SyncIn/SyncOut path.")]
    public unsafe struct StateRecorderFilter
    {
        public bool IsCreated => false;
        public void* ToUnsafePtr() => null;
        public static StateRecorderFilter Create(void* userData) => default;
        public void Destroy() { }
    }

    [Obsolete("Legacy compatibility shell. New code should use Higo.Gameplay.Runtime.Blobs.BlobBuilder.")]
    public unsafe struct BlobBuilder
    {
        public static BlobBuilder Create(int initialCapacity) => default;
        public void* ToUnsafePtr() => null;
        public uint GetRequiredByteCount() => 0;
        public void Flush(void* destination, uint size) { }
        public void Reset() { }
        public void Destroy() { }
    }

    public enum JPH_DebugRenderer_CastShadow
    {
        Off = 0,
        On = 1,
    }

    public enum JPH_BodyManager_ShapeColor
    {
        InstanceColor = 0,
        ShapeTypeColor = 1,
        MotionTypeColor = 2,
        SleepColor = 3,
        IslandColor = 4,
        MaterialColor = 5,
    }

    public enum JPH_SoftBodyConstraintColor
    {
        ConstraintType = 0,
        ConstraintGroup = 1,
        ConstraintOrder = 2,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DrawSettings
    {
        public byte drawGetSupportFunction;
        public byte drawSupportDirection;
        public byte drawGetSupportingFace;
        public byte drawShape;
        public byte drawShapeWireframe;
        public byte drawBoundingBox;
        public byte drawCenterOfMassTransform;
        public byte drawWorldTransform;
        public byte drawVelocity;
        public byte drawMassAndInertia;
        public byte drawSleepStats;
        public byte drawSoftBodyVertices;
        public byte drawSoftBodyVertexVelocities;
        public byte drawSoftBodyEdgeConstraints;
        public byte drawSoftBodyBendConstraints;
        public byte drawSoftBodyVolumeConstraints;
        public byte drawSoftBodySkinConstraints;
        public byte drawSoftBodyLRAConstraints;
        public byte drawSoftBodyPredictedBounds;
        public JPH_BodyManager_ShapeColor drawShapeColor;
        public JPH_SoftBodyConstraintColor drawSoftBodyConstraintColor;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_DebugRenderer_Procs
    {
        public IntPtr DrawLine;
        public IntPtr DrawTriangle;
        public IntPtr DrawText3D;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JPH_StateRecorderFilter_Procs
    {
        public IntPtr ShouldSaveBody;
        public IntPtr ShouldSaveConstraint;
        public IntPtr ShouldSaveContact;
        public IntPtr ShouldRestoreContact;
    }

    [Obsolete("Legacy compatibility shell. Debug rendering is disabled for the minimal glue path.")]
    public unsafe struct DebugRenderer
    {
        public bool IsCreated => false;
        public static DebugRenderer Create(void* userData) => default;
        public static void SetProcs(JPH_DebugRenderer_Procs* procs) { }
        public void* ToUnsafePtr() => null;
        public void NextFrame() { }
        public void SetCameraPos(float3 position) { }
        public void Destroy() { }
    }

    [Obsolete("Legacy compatibility shell. Debug rendering is disabled for the minimal glue path.")]
    public unsafe struct BodyDrawFilter
    {
        public bool IsCreated => false;
        public static BodyDrawFilter Create(void* userData) => default;
        public void* ToUnsafePtr() => null;
        public void Destroy() { }
    }
}
