using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Jolt
{
    public static unsafe partial class UnsafeBindings
    {
        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Init();

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Shutdown();

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_PhysicsSystem* JPH_PhysicsSystem_Create([NativeTypeName("uint32_t")] uint maxBodies, [NativeTypeName("uint32_t")] uint numBodyMutexes, [NativeTypeName("uint32_t")] uint maxBodyPairs, [NativeTypeName("uint32_t")] uint maxContactConstraints);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_PhysicsSystem* JPH_PhysicsSystem_CreateConfigured([NativeTypeName("uint32_t")] uint maxBodies, [NativeTypeName("uint32_t")] uint numBodyMutexes, [NativeTypeName("uint32_t")] uint maxBodyPairs, [NativeTypeName("uint32_t")] uint maxContactConstraints, [NativeTypeName("const uint64_t *")] ulong* collisionMasks, [NativeTypeName("uint32_t")] uint objectLayerCount);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_PhysicsSystem_Destroy(JPH_PhysicsSystem* system);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint32_t")]
        public static extern uint JPH_PhysicsSystem_Update(JPH_PhysicsSystem* system, float deltaTime, int collisionSteps);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_PhysicsSystem_OptimizeBroadPhase(JPH_PhysicsSystem* system);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_GetGravity([NativeTypeName("const JPH_PhysicsSystem *")] JPH_PhysicsSystem* system, [NativeTypeName("JPH_Vec3 *")] float3* gravity);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_ShouldCollide([NativeTypeName("const JPH_PhysicsSystem *")] JPH_PhysicsSystem* system, [NativeTypeName("uint32_t")] uint objectLayer1, [NativeTypeName("uint32_t")] uint objectLayer2);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_DrawDebugLines(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DebugDrawSettings *")] JPH_DebugDrawSettings* settings, JPH_DebugDrawLine* lines, [NativeTypeName("uint32_t")] uint lineCapacity, [NativeTypeName("uint32_t *")] uint* requiredLineCount);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_SyncWorldIn(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_WorldSyncIn *")] JPH_WorldSyncIn* sync);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_SyncWorldOut(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_WorldSyncOut *")] JPH_WorldSyncOut* sync);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CanonicalizeJoints([NativeTypeName("const JPH_PhysicsSystem *")] JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODWorldDefinition *")] JPH_DODWorldDefinition* authoredDefinition, JPH_DODBuffer* canonicalJoints);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint64_t")]
        public static extern ulong JPH_PhysicsSystem_GetFullStateFingerprint();

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_GetContinuationCounts(JPH_PhysicsSystem* system, JPH_DODContinuationCounts* counts);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CaptureGlobalContinuation(JPH_PhysicsSystem* system, JPH_DODGlobalContinuation* continuation);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CaptureBodyContinuations(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODWorldDefinition *")] JPH_DODWorldDefinition* definition, JPH_DODBuffer* bodies, [NativeTypeName("uint32_t *")] uint* actualCount);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CaptureContactContinuations(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODWorldDefinition *")] JPH_DODWorldDefinition* definition, JPH_DODBuffer* bodyPairs, JPH_DODBuffer* manifolds, JPH_DODBuffer* contactPoints, JPH_DODContinuationCounts* actualCounts);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CaptureConstraintContinuations(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODWorldDefinition *")] JPH_DODWorldDefinition* definition, JPH_DODBuffer* constraints, [NativeTypeName("uint32_t *")] uint* actualCount);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CaptureCharacterContinuations(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODWorldDefinition *")] JPH_DODWorldDefinition* definition, JPH_DODBuffer* characters, JPH_DODBuffer* characterContacts, JPH_DODContinuationCounts* actualCounts);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_FullStateSyncStatus JPH_PhysicsSystem_ValidateFullState([NativeTypeName("const JPH_PhysicsSystem *")] JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODFullStateIn *")] JPH_DODFullStateIn* fullState);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_FullStateSyncStatus JPH_PhysicsSystem_SyncFullStateIn(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_DODFullStateIn *")] JPH_DODFullStateIn* fullState);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_PhysicsSystem_ClearBodies(JPH_PhysicsSystem* system);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_HasBody(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_RemoveAndDestroyBody(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_SetBodyPositionAndRotation(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("JPH_Vec3")] float3 position, [NativeTypeName("JPH_Quat")] quaternion rotation, JPH_Activation activation);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_GetBodyPositionAndRotation(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("JPH_Vec3 *")] float3* position, [NativeTypeName("JPH_Quat *")] quaternion* rotation);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_SetBodyLinearAndAngularVelocity(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("JPH_Vec3")] float3 linearVelocity, [NativeTypeName("JPH_Vec3")] float3 angularVelocity);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_GetBodyLinearAndAngularVelocity(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("JPH_Vec3 *")] float3* linearVelocity, [NativeTypeName("JPH_Vec3 *")] float3* angularVelocity);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_AddBodyForce(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("const JPH_Vec3 *")] float3* force);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_AddBodyTorque(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("const JPH_Vec3 *")] float3* torque);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_AddBodyForceAndTorque(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("const JPH_Vec3 *")] float3* force, [NativeTypeName("const JPH_Vec3 *")] float3* torque);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_AddBodyImpulse(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("const JPH_Vec3 *")] float3* impulse);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_ActivateBody(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_DeactivateBody(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_IsBodyActive(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_RemoveAndDestroyConstraintByEntity(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CastRayEntity(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_RayCast *")] JPH_RayCast* ray, JPH_EntityRayCastResult* hit);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint32_t")]
        public static extern uint JPH_PhysicsSystem_CastRayAllEntities(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_RayCast *")] JPH_RayCast* ray, JPH_EntityRayCastResult* hits, [NativeTypeName("uint32_t")] uint maxHits);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_CreateCharacterVirtual(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_CharacterVirtualCreation *")] JPH_CharacterVirtualCreation* creation);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_DestroyCharacterVirtual(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_SetCharacterVirtualState(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, [NativeTypeName("const JPH_CharacterVirtualState *")] JPH_CharacterVirtualState* state, [NativeTypeName("uint8_t")] byte resetContacts);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_GetCharacterVirtualState(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, JPH_CharacterVirtualState* state);

        [DllImport("joltc", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_ExtendedUpdateCharacterVirtual(JPH_PhysicsSystem* system, [NativeTypeName("uint64_t")] ulong entityID, float deltaTime, [NativeTypeName("JPH_Vec3")] float3 gravity, [NativeTypeName("const JPH_CharacterVirtualUpdateSettings *")] JPH_CharacterVirtualUpdateSettings* settings, [NativeTypeName("uint64_t")] ulong collisionLayerMask);
    }
}
