using System;
using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Jolt
{
    public static unsafe partial class UnsafeBindings
    {
        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Init();

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Shutdown();

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_PhysicsSystem* JPH_PhysicsSystem_Create([NativeTypeName("uint32_t")] uint maxBodies, [NativeTypeName("uint32_t")] uint numBodyMutexes, [NativeTypeName("uint32_t")] uint maxBodyPairs, [NativeTypeName("uint32_t")] uint maxContactConstraints);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_PhysicsSystem_Destroy(JPH_PhysicsSystem* system);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint32_t")]
        public static extern uint JPH_PhysicsSystem_Update(JPH_PhysicsSystem* system, float deltaTime, int collisionSteps);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_BodyInterface* JPH_PhysicsSystem_GetBodyInterface(JPH_PhysicsSystem* system);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_NarrowPhaseQuery* JPH_PhysicsSystem_GetNarrowPhaseQueryNoLock(JPH_PhysicsSystem* system);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_Shape* JPH_Shape_CreateSphere(float radius);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_Shape* JPH_Shape_CreateBox([NativeTypeName("JPH_Vec3")] float3 halfExtent, float convexRadius);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_Shape* JPH_Shape_CreateCapsule(float halfHeightOfCylinder, float radius);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Shape_AddRef([NativeTypeName("const JPH_Shape *")] JPH_Shape* shape);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_Shape_Release([NativeTypeName("const JPH_Shape *")] JPH_Shape* shape);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("JPH_BodyID")]
        public static extern uint JPH_BodyInterface_CreateAndAddBody(JPH_BodyInterface* bodyInterface, [NativeTypeName("const JPH_BodyCreationSettings *")] JPH_BodyCreationSettings* settings, JPH_Activation activation);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_BodyInterface_RemoveAndDestroyBody(JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID, [NativeTypeName("JPH_Vec3")] float3 position, [NativeTypeName("JPH_Quat")] quaternion rotation, JPH_Activation activation);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_BodyInterface_GetPositionAndRotation([NativeTypeName("const JPH_BodyInterface *")] JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID, [NativeTypeName("JPH_Vec3 *")] float3* position, [NativeTypeName("JPH_Quat *")] quaternion* rotation);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_BodyInterface_SetLinearAndAngularVelocity(JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID, [NativeTypeName("JPH_Vec3")] float3 linearVelocity, [NativeTypeName("JPH_Vec3")] float3 angularVelocity);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_BodyInterface_GetLinearAndAngularVelocity([NativeTypeName("const JPH_BodyInterface *")] JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID, [NativeTypeName("JPH_Vec3 *")] float3* linearVelocity, [NativeTypeName("JPH_Vec3 *")] float3* angularVelocity);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_BodyInterface_IsAdded([NativeTypeName("const JPH_BodyInterface *")] JPH_BodyInterface* bodyInterface, [NativeTypeName("JPH_BodyID")] uint bodyID);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_Constraint* JPH_PhysicsSystem_CreateAndAddConstraint(JPH_PhysicsSystem* system, [NativeTypeName("JPH_BodyID")] uint bodyID1, [NativeTypeName("JPH_BodyID")] uint bodyID2, [NativeTypeName("const JPH_ConstraintCreationSettings *")] JPH_ConstraintCreationSettings* settings);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_RemoveAndDestroyConstraint(JPH_PhysicsSystem* system, JPH_Constraint* constraint);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_NarrowPhaseQuery_CastRay([NativeTypeName("const JPH_NarrowPhaseQuery *")] JPH_NarrowPhaseQuery* query, [NativeTypeName("const JPH_RayCast *")] JPH_RayCast* ray, JPH_RayCastResult* hit);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint32_t")]
        public static extern uint JPH_NarrowPhaseQuery_CastRayAll([NativeTypeName("const JPH_NarrowPhaseQuery *")] JPH_NarrowPhaseQuery* query, [NativeTypeName("const JPH_RayCast *")] JPH_RayCast* ray, JPH_RayCastResult* hits, [NativeTypeName("uint32_t")] uint maxHits);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_PhysicsSystemState* JPH_PhysicsSystem_SaveAlignedState(JPH_PhysicsSystem* system);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("uint8_t")]
        public static extern byte JPH_PhysicsSystem_RestoreAlignedState(JPH_PhysicsSystem* system, [NativeTypeName("const JPH_PhysicsSystemState *")] JPH_PhysicsSystemState* state);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("size_t")]
        public static extern UIntPtr JPH_PhysicsSystemState_GetSize([NativeTypeName("const JPH_PhysicsSystemState *")] JPH_PhysicsSystemState* state);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        [return: NativeTypeName("const uint8_t *")]
        public static extern byte* JPH_PhysicsSystemState_GetData([NativeTypeName("const JPH_PhysicsSystemState *")] JPH_PhysicsSystemState* state);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern JPH_PhysicsSystemState* JPH_PhysicsSystemState_CreateFromData([NativeTypeName("const uint8_t *")] byte* data, [NativeTypeName("size_t")] UIntPtr size);

        [DllImport("joltcd", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern void JPH_PhysicsSystemState_Destroy(JPH_PhysicsSystemState* state);
    }
}
