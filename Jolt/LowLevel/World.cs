using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Jolt.LowLevel
{
    public readonly struct BodyId : IEquatable<BodyId>
    {
        public static readonly BodyId Invalid = default;

        public readonly ulong Value;

        public BodyId(ulong value)
        {
            Value = value;
        }

        public bool IsValid => Value != 0;

        public bool Equals(BodyId other)
        {
            return Value == other.Value;
        }

        public override bool Equals(object obj)
        {
            return obj is BodyId other && Equals(other);
        }

        public override int GetHashCode()
        {
            return Value.GetHashCode();
        }

        public override string ToString()
        {
            return IsValid ? Value.ToString() : "BodyId.Invalid";
        }
    }

    public enum MotionType : byte
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2,
    }

    public enum MotionQuality : byte
    {
        Discrete = 0,
        LinearCast = 1,
    }

    public enum ConstraintKind : byte
    {
        Fixed = 1,
        Distance = 2,
    }

    public enum ConstraintSpace : byte
    {
        LocalToBodyCOM = 0,
        WorldSpace = 1,
    }

    public readonly struct ConstraintId : IEquatable<ConstraintId>
    {
        public static readonly ConstraintId Invalid = default;
        public readonly ulong Value;

        public ConstraintId(ulong value) => Value = value;
        public bool IsValid => Value != 0;
        public bool Equals(ConstraintId other) => Value == other.Value;
        public override bool Equals(object obj) => obj is ConstraintId other && Equals(other);
        public override int GetHashCode() => Value.GetHashCode();
        public override string ToString() => IsValid ? Value.ToString() : "ConstraintId.Invalid";
    }

    public struct ConstraintCreation
    {
        public ConstraintKind Kind;
        public ConstraintSpace Space;
        public bool Enabled;
        public bool AutoDetectPoint;
        public uint Priority;
        public uint NumVelocityStepsOverride;
        public uint NumPositionStepsOverride;
        public float MinDistance;
        public float MaxDistance;
        public ulong UserData;

        public static ConstraintCreation Fixed(ulong userData = 0)
        {
            return new ConstraintCreation
            {
                Kind = ConstraintKind.Fixed,
                Space = ConstraintSpace.WorldSpace,
                Enabled = true,
                AutoDetectPoint = true,
                MinDistance = -1.0f,
                MaxDistance = -1.0f,
                UserData = userData,
            };
        }

        public static ConstraintCreation Distance(ulong userData = 0)
        {
            return new ConstraintCreation
            {
                Kind = ConstraintKind.Distance,
                Space = ConstraintSpace.WorldSpace,
                Enabled = true,
                MinDistance = -1.0f,
                MaxDistance = -1.0f,
                UserData = userData,
            };
        }
    }

    public readonly struct RaycastInput
    {
        public readonly float3 Origin;
        public readonly float3 Direction;

        public RaycastInput(float3 origin, float3 direction)
        {
            Origin = origin;
            Direction = direction;
        }
    }

    public readonly struct RaycastHit
    {
        public readonly BodyId BodyId;
        public readonly float Fraction;
        public readonly uint SubShapeId;

        public RaycastHit(BodyId bodyId, float fraction, uint subShapeId = 0)
        {
            BodyId = bodyId;
            Fraction = fraction;
            SubShapeId = subShapeId;
        }
    }

    public unsafe sealed class World : IDisposable
    {
        private JPH_PhysicsSystem* physicsSystem;
        private static bool s_CastRayAllUnavailable;
        private bool disposed;

        public World(uint maxBodies, uint numBodyMutexes = 0, uint maxBodyPairs = 65536, uint maxContactConstraints = 10240)
        {
            UnsafeBindings.JPH_Init();
            physicsSystem = UnsafeBindings.JPH_PhysicsSystem_Create(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints);
            if (physicsSystem == null)
            {
                UnsafeBindings.JPH_Shutdown();
                throw new InvalidOperationException("Failed to create Jolt physics system.");
            }

        }

        public bool IsCreated => physicsSystem != null;
        internal JPH_PhysicsSystem* PhysicsSystem => physicsSystem;

        public void RemoveAndDestroyBody(BodyId bodyId)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return;
            }

            UnsafeBindings.JPH_PhysicsSystem_RemoveAndDestroyBody(physicsSystem, bodyId.Value);
        }

        public bool SetPositionAndRotation(BodyId bodyId, float3 position, quaternion rotation, bool activate)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            return UnsafeBindings.JPH_PhysicsSystem_SetBodyPositionAndRotation(
                physicsSystem,
                bodyId.Value,
                position,
                rotation,
                activate ? JPH_Activation.Activate : JPH_Activation.DontActivate) != 0;
        }

        public bool GetPositionAndRotation(BodyId bodyId, out float3 position, out quaternion rotation)
        {
            position = default;
            rotation = quaternion.identity;

            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            fixed (float3* positionPtr = &position)
            fixed (quaternion* rotationPtr = &rotation)
            {
                return UnsafeBindings.JPH_PhysicsSystem_GetBodyPositionAndRotation(
                    physicsSystem,
                    bodyId.Value,
                    positionPtr,
                    rotationPtr) != 0;
            }
        }

        public bool SetVelocity(BodyId bodyId, float3 linearVelocity, float3 angularVelocity)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            return UnsafeBindings.JPH_PhysicsSystem_SetBodyLinearAndAngularVelocity(
                physicsSystem,
                bodyId.Value,
                linearVelocity,
                angularVelocity) != 0;
        }

        public bool GetVelocity(BodyId bodyId, out float3 linearVelocity, out float3 angularVelocity)
        {
            linearVelocity = default;
            angularVelocity = default;

            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            fixed (float3* linearPtr = &linearVelocity)
            fixed (float3* angularPtr = &angularVelocity)
            {
                return UnsafeBindings.JPH_PhysicsSystem_GetBodyLinearAndAngularVelocity(
                    physicsSystem,
                    bodyId.Value,
                    linearPtr,
                    angularPtr) != 0;
            }
        }

        public bool AddForce(BodyId bodyId, float3 force)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_AddBodyForce(physicsSystem, bodyId.Value, &force);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool AddTorque(BodyId bodyId, float3 torque)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_AddBodyTorque(physicsSystem, bodyId.Value, &torque);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool AddForceAndTorque(BodyId bodyId, float3 force, float3 torque)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_AddBodyForceAndTorque(physicsSystem, bodyId.Value, &force, &torque);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool AddImpulse(BodyId bodyId, float3 impulse)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_AddBodyImpulse(physicsSystem, bodyId.Value, &impulse);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool Activate(BodyId bodyId)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_ActivateBody(physicsSystem, bodyId.Value);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool Deactivate(BodyId bodyId)
        {
            if (physicsSystem == null || !bodyId.IsValid)
            {
                return false;
            }

            try
            {
                UnsafeBindings.JPH_PhysicsSystem_DeactivateBody(physicsSystem, bodyId.Value);
                return true;
            }
            catch (EntryPointNotFoundException)
            {
                return false;
            }
        }

        public bool IsActive(BodyId bodyId)
        {
            return physicsSystem != null &&
                   bodyId.IsValid &&
                   UnsafeBindings.JPH_PhysicsSystem_IsBodyActive(physicsSystem, bodyId.Value) != 0;
        }

        public bool RemoveAndDestroyConstraint(ConstraintId constraintId)
        {
            if (physicsSystem == null || !constraintId.IsValid)
            {
                return false;
            }

            return UnsafeBindings.JPH_PhysicsSystem_RemoveAndDestroyConstraintByEntity(
                physicsSystem, constraintId.Value) != 0;
        }

        public uint Step(float deltaTime, int collisionSteps)
        {
            return physicsSystem != null
                ? UnsafeBindings.JPH_PhysicsSystem_Update(physicsSystem, deltaTime, collisionSteps)
                : 0xffffffffu;
        }

        public void OptimizeBroadPhase()
        {
            if (physicsSystem != null)
            {
                UnsafeBindings.JPH_PhysicsSystem_OptimizeBroadPhase(physicsSystem);
            }
        }

        public bool CastRay(in RaycastInput input, out RaycastHit hit)
        {
            hit = default;
            if (physicsSystem == null)
            {
                return false;
            }

            var ray = new JPH_RayCast
            {
                origin = input.Origin,
                direction = input.Direction,
            };
            var nativeHit = new JPH_EntityRayCastResult { fraction = 1.0f };

            if (UnsafeBindings.JPH_PhysicsSystem_CastRayEntity(physicsSystem, &ray, &nativeHit) == 0)
            {
                return false;
            }

            hit = new RaycastHit(new BodyId(nativeHit.entityID), nativeHit.fraction, nativeHit.subShapeID2);
            return true;
        }

        public int CastRayAll(in RaycastInput input, NativeList<RaycastHit> hits)
        {
            if (!hits.IsCreated || physicsSystem == null)
            {
                return 0;
            }

            var ray = new JPH_RayCast
            {
                origin = input.Origin,
                direction = input.Direction,
            };

            if (s_CastRayAllUnavailable)
            {
                return CastRayAllFallback(input, hits);
            }

            uint hitCount;
            try
            {
                hitCount = UnsafeBindings.JPH_PhysicsSystem_CastRayAllEntities(physicsSystem, &ray, null, 0);
            }
            catch (EntryPointNotFoundException)
            {
                s_CastRayAllUnavailable = true;
                return CastRayAllFallback(input, hits);
            }

            if (hitCount == 0)
            {
                return 0;
            }

            using var nativeHits = new NativeArray<JPH_EntityRayCastResult>((int)hitCount, Allocator.Temp);
            var writtenCount = UnsafeBindings.JPH_PhysicsSystem_CastRayAllEntities(
                physicsSystem,
                &ray,
                (JPH_EntityRayCastResult*)nativeHits.GetUnsafePtr(),
                hitCount);

            var count = (int)writtenCount;
            for (var i = 0; i < count; i++)
            {
                var nativeHit = nativeHits[i];
                hits.Add(new RaycastHit(new BodyId(nativeHit.entityID), nativeHit.fraction, nativeHit.subShapeID2));
            }

            return count;
        }

        private int CastRayAllFallback(in RaycastInput input, NativeList<RaycastHit> hits)
        {
            const int kMaxIterations = 128;
            const float kAdvanceEpsilon = 1e-2f;

            var startLength = hits.Length;
            var consumedFraction = 0f;
            for (var i = 0; i < kMaxIterations && consumedFraction < 1f; i++)
            {
                var remainingFraction = 1f - consumedFraction;
                var origin = input.Origin + input.Direction * consumedFraction;
                var direction = input.Direction * remainingFraction;
                if (!CastRay(new RaycastInput(origin, direction), out var hit))
                {
                    break;
                }

                var hitFraction = consumedFraction + math.saturate(hit.Fraction) * remainingFraction;
                if (!ContainsHit(hits, hit.BodyId, hit.SubShapeId))
                {
                    hits.Add(new RaycastHit(hit.BodyId, hitFraction, hit.SubShapeId));
                }

                var nextFraction = hitFraction + kAdvanceEpsilon;
                if (nextFraction <= consumedFraction)
                {
                    nextFraction = consumedFraction + kAdvanceEpsilon;
                }

                consumedFraction = nextFraction;
            }

            return hits.Length - startLength;
        }

        private static bool ContainsHit(NativeList<RaycastHit> hits, BodyId bodyId, uint subShapeId)
        {
            for (var i = 0; i < hits.Length; i++)
            {
                var hit = hits[i];
                if (hit.BodyId.Equals(bodyId) && hit.SubShapeId == subShapeId)
                {
                    return true;
                }
            }

            return false;
        }

        public void Dispose()
        {
            if (disposed)
            {
                return;
            }

            disposed = true;
            if (physicsSystem != null)
            {
                UnsafeBindings.JPH_PhysicsSystem_Destroy(physicsSystem);
                physicsSystem = null;
            }

            UnsafeBindings.JPH_Shutdown();
        }

        private static JPH_ConstraintCreationSettings ToNative(in ConstraintCreation creation)
        {
            return new JPH_ConstraintCreationSettings
            {
                kind = (byte)creation.Kind,
                enabled = creation.Enabled ? (byte)1 : (byte)0,
                space = (byte)creation.Space,
                autoDetectPoint = creation.AutoDetectPoint ? (byte)1 : (byte)0,
                priority = creation.Priority,
                numVelocityStepsOverride = creation.NumVelocityStepsOverride,
                numPositionStepsOverride = creation.NumPositionStepsOverride,
                minDistance = creation.MinDistance,
                maxDistance = creation.MaxDistance,
                userData = creation.UserData,
            };
        }
    }

}
