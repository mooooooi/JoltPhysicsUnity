using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Jolt.LowLevel
{
    public readonly struct BodyId : IEquatable<BodyId>
    {
        public static readonly BodyId Invalid = new BodyId(0xffffffffu);

        public readonly uint Value;

        public BodyId(uint value)
        {
            Value = value;
        }

        public bool IsValid => Value != Invalid.Value;

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
            return (int)Value;
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

    public readonly unsafe struct Shape : IDisposable
    {
        public readonly JPH_Shape* Ptr;

        public Shape(JPH_Shape* ptr)
        {
            Ptr = ptr;
        }

        public bool IsCreated => Ptr != null;

        public static Shape CreateSphere(float radius)
        {
            return new Shape(UnsafeBindings.JPH_Shape_CreateSphere(radius));
        }

        public static Shape CreateBox(float3 halfExtent, float convexRadius = 0.05f)
        {
            return new Shape(UnsafeBindings.JPH_Shape_CreateBox(halfExtent, convexRadius));
        }

        public static Shape CreateCapsule(float halfHeightOfCylinder, float radius)
        {
            return new Shape(UnsafeBindings.JPH_Shape_CreateCapsule(halfHeightOfCylinder, radius));
        }

        public void AddRef()
        {
            UnsafeBindings.JPH_Shape_AddRef(Ptr);
        }

        public void Dispose()
        {
            UnsafeBindings.JPH_Shape_Release(Ptr);
        }
    }

    public struct BodyCreation
    {
        public Shape Shape;
        public float3 Position;
        public quaternion Rotation;
        public float3 LinearVelocity;
        public float3 AngularVelocity;
        public ulong UserData;
        public uint ObjectLayer;
        public MotionType MotionType;
        public MotionQuality MotionQuality;
        public bool IsSensor;
        public bool AllowSleeping;
        public float LinearDamping;
        public float AngularDamping;
        public float MaxLinearVelocity;
        public float MaxAngularVelocity;
        public float GravityFactor;

        public static BodyCreation Dynamic(Shape shape, float3 position, quaternion rotation, uint objectLayer = 0)
        {
            return new BodyCreation
            {
                Shape = shape,
                Position = position,
                Rotation = rotation,
                ObjectLayer = objectLayer,
                MotionType = MotionType.Dynamic,
                MotionQuality = MotionQuality.Discrete,
                AllowSleeping = true,
                LinearDamping = 0.05f,
                AngularDamping = 0.05f,
                MaxLinearVelocity = 500.0f,
                MaxAngularVelocity = 200.0f,
                GravityFactor = 1.0f,
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

        public RaycastHit(BodyId bodyId, float fraction)
        {
            BodyId = bodyId;
            Fraction = fraction;
        }
    }

    public unsafe sealed class World : IDisposable
    {
        private JPH_PhysicsSystem* physicsSystem;
        private JPH_BodyInterface* bodyInterface;
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

            bodyInterface = UnsafeBindings.JPH_PhysicsSystem_GetBodyInterface(physicsSystem);
        }

        public bool IsCreated => physicsSystem != null;
        public JPH_PhysicsSystem* PhysicsSystem => physicsSystem;
        public JPH_BodyInterface* BodyInterface => bodyInterface;

        public BodyId CreateAndAddBody(in BodyCreation creation, bool activate)
        {
            if (disposed || physicsSystem == null || bodyInterface == null || !creation.Shape.IsCreated)
            {
                return BodyId.Invalid;
            }

            var settings = ToNative(creation);
            var bodyId = UnsafeBindings.JPH_BodyInterface_CreateAndAddBody(
                bodyInterface,
                &settings,
                activate ? JPH_Activation.Activate : JPH_Activation.DontActivate);
            return new BodyId(bodyId);
        }

        public void RemoveAndDestroyBody(BodyId bodyId)
        {
            if (bodyInterface == null || !bodyId.IsValid)
            {
                return;
            }

            UnsafeBindings.JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, bodyId.Value);
        }

        public bool SetPositionAndRotation(BodyId bodyId, float3 position, quaternion rotation, bool activate)
        {
            if (bodyInterface == null || !bodyId.IsValid)
            {
                return false;
            }

            return UnsafeBindings.JPH_BodyInterface_SetPositionAndRotation(
                bodyInterface,
                bodyId.Value,
                position,
                rotation,
                activate ? JPH_Activation.Activate : JPH_Activation.DontActivate) != 0;
        }

        public bool GetPositionAndRotation(BodyId bodyId, out float3 position, out quaternion rotation)
        {
            position = default;
            rotation = quaternion.identity;

            if (bodyInterface == null || !bodyId.IsValid)
            {
                return false;
            }

            fixed (float3* positionPtr = &position)
            fixed (quaternion* rotationPtr = &rotation)
            {
                return UnsafeBindings.JPH_BodyInterface_GetPositionAndRotation(
                    bodyInterface,
                    bodyId.Value,
                    positionPtr,
                    rotationPtr) != 0;
            }
        }

        public bool SetVelocity(BodyId bodyId, float3 linearVelocity, float3 angularVelocity)
        {
            if (bodyInterface == null || !bodyId.IsValid)
            {
                return false;
            }

            return UnsafeBindings.JPH_BodyInterface_SetLinearAndAngularVelocity(
                bodyInterface,
                bodyId.Value,
                linearVelocity,
                angularVelocity) != 0;
        }

        public bool GetVelocity(BodyId bodyId, out float3 linearVelocity, out float3 angularVelocity)
        {
            linearVelocity = default;
            angularVelocity = default;

            if (bodyInterface == null || !bodyId.IsValid)
            {
                return false;
            }

            fixed (float3* linearPtr = &linearVelocity)
            fixed (float3* angularPtr = &angularVelocity)
            {
                return UnsafeBindings.JPH_BodyInterface_GetLinearAndAngularVelocity(
                    bodyInterface,
                    bodyId.Value,
                    linearPtr,
                    angularPtr) != 0;
            }
        }

        public uint Step(float deltaTime, int collisionSteps)
        {
            return physicsSystem != null
                ? UnsafeBindings.JPH_PhysicsSystem_Update(physicsSystem, deltaTime, collisionSteps)
                : 0xffffffffu;
        }

        public bool CastRay(in RaycastInput input, out RaycastHit hit)
        {
            hit = default;
            if (physicsSystem == null)
            {
                return false;
            }

            var query = UnsafeBindings.JPH_PhysicsSystem_GetNarrowPhaseQueryNoLock(physicsSystem);
            if (query == null)
            {
                return false;
            }

            var ray = new JPH_RayCast
            {
                origin = input.Origin,
                direction = input.Direction,
            };
            var nativeHit = new JPH_RayCastResult { fraction = 1.0f };

            if (UnsafeBindings.JPH_NarrowPhaseQuery_CastRay(query, &ray, &nativeHit) == 0)
            {
                return false;
            }

            hit = new RaycastHit(new BodyId(nativeHit.bodyID), nativeHit.fraction);
            return true;
        }

        public UnsafeState SaveState()
        {
            return new UnsafeState(UnsafeBindings.JPH_PhysicsSystem_SaveAlignedState(physicsSystem));
        }

        public bool RestoreState(UnsafeState state)
        {
            return physicsSystem != null
                && state.IsCreated
                && UnsafeBindings.JPH_PhysicsSystem_RestoreAlignedState(physicsSystem, state.Ptr) != 0;
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
                bodyInterface = null;
            }

            UnsafeBindings.JPH_Shutdown();
        }

        private static JPH_BodyCreationSettings ToNative(in BodyCreation creation)
        {
            return new JPH_BodyCreationSettings
            {
                shape = creation.Shape.Ptr,
                position = creation.Position,
                rotation = creation.Rotation,
                linearVelocity = creation.LinearVelocity,
                angularVelocity = creation.AngularVelocity,
                userData = creation.UserData,
                objectLayer = creation.ObjectLayer,
                motionType = (byte)creation.MotionType,
                motionQuality = (byte)creation.MotionQuality,
                isSensor = creation.IsSensor ? (byte)1 : (byte)0,
                allowSleeping = creation.AllowSleeping ? (byte)1 : (byte)0,
                linearDamping = creation.LinearDamping,
                angularDamping = creation.AngularDamping,
                maxLinearVelocity = creation.MaxLinearVelocity,
                maxAngularVelocity = creation.MaxAngularVelocity,
                gravityFactor = creation.GravityFactor,
            };
        }
    }

    public readonly unsafe struct UnsafeState : IDisposable
    {
        public readonly JPH_PhysicsSystemState* Ptr;

        public UnsafeState(JPH_PhysicsSystemState* ptr)
        {
            Ptr = ptr;
        }

        public bool IsCreated => Ptr != null;
        public UIntPtr Size => IsCreated ? UnsafeBindings.JPH_PhysicsSystemState_GetSize(Ptr) : UIntPtr.Zero;
        public byte* Data => IsCreated ? UnsafeBindings.JPH_PhysicsSystemState_GetData(Ptr) : null;

        public void Dispose()
        {
            UnsafeBindings.JPH_PhysicsSystemState_Destroy(Ptr);
        }
    }
}
