using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Jolt.Job
{
    [BurstCompile]
    public unsafe struct UpdatePhysicsSystemJob : IJob
    {
        [NativeDisableUnsafePtrRestriction]
        public JPH_PhysicsSystem* PhysicsSystem;
        public float DeltaTime;

        public void Execute()
        {
            UnsafeBindings.JPH_PhysicsSystem_Update(PhysicsSystem, DeltaTime, 1);
        }
    }
}
