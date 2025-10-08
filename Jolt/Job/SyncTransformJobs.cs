using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Jolt.Job
{
    [BurstCompile]
    public unsafe struct SyncTransformJob : IJobParallelForBatch
    {
        public BodyInterface bodyInterface;
        public NativeArray<PhysicsBodyInterpolation> interpolations;

        public void Execute(int startIndex, int count)
        {
            NativePhysicsUtility.SyncTransforms(
                bodyInterface.ToUnsafePtr(),
                count,
                (PhysicsBodyInterpolation*)interpolations.GetUnsafePtr() + startIndex);
        }
    }
}
