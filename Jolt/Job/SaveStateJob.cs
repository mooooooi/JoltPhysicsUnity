using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityEngine;

namespace Jolt.Job
{
    [BurstCompile]
    public unsafe struct SaveStateJob : IJob
    {
        public PhysicsSystem physicsSystem;
        public StateRecorderImpl stateRecorder;
        public StateRecorderFilter stateRecorderFilter;
        public NativeRingBuffer histories;

        public void Execute()
        {
            physicsSystem.SaveState(
                stateRecorder.ToUnsafePtr(), 
                JPH_StateRecorderState.All,
                stateRecorderFilter.ToUnsafePtr());
            var requiredDataSize = stateRecorder.GetDataSize();

            var ptr = stackalloc byte[requiredDataSize];

            stateRecorder.ReadBytes(ptr, requiredDataSize);
            histories.Enqueue(ptr, requiredDataSize);
            stateRecorder.Clear();
        }
    }
}
