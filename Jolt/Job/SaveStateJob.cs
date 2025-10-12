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

        public uint frameId;
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
            histories.Allocate(frameId, ptr, requiredDataSize);
            stateRecorder.Clear();
        }
    }

    [BurstCompile]
    public unsafe struct SaveAlignedJob : IJob
    {
        public uint frameId;
        public PhysicsSystem physicsSystem;
        public StateRecorderFilter stateRecorderFilter;
        public NativeRingBuffer histories;
        
        public void Execute()
        {
            var builder = physicsSystem.SaveAlignedState(
                JPH_StateRecorderState.All,
                stateRecorderFilter.ToUnsafePtr());
            
            var requiredDataSize = builder.GetRequiredByteCount();
            histories.Allocate(frameId, (int)requiredDataSize, out var span);
            fixed (void* ptr = span)
            {
                builder.Flush(ptr, requiredDataSize);
            }
            // builder.Destroy();
        }
    }
}
