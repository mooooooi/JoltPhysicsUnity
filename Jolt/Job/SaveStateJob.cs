using System;
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
        }
    }

    [BurstCompile]
    public unsafe struct SaveAlignedJob : IJob
    {
        public uint frameId;
        public PhysicsSystem physicsSystem;
        public StateRecorderFilter stateRecorderFilter;
        public NativeRingBuffer histories;
        public BlobBuilder builder;
        
        public void Execute()
        {
        }
    }
}
