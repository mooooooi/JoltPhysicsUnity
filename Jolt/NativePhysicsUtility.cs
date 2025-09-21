using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace Jolt
{
    public struct PhysicsBodyInterpolation
    {
        public uint bodyId;
        public float4x4 previous;
        public float4x4 current;
    }
    
    [BurstCompile]
    public static class NativePhysicsUtility
    {
        public static unsafe void SyncTransforms(
            BodyInterface bodyInterface, NativeArray<PhysicsBodyInterpolation> interpolations)
        {
            SyncTransforms(bodyInterface.ToUnsafePtr(), interpolations.Length, (PhysicsBodyInterpolation*)interpolations.GetUnsafePtr());
        }
        
        [BurstCompile]
        internal static unsafe void SyncTransforms(JPH_BodyInterface* bodies, int length, PhysicsBodyInterpolation* interpolations)
        {
            for (var i = 0; i < length; i++)
            {
                ref var interpolation = ref interpolations[i];
                
                interpolation.previous = interpolation.current;
                
                rmatrix4x4 m;
                UnsafeBindings.JPH_BodyInterface_GetWorldTransform(bodies, interpolation.bodyId, &m);
                interpolation.current = m;
            }
        }

        public static JobHandle ScheduleInterpolateTransforms(
            TransformAccessArray transforms, NativeArray<PhysicsBodyInterpolation> interpolations, float interpolationFactor, JobHandle inputDeps = default)
        {
            var job = new PhysicsBodyInterpolationJob()
            {
                interpolationFactor = interpolationFactor, interpolations = interpolations
            };
            return job.ScheduleByRef(transforms, inputDeps);
        }
    }

    [BurstCompile]
    public struct PhysicsBodyInterpolationJob : IJobParallelForTransform
    {
        public float interpolationFactor;
        [ReadOnly]
        public NativeArray<PhysicsBodyInterpolation> interpolations;
        public void Execute(int index, TransformAccess transform)
        {
            var interpolation = interpolations[index];
            var position = math.lerp(interpolation.previous.c3.xyz, interpolation.current.c3.xyz, interpolationFactor);
            var rotation = math.slerp(new quaternion(interpolation.previous), new quaternion(interpolation.current), interpolationFactor);
            transform.SetPositionAndRotation(position, rotation);
        }
    }
}
