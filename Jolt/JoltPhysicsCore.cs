using System;
using Jolt.Job;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Rendering;

namespace Jolt
{
    public unsafe class JoltPhysicsCore : IDisposable
    {
        private static readonly ProfilerMarker m_SaveStateMarker = new ProfilerMarker("SaveState");
        private static readonly ProfilerMarker m_RestoreStateMarker = new ProfilerMarker("RestoreState");
        private static readonly ProfilerMarker m_SimulateMarker = new ProfilerMarker("Simulate");
        private static readonly ProfilerMarker m_SyncTransformMarker = new ProfilerMarker("SyncTransform");
        private static readonly ProfilerCounterValue<uint> m_TotalBodiesCounter =
            new ProfilerCounterValue<uint>(ProfilerCategory.Physics, "Jolt Bodies Total", ProfilerMarkerDataUnit.Count);
        private static readonly ProfilerCounterValue<uint> m_ActiveRigidBodiesCounter =
            new ProfilerCounterValue<uint>(ProfilerCategory.Physics, "Jolt Bodies Active Rigid", ProfilerMarkerDataUnit.Count);
        private static readonly ProfilerCounterValue<uint> m_InterpolationBodyCounter =
            new ProfilerCounterValue<uint>(ProfilerCategory.Physics, "Jolt Bodies Interpolated", ProfilerMarkerDataUnit.Count);

        public static JoltPhysicsCore Main { get; private set; }

        public readonly PhysicsSystem PhysicsSystem;
        public readonly JobSystem JobSystem;
        public BodyInterface BodyInterface => PhysicsSystem.GetBodyInterface();
        
        public readonly int SaveHistoryCount;
        public bool PrintHistoryMemoryUsed;

        private bool m_Disposed;
        private TransformAccessArray m_Transforms;
        private NativeList<PhysicsBodyInterpolation> m_Interpolations;

        private StateRecorderImpl m_StateRecorder;
        private StateRecorderFilter m_StateRecorderFilter;

        private byte[] m_TempBytes = new byte[4096];

        private float m_InterpolationStartTime;
        private float m_InterpolationDeltaTime;

        public StateRecorderFilter  StateRecorderFilter => m_StateRecorderFilter;
        public StateRecorderImpl StateRecorder => m_StateRecorder;

        public JoltPhysicsCore(
            ObjectLayerPairFilter objectLayerPairFilter, BroadPhaseLayerInterface broadPhaseLayerInterface, ObjectVsBroadPhaseLayerFilter broadPhaseLayerFilter,
            int saveHistoryCount = 0)
        {
            SaveHistoryCount = saveHistoryCount;

            var physicsSystemSettings = default(JPH_PhysicsSystemSettings);

            physicsSystemSettings.objectLayerPairFilter = objectLayerPairFilter.ToUnsafePtr();
            physicsSystemSettings.broadPhaseLayerInterface = broadPhaseLayerInterface.ToUnsafePtr();
            physicsSystemSettings.objectVsBroadPhaseLayerFilter = broadPhaseLayerFilter.ToUnsafePtr();
            
            PhysicsSystem = PhysicsSystem.Create(&physicsSystemSettings);

            var jobSystemSettings = default(JobSystemThreadPoolConfig);
            JobSystem = JobSystemThreadPool.Create(&jobSystemSettings);

            m_Transforms = new TransformAccessArray(32);
            m_Interpolations = new NativeList<PhysicsBodyInterpolation>(32,  Allocator.Persistent);
            
            JoltUnityDebugRendererBridge.Init();
            BodyFilterBridge.Init();
            StateRecorderBridge.Init();

            if (saveHistoryCount > 0)
            {
                m_StateRecorder = StateRecorderImpl.Create();
                m_StateRecorderFilter = StateRecorderFilter.Create(null);
            }

            UpdateProfilerCounters();
        }
        
        public void Dispose()
        {
            if (m_Disposed) return;
            m_Disposed = true;
            ResetProfilerCounters();
            
            if (JobSystem.IsCreated)
                JobSystem.Destroy();
            if (PhysicsSystem.IsCreated)
                PhysicsSystem.Destroy();

            m_Transforms.Dispose();
            m_Interpolations.Dispose();

            if (SaveHistoryCount > 0)
            {
                m_StateRecorder.Destroy();
                m_StateRecorderFilter.Destroy();
               
            }

            if (Main == this) Main = null;
            JoltCore.Shutdown();
        }

        public int GetInterpolationCount()
        {
            return m_Interpolations.Length;
        }

        public JobHandle ScheduleUpdate(float deltaTime, JobHandle dep = default)
        {
            return ScheduleUpdate(m_Interpolations.Length, deltaTime, dep);
        }
        
        public JobHandle ScheduleUpdate(int length, float deltaTime, JobHandle dep = default, bool simulate = true)
        {
            m_InterpolationDeltaTime = deltaTime;
            m_InterpolationStartTime = Time.time;

            if (simulate)
            {
                var updateJob = new UpdatePhysicsSystemJob()
                {
                    physics = PhysicsSystem, job = JobSystem, deltaTime = deltaTime
                };
                dep = updateJob.ScheduleByRef(dep);
            }

            var syncTransformJob = new SyncTransformJob()
            {
                bodyInterface = BodyInterface, interpolations = m_Interpolations.AsArray()
            };
            dep = syncTransformJob.ScheduleParallelByRef(length, 16, dep);
            return dep;
        }

        public void Run(float deltaTime)
        {
            m_InterpolationDeltaTime = deltaTime;
            m_InterpolationStartTime = Time.time;
            
            PhysicsSystem.Update(deltaTime, 1, JobSystem.ToUnsafePtr());
            UpdateProfilerCounters();
            
            var syncTransformJob = new SyncTransformJob()
            {
                bodyInterface = BodyInterface, interpolations = m_Interpolations.AsArray()
            };
            syncTransformJob.RunBatchByRef(m_Interpolations.Length);
        }

        public bool TryRollback(UnsafeRingBuffer histories, uint sequenceId)
        {
            if (!histories.TryGetValue(sequenceId, out var buffer)) return false;

            fixed (void* bufferPtr = buffer)
            {
                // buffer starts with BlobAssetHeader; RestoreAlignedState expects PhysicsSystemState* directly
                return PhysicsSystem.RestoreAlignedState(
                    (byte*)bufferPtr + sizeof(BlobAssetHeader),
                    m_StateRecorderFilter.ToUnsafePtr());
            }
        }

        public void Interpolate()
        {
            var interpolationFactor = math.clamp((Time.time - m_InterpolationStartTime) / m_InterpolationDeltaTime, 0f, 1f);
            var deps = NativePhysicsUtility.ScheduleInterpolateTransforms(m_Transforms, m_Interpolations.AsArray(), interpolationFactor);
            deps.Complete();
        }

        public void MapBodyToTransform(Transform transform, uint bodyId)
        {
            m_Transforms.Add(transform);
            m_Interpolations.Add(new PhysicsBodyInterpolation()
            {
                bodyId = bodyId,
                previous = transform.localToWorldMatrix,
                current = transform.localToWorldMatrix
            });
            m_InterpolationBodyCounter.Value = (uint)m_Interpolations.Length;
        }

        public void UnmapBodyToTransform(uint bodyId)
        {
            int index = -1;
            for (var i = 0; i < m_Interpolations.Length; i++)
            {
                if (m_Interpolations[i].bodyId == bodyId)
                {
                    index = i;
                    break;
                }
            }

            if (index < 0) return;
            m_Transforms.RemoveAtSwapBack(index);
            m_Interpolations.RemoveAtSwapBack(index);
            m_InterpolationBodyCounter.Value = (uint)m_Interpolations.Length;
        }

        public void SetIsMain()
        {
            if (Main != null) throw new InvalidOperationException("Main JoltPhysicsCore is exists, You must dispose it first!");
            Main = this;
        }

        private void UpdateProfilerCounters()
        {
            if (!PhysicsSystem.IsCreated)
            {
                ResetProfilerCounters();
                return;
            }

            var physicsSystem = PhysicsSystem.ToUnsafePtr();
            m_TotalBodiesCounter.Value = UnsafeBindings.JPH_PhysicsSystem_GetNumBodies(physicsSystem);
            m_ActiveRigidBodiesCounter.Value = UnsafeBindings.JPH_PhysicsSystem_GetNumActiveBodies(
                physicsSystem,
                JPH_BodyType.Rigid);
            m_InterpolationBodyCounter.Value = (uint)m_Interpolations.Length;
        }

        private static void ResetProfilerCounters()
        {
            m_TotalBodiesCounter.Value = 0;
            m_ActiveRigidBodiesCounter.Value = 0;
            m_InterpolationBodyCounter.Value = 0;
        }
    }
}
