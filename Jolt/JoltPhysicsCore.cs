using System;
using Jolt.Job;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
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
        }
        
        public void Dispose()
        {
            if (m_Disposed) return;
            m_Disposed = true;
            
            if (JobSystem.IsCreated)
                JobSystem.Destroy();
            if (PhysicsSystem.IsCreated)
                PhysicsSystem.Destroy();
            JoltCore.Shutdown();

            m_Transforms.Dispose();
            m_Interpolations.Dispose();

            if (SaveHistoryCount > 0)
            {
                m_StateRecorder.Destroy();
                m_StateRecorderFilter.Destroy();
               
            }

            if (Main == this) Main = null;
        }

        public JobHandle ScheduleUpdate(float deltaTime, JobHandle dep = default)
        {
            m_InterpolationDeltaTime = deltaTime;
            m_InterpolationStartTime = Time.time;
            
            var updateJob = new UpdatePhysicsSystemJob()
            {
                physics = PhysicsSystem, job = JobSystem, deltaTime = deltaTime
            };
            dep = updateJob.ScheduleByRef(dep);

            var syncTransformJob = new SyncTransformJob()
            {
                bodyInterface = BodyInterface, interpolations = m_Interpolations.AsArray()
            };
            dep = syncTransformJob.ScheduleParallelByRef(m_Interpolations.Length, 16, dep);
            
            return dep;
        }

        public void TryRollback(NativeRingBuffer histories)
        {
            // if (!histories.TryPeek(out var historyPtr, out var historyPtrLength)) return;
            // m_RestoreStateMarker.Begin();
            //     
            // m_StateRecorder.WriteBytes(historyPtr, historyPtrLength);
            // PhysicsSystem.RestoreState(m_StateRecorder.ToUnsafePtr(), m_StateRecorderFilter.ToUnsafePtr());
            // m_StateRecorder.Clear();
            //     
            // histories.Rollback(1);
            // m_RestoreStateMarker.End();
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
        }

        public void SetIsMain()
        {
            if (Main != null) throw new InvalidOperationException("Main JoltPhysicsCore is exists, You must dispose it first!");
            Main = this;
        }
    }
}
