using System;
using Unity.Collections;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Jobs;

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

        private UnsafeUntypedRingBuffer m_Histories;
        private byte[] m_TempBytes = new byte[4096];

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

            if (saveHistoryCount > 0)
            {
                m_StateRecorder = StateRecorderImpl.Create();
                m_StateRecorderFilter = StateRecorderFilter.Create(null);

                m_Histories = new UnsafeUntypedRingBuffer(Allocator.Persistent, 320, 1024);
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
             
                m_Histories.Dispose();
                m_Histories = default;
            }

            if (Main == this) Main = null;
        }

        public JPH_PhysicsUpdateError Tick(float deltaTime)
        {
            
            m_SimulateMarker.Begin();
            var ret = PhysicsSystem.Update(deltaTime, 1, JobSystem.ToUnsafePtr());
            m_SimulateMarker.End();

            m_SyncTransformMarker.Begin();
            NativePhysicsUtility.SyncTransforms(BodyInterface, m_Interpolations.AsArray());
            m_SyncTransformMarker.End();

            if (SaveHistoryCount > 0)
            {
                m_SaveStateMarker.Begin();
                PhysicsSystem.SaveState(m_StateRecorder.ToUnsafePtr(), JPH_StateRecorderState.All, m_StateRecorderFilter.ToUnsafePtr());
                var requiredDataSize = m_StateRecorder.GetDataSize();
                if (requiredDataSize > m_TempBytes.Length)
                {
                    m_TempBytes = new byte[Mathf.NextPowerOfTwo(requiredDataSize)];
                }

                fixed (byte* ptr = m_TempBytes)
                {
                    m_StateRecorder.ReadBytes(ptr, requiredDataSize);
                    m_Histories.Enqueue(ptr, requiredDataSize);
                }
                m_StateRecorder.Clear();
                m_SaveStateMarker.End();
                
                Debug.Log($"History Buffer Size: {(ByteSize)m_Histories.AllocatedBufferLength,10}/{(ByteSize)m_Histories.BufferLength,10}, " +
                          $"Length: {m_Histories.Length,5}/{m_Histories.Capacity,5}");
            }
            
            
            return ret;
        }

        public void TryRollback()
        {
            if (!m_Histories.TryPeek(out var historyPtr, out var historyPtrLength)) return;
            m_RestoreStateMarker.Begin();
                
            m_StateRecorder.WriteBytes(historyPtr, historyPtrLength);
            PhysicsSystem.RestoreState(m_StateRecorder.ToUnsafePtr(), m_StateRecorderFilter.ToUnsafePtr());
            m_StateRecorder.Clear();
                
            m_Histories.Rollback(1);
                
            m_RestoreStateMarker.End();
        }

        public void Interpolate(float deltaTime)
        {
            var deps = NativePhysicsUtility.ScheduleInterpolateTransforms(m_Transforms, m_Interpolations.AsArray(), deltaTime);
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
