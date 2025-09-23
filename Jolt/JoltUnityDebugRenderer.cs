using System;
using Jolt;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt
{
    public class JoltUnityDebugRenderer : MonoBehaviour
    {
        public bool Draw;
        
        private DebugRenderer m_DebugRenderer;
        private JPH_DrawSettings m_DrawSettings;
        private BodyDrawFilter m_BodyFilter;
        
        private unsafe void Awake()
        {
            m_DebugRenderer = DebugRenderer.Create(null);
            fixed(JPH_DrawSettings* settingsPtr = &m_DrawSettings)
                UnsafeBindings.JPH_DrawSettings_InitDefault(settingsPtr);
            
            m_BodyFilter = BodyDrawFilter.Create(null);
        }

        public void NextFrame()
        {
            m_DebugRenderer.NextFrame();
        }

        public unsafe void Render(PhysicsSystem physicsSystem)
        {
            Camera camera = null;
#if UNITY_EDITOR
            camera = UnityEditor.SceneView.lastActiveSceneView?.camera;
#endif
            
            if (camera !=null)
            {
                m_DebugRenderer.SetCameraPos(camera.transform.position);
            }
            if (Draw)
            {
                if (!physicsSystem.IsCreated) return;

                fixed (JPH_DrawSettings* settingsPtr = &m_DrawSettings)
                {
                    physicsSystem.DrawBodies(settingsPtr, m_DebugRenderer.ToUnsafePtr(), m_BodyFilter.ToUnsafePtr());
                }
            }
        }

        public void LateUpdate()
        {
            Render(JoltPhysicsCore.Main.PhysicsSystem);
        }

        private void OnDestroy()
        {
            m_DebugRenderer.Destroy();
            m_DebugRenderer = default;
            
            m_BodyFilter.Destroy();
            m_BodyFilter = default;
        }
    }
}
