using System;
using Jolt;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt
{
    [Flags]
    public enum DebuggerDrawFlags
    {
        GetSupportFunction = 1 << 0,
        SupportDirection = 1 << 1,          
        GetSupportingFace = 1 << 2,          
        Shape = 1 << 3,
        ShapeWireframe = 1 << 4,
        BoundingBox = 1 << 5,
        CenterOfMassTransform = 1 << 6,
        WorldTransform = 1 << 7,
        Velocity = 1 << 8,
        MassAndInertia = 1 << 9,
        SleepStats = 1 << 10,
        SoftBodyVertices = 1 << 11,
        SoftBodyVertexVelocities = 1 << 12,
        SoftBodyEdgeConstraints = 1 << 13,
        SoftBodyBendConstraints = 1 << 14,
        SoftBodyVolumeConstraints = 1 << 15,
        SoftBodySkinConstraints = 1 << 16,
        SoftBodyLRAConstraints = 1 << 17,
        SoftBodyPredictedBounds = 1 << 18
    }
    
    public enum ShapeColor : uint
    {
        InstanceColor,
        ShapeTypeColor,
        MotionTypeColor,
        SleepColor,
        IslandColor,
        MaterialColor,
    }
    
    public enum SoftBodyConstraintColor : uint
    {
        ConstraintType,
        ConstraintGroup,
        ConstraintOrder,
    }
    
    [Serializable]
    public struct DebuggerDrawSettings
    {
        public DebuggerDrawFlags Flags;
        public SoftBodyConstraintColor drawSoftBodyConstraintColor;
    }
    
    public class JoltUnityDebugRenderer : MonoBehaviour
    {
        public bool Draw;
        public DebuggerDrawSettings Settings;
        public ShapeColor ShapeColor;
        public SoftBodyConstraintColor SoftBodyConstraintColor;
        
        private DebugRenderer m_DebugRenderer;
        private JPH_DrawSettings m_DrawSettings;
        private BodyDrawFilter m_BodyFilter;
        private PhysicsSystem m_PhysicsSystem;
        
        private unsafe void Awake()
        {
            m_DebugRenderer = DebugRenderer.Create(null);
            
            updateSettings();
            
            m_BodyFilter = BodyDrawFilter.Create(null);
        }

        public void NextFrame()
        {
            m_DebugRenderer.NextFrame();
        }

        public void BindPhysicsSystem(PhysicsSystem physicsSystem)
        {
            m_PhysicsSystem = physicsSystem;
        }

        public void UnbindPhysicsSystem()
        {
            m_PhysicsSystem = default;
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
            Render(m_PhysicsSystem);
        }

        private void OnDestroy()
        {
            m_DebugRenderer.Destroy();
            m_DebugRenderer = default;
            
            m_BodyFilter.Destroy();
            m_BodyFilter = default;
        }

        private byte getFlagByte(DebuggerDrawFlags flag)
        {
            return (Settings.Flags & flag) > 0 ? (byte)1 : (byte)0;
        }
        
        private void updateSettings()
        {
            m_DrawSettings.drawBoundingBox = getFlagByte(DebuggerDrawFlags.BoundingBox);
            m_DrawSettings.drawCenterOfMassTransform = getFlagByte(DebuggerDrawFlags.CenterOfMassTransform);
            m_DrawSettings.drawGetSupportFunction = getFlagByte(DebuggerDrawFlags.GetSupportFunction);
            m_DrawSettings.drawGetSupportingFace = getFlagByte(DebuggerDrawFlags.GetSupportingFace);
            m_DrawSettings.drawMassAndInertia = getFlagByte(DebuggerDrawFlags.MassAndInertia);
            m_DrawSettings.drawShape = getFlagByte(DebuggerDrawFlags.Shape);
            m_DrawSettings.drawShapeWireframe = getFlagByte(DebuggerDrawFlags.ShapeWireframe);
            m_DrawSettings.drawSleepStats = getFlagByte(DebuggerDrawFlags.SleepStats);
            m_DrawSettings.drawSoftBodyBendConstraints = getFlagByte(DebuggerDrawFlags.SoftBodyBendConstraints);
            m_DrawSettings.drawSoftBodyEdgeConstraints = getFlagByte(DebuggerDrawFlags.SoftBodyEdgeConstraints);
            m_DrawSettings.drawSoftBodyLRAConstraints = getFlagByte(DebuggerDrawFlags.SoftBodyLRAConstraints);
            m_DrawSettings.drawSoftBodyPredictedBounds = getFlagByte(DebuggerDrawFlags.SoftBodyPredictedBounds);
            m_DrawSettings.drawSoftBodySkinConstraints = getFlagByte(DebuggerDrawFlags.SoftBodySkinConstraints);
            m_DrawSettings.drawSoftBodyVertexVelocities = getFlagByte(DebuggerDrawFlags.SoftBodyVertexVelocities);
            m_DrawSettings.drawSoftBodyVertices = getFlagByte(DebuggerDrawFlags.SoftBodyVertices);
            m_DrawSettings.drawSoftBodyVolumeConstraints = getFlagByte(DebuggerDrawFlags.SoftBodyVolumeConstraints);
            m_DrawSettings.drawSupportDirection = getFlagByte(DebuggerDrawFlags.SupportDirection);
            m_DrawSettings.drawVelocity = getFlagByte(DebuggerDrawFlags.Velocity);
            m_DrawSettings.drawWorldTransform = getFlagByte(DebuggerDrawFlags.WorldTransform);

            m_DrawSettings.drawShapeColor = (JPH_BodyManager_ShapeColor)ShapeColor;
            m_DrawSettings.drawSoftBodyConstraintColor = (JPH_SoftBodyConstraintColor)SoftBodyConstraintColor;
        }
        
        #if UNITY_EDITOR
        private void OnValidate()
        {
            updateSettings();
        }
#endif
    }
}
