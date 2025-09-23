using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    public class PlaneShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float HalfExtends = 1f;
        private Jolt.PlaneShape m_Shape;

        private void OnDestroy()
        {
            if (m_Shape.IsCreated)
            {
                m_Shape.Destroy();
                m_Shape = default;
            }
        }

        public unsafe Shape GetOrCreateShape()
        {
            var planeSettings = new JPH_Plane
            {
                normal = math.up(), distance = 0f
            };

            m_Shape = Jolt.PlaneShape.Create(&planeSettings, null, HalfExtends * 5);

            return m_Shape.AsShape;
        }
        
        public override  void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WirePlane(float3.zero, math.up(), new float2(HalfExtends * 10, HalfExtends * 10));
            }
        }
    }
}
