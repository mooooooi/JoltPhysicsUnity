using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class BoxCollider : MonoBehaviourGizmos, IPhysicsShape
    {
        public float3 HalfExtents = new float3(1, 1, 1);
        public float ConvexRadius = 0f;
        
        private BoxShape m_Shape;
        private uint m_BodyId;

        private void OnDestroy()
        {
            if (m_Shape.IsCreated)
            {
                m_Shape.Destroy();
                m_Shape = default;
            }
        }

        public Shape GetOrCreateShape()
        {
            if (!m_Shape.IsCreated)
            {
                m_Shape = BoxShape.Create(HalfExtents, ConvexRadius);
            }

            return m_Shape.AsShape;
        }
        
        public override  void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WireBox(float3.zero, HalfExtents * 2);
            }
        }
    }
}
