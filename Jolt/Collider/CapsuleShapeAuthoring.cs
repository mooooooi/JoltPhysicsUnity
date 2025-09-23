using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class CapsuleShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float HalfHeight = 0.5f;
        
        public float Radius = 0.5f;
        
        private Jolt.CapsuleShape m_Shape;
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
                m_Shape = Jolt.CapsuleShape.Create(HalfHeight, Radius);
            }

            return m_Shape.AsShape;
        }
        
        public override  void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WireCapsule(- transform.up * HalfHeight * 2, math.up(), HalfHeight * 2f + Radius * 2, Radius);
            }
        }
        
    }
}
