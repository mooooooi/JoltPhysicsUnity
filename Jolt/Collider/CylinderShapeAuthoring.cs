using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class CylinderShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float HalfHeight = 1f;
        
        public float Radius = 0.5f;
        
        private Jolt.CylinderShape m_Shape;
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
                m_Shape = Jolt.CylinderShape.Create(HalfHeight, Radius);
            }

            return m_Shape.AsShape;
        }
        
        public override  void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WireCylinder(- transform.up * HalfHeight, math.up(), HalfHeight * 2f, Radius);
            }
        }
    }
}
