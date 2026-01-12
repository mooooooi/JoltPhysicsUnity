using System;
using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class SphereShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float Capsule = 1f;
        private Jolt.SphereShape m_Shape;
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
                var scale3 = (float3)transform.localScale;
                var scale = math.min(scale3.x, scale3.z);
                m_Shape = Jolt.SphereShape.Create(Capsule * scale);
            }

            return m_Shape.AsShape;
        }

        public override  void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WireSphere(float3.zero, Capsule);
            }
        }
    }
}
