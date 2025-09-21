using System;
using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class SphereCollider : MonoBehaviourGizmos, IPhysicsShape
    {
        public float Capsule = 1f;
        private SphereShape m_Shape;
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
                m_Shape = SphereShape.Create(Capsule);
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
