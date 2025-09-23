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
        public float3 Center;
        
        private Shape m_Shape;
        private uint m_BodyId;

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
            if (!m_Shape.IsCreated)
            {
                var halfHeight = math.max(HalfHeight, Radius);
                var capsuleShapeSettings = CapsuleShapeSettings.Create(halfHeight - Radius, Radius);
                var offsetShapeSettings = RotatedTranslatedShapeSettings.Create(Center, quaternion.identity, capsuleShapeSettings.AsShapeSettings.ToUnsafePtr());
                m_Shape = offsetShapeSettings.CreateShape().AsShape;
            }
            return m_Shape;
        }

        public JPH_Plane GetSupportingVolume() => new JPH_Plane()
        {

            normal = math.up(), distance = HalfHeight - math.dot(Center, math.up())
        };

        public override void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                var halfHeight = math.max(HalfHeight, Radius);
                using (Draw.WithMatrix(transform.localToWorldMatrix))
                {
                    Draw.WireCapsule(-(halfHeight) * math.up() + Center, math.up(), halfHeight * 2f, Radius);
                }
            }
        }
        
    }
}
