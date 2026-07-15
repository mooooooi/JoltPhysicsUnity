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
