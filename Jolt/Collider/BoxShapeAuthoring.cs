using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class BoxShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float3 HalfExtents = new float3(1, 1, 1);
        public float ConvexRadius = 0f;
        
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
