using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    public class PlaneShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float HalfExtends = 1f;
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
