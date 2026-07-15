using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class SphereShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        public float Capsule = 1f;
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
