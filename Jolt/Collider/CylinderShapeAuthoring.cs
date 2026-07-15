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
        
        public override void DrawGizmos()
        {
            if (GizmoContext.InSelection(this))
            {
                using(Draw.WithMatrix(transform.localToWorldMatrix))
                    Draw.WireCylinder(- transform.up * HalfHeight, math.up(), HalfHeight * 2f, Radius);
            }
        }
    }
}
