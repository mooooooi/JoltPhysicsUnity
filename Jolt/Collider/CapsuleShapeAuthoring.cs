using Drawing;
using Unity.Mathematics;
using UnityEngine;

namespace Jolt.Collider
{
    [DisallowMultipleComponent]
    public class CapsuleShapeAuthoring : MonoBehaviourGizmos, IPhysicsShape
    {
        // Half of the capsule's end-to-end height, including both hemispheres. The default
        // matches Unity's primitive Capsule: radius 0.5 and total height 2.
        [Tooltip("Half of the capsule's total height, including its hemispheres.")]
        public float HalfHeight = 1f;
        
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
