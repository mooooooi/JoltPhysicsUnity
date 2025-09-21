using System;
using Unity.Mathematics;

namespace Jolt
{
    public partial struct MeshShapeSettings
    {
        public static unsafe MeshShapeSettings Create(ReadOnlySpan<JPH_Triangle> triangles)
        {
            fixed (JPH_Triangle* ptr = triangles)
            {
                var returnValue = UnsafeBindings.JPH_MeshShapeSettings_Create(ptr, (uint)triangles.Length);
                return new MeshShapeSettings{ Ptr = returnValue };
            }
        }

        public static unsafe MeshShapeSettings Create2(ReadOnlySpan<float3> vertices, ReadOnlySpan<JPH_IndexedTriangle> triangles)
        {
            fixed (float3* vptr = vertices)
            fixed (JPH_IndexedTriangle* tptr = triangles)
            {
                var returnValue = UnsafeBindings.JPH_MeshShapeSettings_Create2(vptr, (uint)vertices.Length, tptr, (uint)triangles.Length);
                return new MeshShapeSettings{ Ptr = returnValue };
            }
        }
    }
}
