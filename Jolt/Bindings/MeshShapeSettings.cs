using System;
using Unity.Mathematics;

namespace Jolt
{
    public partial struct MeshShapeSettings
    {
        public static unsafe MeshShapeSettings Create(ReadOnlySpan<JPH_Triangle> triangles)
        {
            return default;
        }

        public static unsafe MeshShapeSettings Create2(ReadOnlySpan<float3> vertices, ReadOnlySpan<JPH_IndexedTriangle> triangles)
        {
            return default;
        }
    }
}
