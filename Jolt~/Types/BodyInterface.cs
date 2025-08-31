using System;
using Unity.Mathematics;

namespace Jolt
{
    [GenerateBindings("JPH_BodyInterface")]
    public partial struct BodyInterface
    {
        internal NativeHandle<JPH_BodyInterface> Handle;

        public IntPtr GetUnsafePtr() => Handle.RawValue;

        public static unsafe float4x4 UnsafeGetWorldTransform(IntPtr ptr, BodyID body)
        {
            rmatrix4x4 m;
            UnsafeBindings.JPH_BodyInterface_GetWorldTransform((JPH_BodyInterface*)ptr, body, &m);
            return m;
        }
    }
}
