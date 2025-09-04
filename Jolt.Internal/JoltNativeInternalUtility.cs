using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Jolt.Internal
{
    internal static class JoltNativeInternalUtility
    {
        public static void LeakRecord(IntPtr handle, int callstacksToSkip)
        {
            UnsafeUtility.LeakRecord(handle, LeakCategory.Persistent, callstacksToSkip);
        }

        public static void LeakErase(IntPtr handle)
        {
            UnsafeUtility.LeakErase(handle, LeakCategory.Persistent);
        }

        unsafe struct UnsafeHeader<TPtr> where TPtr : unmanaged
        {
            public TPtr** Ptr;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            internal AtomicSafetyHandle m_Safety;
#endif
        }

        public static unsafe void Initialize<TStruct, TPtr>(ref TStruct t, TPtr* ptr) where TStruct : unmanaged where TPtr : unmanaged
        {
            ref var header = ref UnsafeUtility.As<TStruct, UnsafeHeader<TPtr>>(ref t);
            header.Ptr = (TPtr**)UnsafeUtility.MallocTracked(sizeof(TPtr*), UnsafeUtility.AlignOf<IntPtr>(), Allocator.Persistent, 1);
            *header.Ptr = ptr;
        }
    }
}
