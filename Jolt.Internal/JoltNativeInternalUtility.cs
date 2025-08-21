using System;
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
    }
}
