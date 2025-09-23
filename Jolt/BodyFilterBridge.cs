using System.Runtime.InteropServices;

namespace Jolt
{
    public static class BodyFilterBridge
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate bool ShouldDrawDel(void* userData, JPH_Body* body);

        private static bool IsInitialized;
        private static GCHandle Handle;

        [AOT.MonoPInvokeCallback(typeof(ShouldDrawDel))]
        private static unsafe bool ShouldDraw(void* userData, JPH_Body* body)
        {
            return true;
        }

        public static unsafe void Init()
        {
            if (IsInitialized) return;
            IsInitialized = true;

            var props = new JPH_BodyDrawFilter_Procs
            {
                ShouldDraw = Marshal.GetFunctionPointerForDelegate<ShouldDrawDel>(ShouldDraw)
            };
            Handle = GCHandle.Alloc(props, GCHandleType.Pinned);
            
            UnsafeBindings.JPH_BodyDrawFilter_SetProcs((JPH_BodyDrawFilter_Procs*)Handle.AddrOfPinnedObject());
        }
    }
}
