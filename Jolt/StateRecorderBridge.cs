using System.Runtime.InteropServices;

namespace Jolt
{
    public static class StateRecorderBridge
    {
        private static bool IsInitialized;
        private static GCHandle Handle;
        
        public static unsafe void Init()
        {
            if (IsInitialized) return;
            IsInitialized = true;

            var props = new JPH_StateRecorderFilter_Procs()
            {
                ShouldSaveContact = Marshal.GetFunctionPointerForDelegate<ShouldSaveContactDel>(ShouldSaveContact), 
                ShouldRestoreContact = Marshal.GetFunctionPointerForDelegate<ShouldRestoreContactDel>(ShouldRestoreContact)
            };
            Handle = GCHandle.Alloc(props, GCHandleType.Pinned);
            
            UnsafeBindings.JPH_StateRecorderFilter_SetProcs((JPH_StateRecorderFilter_Procs*)Handle.AddrOfPinnedObject());
        }
        
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate bool ShouldSaveContactDel(void* userData, uint body0, uint body1);
        [AOT.MonoPInvokeCallback(typeof(ShouldSaveContactDel))]
        private static unsafe bool ShouldSaveContact(void* userData, uint body0, uint body1)
        {
            return true; 
        }
        
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate bool ShouldRestoreContactDel(void* userData, uint body0, uint body1);
        [AOT.MonoPInvokeCallback(typeof(ShouldRestoreContactDel))]
        private static unsafe bool ShouldRestoreContact(void* userData, uint body0, uint body1)
        {
            return true;
        }
    }
}
