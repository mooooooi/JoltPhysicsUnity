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
                ShouldSaveBody = Marshal.GetFunctionPointerForDelegate<ShouldSaveBodyDel>(ShouldSaveBody),
                ShouldSaveConstraint = Marshal.GetFunctionPointerForDelegate<ShouldSaveConstraintDel>(ShouldSaveConstraint),
                ShouldSaveContact = Marshal.GetFunctionPointerForDelegate<ShouldSaveContactDel>(ShouldSaveContact), 
                ShouldRestoreContact = Marshal.GetFunctionPointerForDelegate<ShouldRestoreContactDel>(ShouldRestoreContact)
            };
            Handle = GCHandle.Alloc(props, GCHandleType.Pinned);
            
            UnsafeBindings.JPH_StateRecorderFilter_SetProcs((JPH_StateRecorderFilter_Procs*)Handle.AddrOfPinnedObject());
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate bool ShouldSaveBodyDel(void* userData, JPH_Body* body);
        [AOT.MonoPInvokeCallback(typeof(ShouldSaveBodyDel))]
        private static unsafe bool ShouldSaveBody(void* userData, JPH_Body* body)
        {
            if (body == null)
                return false;

            if (UnsafeBindings.JPH_Body_IsStatic(body) != 0)
                return false;

            return UnsafeBindings.JPH_Body_GetObjectLayer(body) != 0;
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate bool ShouldSaveConstraintDel(void* userData, JPH_Constraint* constraint);
        [AOT.MonoPInvokeCallback(typeof(ShouldSaveConstraintDel))]
        private static unsafe bool ShouldSaveConstraint(void* userData, JPH_Constraint* constraint)
        {
            return true;
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
