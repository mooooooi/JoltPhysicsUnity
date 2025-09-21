using System.Runtime.InteropServices;
using Drawing;

namespace Jolt
{
    public static class JoltUnityDebugRendererBridge
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate void DrawLineDel(void* userData, rvec3* p0, rvec3* p1, uint color);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate void DrawTriangleDel(void* userData, rvec3* p0, rvec3* p1, rvec3* p2, uint color,
            JPH_DebugRenderer_CastShadow castShadow);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private unsafe delegate void DrawText3DDel(void* userData, rvec3* point, 
            [MarshalAs(UnmanagedType.LPStr)]
            string text, uint color, float fontSize);
        
        private static unsafe void DrawLine(void* userData, rvec3* p0, rvec3* p1, uint color)
        {
            var structColor = new JPH_Color()
            {
                Value = color
            };
            Draw.Line(*p0, *p1, structColor);
        }
        
        private static unsafe void DrawTriangle(
            void* userData, rvec3* p0, rvec3* p1, rvec3* p2, uint color, JPH_DebugRenderer_CastShadow castShadow)
        {
            var structColor = new JPH_Color()
            {
                Value = color
            };
            Draw.SolidTriangle(*p0, *p1, *p2, structColor);
        }

        private static unsafe void DrawText3D(void* userData, rvec3* point, string text, uint color, float fontSize)
        {
            var structColor = new JPH_Color()
            {
                Value = color
            };
            Draw.Label2D(*point, text, fontSize, structColor);
        }

        private static bool IsInitialized;
        private static GCHandle Handle;
        public static unsafe void Init()
        {
            if (IsInitialized) return;
            IsInitialized = true;

            var props = new JPH_DebugRenderer_Procs()
            {
                DrawLine = Marshal.GetFunctionPointerForDelegate<DrawLineDel>(DrawLine),
                DrawTriangle = Marshal.GetFunctionPointerForDelegate<DrawTriangleDel>(DrawTriangle),
                DrawText3D = Marshal.GetFunctionPointerForDelegate<DrawText3DDel>(DrawText3D),
            };

            Handle = GCHandle.Alloc(props, GCHandleType.Pinned);
            
            DebugRenderer.SetProcs((JPH_DebugRenderer_Procs*)Handle.AddrOfPinnedObject());
        }
    }
}
