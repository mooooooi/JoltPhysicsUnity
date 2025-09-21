using System.Runtime.InteropServices;
using UnityEngine;

namespace Jolt
{
    internal static class JoltRuntimeInitialization
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        public static void Initialize()
        {
            JoltCore.SetAssertFailureHandler(Marshal.GetFunctionPointerForDelegate<OnAssertFailureDel>(OnAssertFailure));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate bool OnAssertFailureDel(string expr, string message, string file, uint line);
        
        private static bool OnAssertFailure(string expr, string message, string file, uint line)
        {
            Debug.Log($"Jolt Assertion Failed:\n{expr}\n{message}\n{file}\n{line}");
            return false;
        }
    }
}
