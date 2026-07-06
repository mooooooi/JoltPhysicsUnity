using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Jolt
{
    public static class JoltRuntimeInitialization
    {
        private static readonly object s_AssertLock = new();
        private static readonly OnAssertFailureDel s_OnAssertFailure = OnAssertFailure;
        private static string s_PendingAssertMessage;
        private static string s_LastThrownAssertMessage;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        public static void Initialize()
        {
            lock (s_AssertLock)
            {
                s_PendingAssertMessage = null;
                s_LastThrownAssertMessage = null;
            }
        }

        public static void ThrowPendingAssertIfAny()
        {
            string pendingMessage = null;
            lock (s_AssertLock)
            {
                if (s_PendingAssertMessage == null)
                    return;

                pendingMessage = s_PendingAssertMessage;
                s_LastThrownAssertMessage = pendingMessage;
                s_PendingAssertMessage = null;
            }

            throw new JoltAssertException(pendingMessage);
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate bool OnAssertFailureDel(string expr, string message, string file, uint line);
        
        [AOT.MonoPInvokeCallback(typeof(OnAssertFailureDel))]
        private static bool OnAssertFailure(string expr, string message, string file, uint line)
        {
            var formattedMessage = $"Jolt Assertion Failed: {expr}\n{message}\n{file}:{line}";
            lock (s_AssertLock)
            {
                if (s_PendingAssertMessage == null &&
                    !string.Equals(s_LastThrownAssertMessage, formattedMessage, StringComparison.Ordinal))
                {
                    s_PendingAssertMessage = formattedMessage;
                }
            }

            Debug.LogAssertion(formattedMessage);
            return false;
        }

        public sealed class JoltAssertException : Exception
        {
            public JoltAssertException(string message) : base(message)
            {
            }
        }
    }
}
