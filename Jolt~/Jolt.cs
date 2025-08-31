using Unity.Burst;
using static Jolt.Bindings;

namespace Jolt
{
    public static class Jolt
    {
        public struct InitializedDummy {}
        /// <summary>
        /// True Jolt is current initialized.
        /// </summary>
        public static bool Initialized { get; private set; }
        public static SharedStatic<bool> s_Initialized = SharedStatic<bool>.GetOrCreate<InitializedDummy>();

        /// <summary>
        /// Initialize Jolt, returning true if initialization succeeded.
        /// </summary>
        public static bool Initialize()
        {
            if (!Initialized)
            {
                Initialized = JPH_Init();
                s_Initialized.Data = Initialized;
            }

            return Initialized;
        }

        /// <summary>
        /// Set the assertion failure handler for Jolt.
        /// </summary>
        public static void SetAssertFailureHandler(AssertFailureHandler handler)
        {
            JPH_SetAssertFailureHandler(handler);
        }

        /// <summary>
        /// Shutdown Jolt.
        /// </summary>
        public static void Shutdown()
        {
            JPH_Shutdown();

            Initialized = false;
            s_Initialized.Data = false;
        }
    }
}
