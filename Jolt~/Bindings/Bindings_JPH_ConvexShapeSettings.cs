﻿namespace Jolt
{
    internal static unsafe partial class Bindings
    {
        public static float JPH_ConvexShapeSettings_GetDensity(NativeHandle<JPH_ConvexShapeSettings> settings)
        {
            AssertInitialized();

            return UnsafeBindings.JPH_ConvexShapeSettings_GetDensity(settings);
        }

        public static void JPH_ConvexShapeSettings_SetDensity(NativeHandle<JPH_ConvexShapeSettings> settings, float density)
        {
            AssertInitialized();

            UnsafeBindings.JPH_ConvexShapeSettings_SetDensity(settings, density);
        }
    }
}
