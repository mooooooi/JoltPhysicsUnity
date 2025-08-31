using System.Runtime.InteropServices;
using Unity.Mathematics;

using static Jolt.Bindings;

namespace Jolt
{
    [StructLayout(LayoutKind.Sequential), ExpectedStructSize(typeof(JPH_SliderConstraintSettings))]
    public struct SliderConstraintSettings
    {
        /// <summary>
        /// Create a new instance initialized with the default values.
        /// </summary>
        public static SliderConstraintSettings Create()
        {
            var result = new SliderConstraintSettings();
            JPH_SliderConstraintSettings_Init(ref result);
            return result;
        }

        private ConstraintSettings @base;
        
        public NativeBool Enabled
        {
            get => @base.Enabled;
            set => @base.Enabled = value;
        }

        public uint ConstraintPriority 
        {
            get => @base.ConstraintPriority;
            set => @base.ConstraintPriority = value;
        }

        public uint NumVelocityStepsOverride 
        {
            get => @base.NumVelocityStepsOverride;
            set => @base.NumVelocityStepsOverride = value;
        }

        public uint NumPositionStepsOverride 
        {
            get => @base.NumPositionStepsOverride;
            set => @base.NumPositionStepsOverride = value;
        }

        public float DrawConstraintSize 
        {
            get => @base.DrawConstraintSize;
            set => @base.DrawConstraintSize = value;
        }

        public ulong UserData 
        {
            get => @base.UserData;
            set => @base.UserData = value;
        }

        public ConstraintSpace Space;

        public NativeBool AutoDetectPoint;
        
        public rvec3 Point1;
        
        public float3 SliderAxis1;

        public float3 NormalAxis1;

        public rvec3 Point2;

        public float3 SliderAxis2;

        public float3 NormalAxis2;

        public float LimitsMin;

        public float LimitsMax;

        public SpringSettings LimitsSpringSettings;

        public float MaxFrictionForce;

        public MotorSettings MotorSettings;

        public void SetSliderAxis(float3 axis)
        {
            JPH_SliderConstraintSettings_SetSliderAxis(ref this, axis);
        }
    }
}
