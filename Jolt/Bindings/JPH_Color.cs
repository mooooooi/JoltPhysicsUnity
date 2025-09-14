using System.Runtime.InteropServices;
using UnityEngine;

namespace Jolt
{
    [StructLayout(LayoutKind.Explicit)]
    public struct JPH_Color
    {
        [FieldOffset(0)]
        public uint Value;
        [FieldOffset(0)]
        public byte R;
        [FieldOffset(1)]
        public byte G;
        [FieldOffset(2)]
        public byte B;
        [FieldOffset(3)]
        public byte A;

        public static implicit operator Color(JPH_Color source)
        {
            return new Color(source.R / 255f, source.G / 255f, source.B / 255f, source.A / 255f);
        }

        public static implicit operator JPH_Color(Color source)
        {
            return new JPH_Color
            {
                R = (byte)(source.r * 255),
                G = (byte)(source.r * 255), 
                B = (byte)(source.r * 255), 
                A = (byte)(source.r * 255)
            };
        }
    }
}
