using System;
using System.Threading;

namespace Jolt
{
    public class NativeTypeNameAttribute : Attribute
    {
        public readonly string Value;

        public NativeTypeNameAttribute(string value)
        {
            Value = value;
        }
    }
}
