using System;
using System.Diagnostics;
using Unity.Collections.LowLevel.Unsafe;

namespace Jolt
{
    public struct JPH_BlobArray<T> where T : unmanaged
    {
        internal int mOffsetPtr;
        internal int mLength;

        public int Length => mLength;

        public unsafe ref T this[int index]
        {
            get
            {
                CheckRange(index);
                var ptr = (T*)((byte*)UnsafeUtility.AddressOf(ref mOffsetPtr) + mOffsetPtr);
                return ref ptr[index];
            }
        }

        private void CheckRange(int index)
        {
            if (index < 0 || index >= mLength)
                throw new IndexOutOfRangeException();
        }
    }
}
