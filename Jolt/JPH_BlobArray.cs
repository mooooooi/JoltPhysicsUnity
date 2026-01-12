using System;
using Unity.Collections.LowLevel.Unsafe;

namespace Jolt
{
    public struct JPH_BlobArray<T> where T : unmanaged
    {
        private int mOffsetPtr;
        private int mLength;

        public unsafe T this[int index]
        {
            get
            {
                CheckRange(index);
                return *(T*)((byte*)UnsafeUtility.AddressOf(ref mOffsetPtr) + mOffsetPtr);
            }

            set
            {
                CheckRange(index);
                *(T*)((byte*)UnsafeUtility.AddressOf(ref mOffsetPtr) + mOffsetPtr) = value;
            }
        }

        private void CheckRange(int index)
        {
            if (index < 0 || index >= mLength)
                throw new IndexOutOfRangeException();
        }
    }
}
