using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Jolt
{
    [StructLayout(LayoutKind.Explicit, Size = 16)]
    public struct BlobAssetHeader
    {
        [FieldOffset(0)]  public uint Length;
        [FieldOffset(8)] public ulong Hash;
    }
    
    public unsafe ref struct NativeBlobBuilderArray<T> where T : struct
    {
        private void* m_data;
        private int m_length;

        /// <summary>
        /// For internal, <see cref="BlobBuilder"/>, use only.
        /// </summary>
        /// <param name="data">The pointer to the allocated buffer.</param>
        /// <param name="length">The length of the buffer.</param>
        public NativeBlobBuilderArray(void* data, int length)
        {
            m_data = data;
            m_length = length;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private void CheckIndexOutOfRange(int index)
        {
            if (0 > index || index >= m_length)
                throw new IndexOutOfRangeException(string.Format("Index {0} is out of range of '{1}' Length.", (object)index, (object)this.m_length));
        }

        /// <summary>
        /// Array index accessor for the elements in the array.
        /// </summary>
        /// <param name="index">The sequential index of an array item.</param>
        /// <exception cref="IndexOutOfRangeException">Thrown when index is less than zero or greater than the length of the array (minus one).</exception>
        public ref T this[int index]
        {
            get
            {
                CheckIndexOutOfRange(index);
                return ref UnsafeUtility.ArrayElementAsRef<T>(m_data, index);
            }
        }

        /// <summary>
        /// Reports the number of elements in the array.
        /// </summary>
        public int Length
        {
            get { return m_length; }
        }

        /// <summary>
        /// Provides a pointer to the data stored in the array.
        /// </summary>
        /// <remarks>You can only call this function in an [unsafe context].
        /// [unsafe context]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <returns>A pointer to the first element in the array.</returns>
        public void* GetUnsafePtr()
        {
            return m_data;
        }
    }
        
    public unsafe struct NativeBlobBuilder : IDisposable
    {
        AllocatorManager.AllocatorHandle m_allocator;
        NativeList<BlobAllocation> m_allocations;
        NativeList<OffsetPtrPatch> m_patches;
        int m_currentChunkIndex;
        int m_chunkSize;

        struct BlobAllocation
        {
            public int size;
            public byte* p;
        }

        struct BlobDataRef
        {
            public int allocIndex;
            public int offset;
        }

        struct OffsetPtrPatch
        {
            public int* offsetPtr;
            public BlobDataRef target;
            public int length; // if length != 0 this is an array patch and the length should be patched
        }

        /// <summary>
        /// Constructs a BlobBuilder object.
        /// </summary>
        /// <param name="allocator">The type of allocator to use for the BlobBuilder's internal, temporary data. Use
        /// <see cref="Unity.Collections.Allocator.Temp"/> unless the BlobBuilder exists across more than four Unity frames.</param>
        /// <param name="chunkSize">(Optional) The minimum amount of memory to allocate while building an asset.
        /// The default value should suit most use cases. A smaller chunkSize results in more allocations; a larger
        /// chunkSize could increase the BlobBuilder's total memory allocation (which is freed when you dispose of
        /// the BlobBuilder.</param>
        public NativeBlobBuilder(AllocatorManager.AllocatorHandle allocator, int chunkSize = 65536)
        {
            m_allocator = allocator;
            m_allocations = new NativeList<BlobAllocation>(16, m_allocator);
            m_patches = new NativeList<OffsetPtrPatch>(16, m_allocator);
            m_chunkSize = CollectionHelper.Align(chunkSize, 16);
            m_currentChunkIndex = -1;
        }

        /// <summary>
        /// Creates the top-level fields of a single blob asset.
        /// </summary>
        /// <remarks>
        /// This function allocates memory for the top-level fields of a blob asset and returns a reference to it. Use
        /// this root reference to initialize field values and to allocate memory for arrays and structs.
        /// </remarks>
        /// <typeparam name="T">A struct that defines the structure of the blob asset.</typeparam>
        /// <returns>A reference to the blob data under construction.</returns>
        public ref T ConstructRoot<T>() where T : struct
        {
            var allocation = Allocate(UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>());
            return ref UnsafeUtility.AsRef<T>(AllocationToPointer(allocation));
        }

        /// <summary>
        /// Copies an array of structs to an array in a blob asset after allocating the necessary memory.
        /// </summary>
        /// <param name="blobArray">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="data">An array  containing structs of type <typeparamref name="T"/>.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly constructed array as a mutable BlobBuilderArray instance.</returns>
        public NativeBlobBuilderArray<T> Construct<T>(ref JPH_BlobArray<T> blobArray, params T[] data) where T : unmanaged
        {
            var constructBlobArray = Allocate(ref blobArray, data.Length);
            for (int i = 0; i != data.Length; i++)
                constructBlobArray[i] = data[i];
            return constructBlobArray;
        }

        /// <summary>
        /// Allocates enough memory to store <paramref name="length"/> elements of struct <typeparamref name="T"/>.
        /// </summary>
        /// <param name="ptr">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="length">The number of elements to allocate.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly allocated array as a mutable BlobBuilderArray instance.</returns>
        public NativeBlobBuilderArray<T> Allocate<T>(ref JPH_BlobArray<T> ptr, int length) where T : unmanaged
        {
            return Allocate(ref ptr, length, UnsafeUtility.AlignOf<T>());
        }

        /// <summary>
        /// Allocates enough memory to store <paramref name="length"/> elements of struct <typeparamref name="T"/>.
        /// </summary>
        /// <param name="ptr">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="length">The number of elements to allocate.</param>
        /// <param name="alignment">The alignment of the allocated memory.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly allocated array as a mutable BlobBuilderArray instance.</returns>
        public NativeBlobBuilderArray<T> Allocate<T>(ref JPH_BlobArray<T> ptr, int length, int alignment) where T : unmanaged
        {
            if (length <= 0)
                return new NativeBlobBuilderArray<T>(null, 0);

            if (!math.ispow2(alignment))
                throw new ArgumentException("Alignment must be a power of two");

            if (alignment > 16)
                throw new ArgumentException("Alignment larger than 16 is not supported");

            var offsetPtr = (int*)UnsafeUtility.AddressOf(ref ptr.mOffsetPtr);

            ValidateAllocation(offsetPtr);

            var allocation = Allocate(UnsafeUtility.SizeOf<T>() * length, alignment);

            var patch = new OffsetPtrPatch
            {
                offsetPtr = offsetPtr,
                target = allocation,
                length = length
            };

            m_patches.Add(patch);
            return new NativeBlobBuilderArray<T>(AllocationToPointer(allocation), length);
        }


        struct SortedIndex : IComparable<SortedIndex>
        {
            public byte* p;
            public int index;
            public int CompareTo(SortedIndex other)
            {
                return ((ulong)p).CompareTo((ulong)other.p);
            }
        }
        
        public BlobAssetHeader* Flush(AllocatorManager.AllocatorHandle allocator)
        {
            // Avoid crash when there are no chunks (DOTS-8681)
            if (m_currentChunkIndex != -1)
            {
                //Align last chunk upwards so all chunks are 16 byte aligned
                AlignChunk(m_currentChunkIndex);
            }
        
            var offsets = new NativeArray<int>(m_allocations.Length + 1, Allocator.Temp);
            var sortedAllocs = new NativeArray<SortedIndex>(m_allocations.Length, Allocator.Temp);
        
            offsets[0] = 0;
            for (int i = 0; i < m_allocations.Length; ++i)
            {
                offsets[i + 1] = offsets[i] + m_allocations[i].size;
                sortedAllocs[i] = new SortedIndex {p = m_allocations[i].p, index = i};
            }
            int dataSize = offsets[m_allocations.Length];
        
            sortedAllocs.Sort();
            var sortedPatches = new NativeArray<SortedIndex>(m_patches.Length, Allocator.Temp);
            for (int i = 0; i < m_patches.Length; ++i)
                sortedPatches[i] = new SortedIndex {p = (byte*)m_patches[i].offsetPtr, index = i};
            sortedPatches.Sort();
        
            byte* buffer = (byte*)UnsafeUtility.MallocTracked(sizeof(BlobAssetHeader) + dataSize, 16, allocator.ToAllocator, 0);
            byte* data = buffer + sizeof(BlobAssetHeader);
        
            for (int i = 0; i < m_allocations.Length; ++i)
                UnsafeUtility.MemCpy(data + offsets[i], m_allocations[i].p, m_allocations[i].size);
        
            int iAlloc = 0;
            var allocStart = m_allocations[sortedAllocs[0].index].p;
            var allocEnd = allocStart + m_allocations[sortedAllocs[0].index].size;
        
            for (int i = 0; i < m_patches.Length; ++i)
            {
                int patchIndex = sortedPatches[i].index;
                int* offsetPtr = (int*)sortedPatches[i].p;
        
                while (offsetPtr >= allocEnd)
                {
                    ++iAlloc;
                    allocStart = m_allocations[sortedAllocs[iAlloc].index].p;
                    allocEnd = allocStart + m_allocations[sortedAllocs[iAlloc].index].size;
                }
        
                var patch = m_patches[patchIndex];
        
                int offsetPtrInData = offsets[sortedAllocs[iAlloc].index] + (int)((byte*)offsetPtr - allocStart);
                int targetPtrInData = offsets[patch.target.allocIndex] + patch.target.offset;
        
                *(int*)(data + offsetPtrInData) = targetPtrInData - offsetPtrInData;
                if (patch.length != 0)
                {
                    *(int*)(data + offsetPtrInData + 4) = patch.length;
                }
            }
        
            sortedPatches.Dispose();
            sortedAllocs.Dispose();
            offsets.Dispose();
        
            BlobAssetHeader* header = (BlobAssetHeader*)buffer;
            *header = new BlobAssetHeader();
            header->Length = (uint)dataSize;
            // @TODO use 64bit hash
            header->Hash = HashBytes(data, header->Length);
            
            return header;
        }

        public static ulong HashBytes(void* inData, uint inSize, ulong inSeed = 0xcbf29ce484222325UL)
        {
            var hash = inSeed;
            for (byte* data = (byte*)inData; data < (byte*)inData + inSize; ++data)
            {
                hash ^= *data;
                hash *= 0x100000001b3UL;
            }
            return hash;
        }

        void* AllocationToPointer(BlobDataRef blobDataRef)
        {
            return m_allocations[blobDataRef.allocIndex].p + blobDataRef.offset;
        }

        BlobAllocation EnsureEnoughRoomInChunk(int size, int alignment)
        {
            if (m_currentChunkIndex == -1)
                return AllocateNewChunk();

            var alloc = m_allocations[m_currentChunkIndex];
            int startOffset = CollectionHelper.Align(alloc.size, alignment);
            if (startOffset + size > m_chunkSize)
                return AllocateNewChunk();

            UnsafeUtility.MemClear(alloc.p + alloc.size, startOffset - alloc.size);

            alloc.size = startOffset;
            return alloc;
        }

        BlobDataRef Allocate(int size, int alignment)
        {
            if (size > m_chunkSize)
            {
                size = CollectionHelper.Align(size, 16);
                var allocIndex = m_allocations.Length;
                var mem = (byte*)UnsafeUtility.MallocTracked(size, alignment, m_allocator.ToAllocator, 0);
                UnsafeUtility.MemClear(mem, size);
                m_allocations.Add(new BlobAllocation {p = mem, size = size});
                return new BlobDataRef {allocIndex = allocIndex, offset = 0};
            }

            BlobAllocation alloc = EnsureEnoughRoomInChunk(size, alignment);

            var offset = alloc.size;
            UnsafeUtility.MemClear(alloc.p + alloc.size, size);
            alloc.size += size;
            m_allocations[m_currentChunkIndex] = alloc;
            return new BlobDataRef {allocIndex = m_currentChunkIndex, offset = offset};
        }

        BlobAllocation AllocateNewChunk()
        {
            // align size of last chunk to 16 bytes so chunks can be concatenated without breaking alignment
            if (m_currentChunkIndex != -1)
            {
                AlignChunk(m_currentChunkIndex);
            }

            m_currentChunkIndex = m_allocations.Length;
            var alloc = new BlobAllocation {p = (byte*)UnsafeUtility.MallocTracked(m_chunkSize, 16, m_allocator.ToAllocator, 0), size = 0};
            m_allocations.Add(alloc);
            return alloc;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        void ValidateAllocation(void* address)
        {
            // ValidateAllocation is most often called with data in recently allocated allocations
            // so this searches backwards
            for (int i = m_allocations.Length - 1; i >= 0; --i)
            {
                var allocation = m_allocations[i];
                if (address >= allocation.p && address < (allocation.p + allocation.size))
                    return;
            }

            throw new InvalidOperationException("The BlobArray passed to Allocate was not allocated by this BlobBuilder or the struct that embeds it was copied by value instead of by ref.");
        }

        private bool GetPatchTarget(void* address, out BlobDataRef blobDataRef)
        {
            // Search backwards; most likely to be referring to objects that were most recently added..
            for (int i = m_allocations.Length-1; i >= 0; i--)
            {
                var allocation = m_allocations[i];
                if (address >= allocation.p && address < (allocation.p + allocation.size))
                {
                    blobDataRef = new BlobDataRef
                    {
                        allocIndex = i,
                        offset = (int)((byte*)address - allocation.p),
                    };

                    return true;
                }
            }

            blobDataRef = default;
            return false;
        }

        /// <summary>
        /// Returns true if this BlobBuilder has been allocated.
        /// </summary>
        public readonly bool IsCreated
            => m_allocations.IsCreated;

        /// <summary>
        /// Disposes of this BlobBuilder instance and frees its temporary memory allocations.
        /// </summary>
        /// <remarks>Call `Dispose()` after calling <see cref="CreateBlobAssetReference{T}"/>.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < m_allocations.Length; ++i)
                UnsafeUtility.FreeTracked(m_allocations[i].p, m_allocator.ToAllocator);
            m_allocations.Dispose();
            m_patches.Dispose();
        }

        void AlignChunk(int chunkIndex)
        {
            var chunk = m_allocations[chunkIndex];
            var oldSize = chunk.size;
            chunk.size = CollectionHelper.Align(chunk.size, 16);
            m_allocations[chunkIndex] = chunk;
            UnsafeUtility.MemSet(chunk.p + oldSize, 0, chunk.size-oldSize);
        }
    }
}
