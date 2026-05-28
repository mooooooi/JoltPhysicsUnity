using System;
using Higo.Gameplay.Runtime.Blobs;
using Unity.Collections;
using GameplayBlobAssetHeader = Higo.Gameplay.Runtime.Blobs.BlobAssetHeader;
using GameplayBlobBuilder = Higo.Gameplay.Runtime.Blobs.BlobBuilder;

namespace Jolt
{
    [Obsolete("Use Higo.Gameplay.Runtime.Blobs.BlobBuilder. This shim exists only for Jolt interop serializers that still use JPH_BlobArray<T>.")]
    public unsafe ref struct NativeBlobBuilderArray<T> where T : struct
    {
        private BlobBuilderArray<T> m_Array;

        internal NativeBlobBuilderArray(BlobBuilderArray<T> array)
        {
            m_Array = array;
        }

        public ref T this[int index] => ref m_Array[index];
        public int Length => m_Array.Length;
        public void* GetUnsafePtr() => m_Array.GetUnsafePtr();
    }

    [Obsolete("Use Higo.Gameplay.Runtime.Blobs.BlobBuilder. This shim exists only for Jolt interop serializers that still use JPH_BlobArray<T>.")]
    public unsafe struct NativeBlobBuilder : IDisposable
    {
        private GameplayBlobBuilder m_Builder;

        public NativeBlobBuilder(AllocatorManager.AllocatorHandle allocator, int chunkSize = 65536)
        {
            m_Builder = new GameplayBlobBuilder(allocator, chunkSize);
        }

        public bool IsCreated => m_Builder.IsCreated;

        public ref T ConstructRoot<T>() where T : unmanaged
        {
            return ref m_Builder.ConstructRoot<T>();
        }

        public NativeBlobBuilderArray<T> Construct<T>(ref JPH_BlobArray<T> blobArray, params T[] data)
            where T : unmanaged
        {
            var array = Allocate(ref blobArray, data.Length);
            for (var i = 0; i < data.Length; i++)
                array[i] = data[i];
            return array;
        }

        public NativeBlobBuilderArray<T> Allocate<T>(ref JPH_BlobArray<T> blobArray, int length)
            where T : unmanaged
        {
            return Allocate(ref blobArray, length, Unity.Collections.LowLevel.Unsafe.UnsafeUtility.AlignOf<T>());
        }

        public NativeBlobBuilderArray<T> Allocate<T>(ref JPH_BlobArray<T> blobArray, int length, int alignment)
            where T : unmanaged
        {
            return new NativeBlobBuilderArray<T>(
                m_Builder.AllocateArray<T>(ref blobArray.mOffsetPtr, ref blobArray.mLength, length, alignment));
        }

        public GameplayBlobAssetHeader* Flush(AllocatorManager.AllocatorHandle allocator)
        {
            return m_Builder.Flush(allocator);
        }

        public static ulong HashBytes(void* data, uint size, ulong seed = 0xcbf29ce484222325UL)
        {
            return BlobUtility.HashBytes(data, size, seed);
        }

        public void Dispose()
        {
            m_Builder.Dispose();
        }
    }
}
