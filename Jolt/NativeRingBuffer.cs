using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Jolt
{
    [StructLayout(LayoutKind.Sequential)]
    unsafe struct UnsafeRingQueueHeader<T> where T : unmanaged
    {
        public T* Ptr;
        public AllocatorManager.AllocatorHandle Allocator;

        public readonly int m_Capacity;
        public int m_Filled;
        public int m_Write;
        public int m_Read;

        public bool TryPeek(out T head)
        {
            if (m_Filled == 0)
            {
                head = default;
                return false;
            }

            head = Ptr[m_Read];
            return true;
        }

        public bool TryPeekTail(out T tail)
        {
            if (m_Filled == 0)
            {
                tail = default;
                return false;
            }

            tail = Ptr[(m_Write - 1 + m_Capacity) % m_Capacity];
            return true;
        }

        public void Clear()
        {
            m_Filled = 0;
            m_Write = 0;
            m_Read = 0;
        }

        public void SetLength(int length)
        {
            Assert.IsTrue(length >= 0 && length < m_Capacity);

            m_Filled = length;
            m_Write = (m_Read + length) % m_Capacity;
        }
    }

    [NativeContainer]
    public unsafe struct NativeRingBuffer : IDisposable
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        internal AtomicSafetyHandle m_Safety;
        internal static readonly SharedStatic<int> s_staticSafetyId = SharedStatic<int>.GetOrCreate<NativeRingBuffer>();
#endif
        
        [NativeDisableUnsafePtrRestriction]
        private UnsafeRingBuffer* m_Buffer;

        public readonly bool IsCreated
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_Buffer != null;
        }

        public readonly int Capacity => m_Buffer->Capacity;
        public readonly int Length => m_Buffer->Length;
        public readonly int AllocatedBufferLength => m_Buffer->AllocatedBufferLength;
        public readonly int BufferLength => m_Buffer->BufferLength;

        public NativeRingBuffer(
            int initialCapacity, int initialBlockSize, AllocatorManager.AllocatorHandle allocator)
        {
            this = default;
            var temp = allocator;
            Initialize(initialCapacity, initialBlockSize, ref temp);
        }

        internal void Initialize<U>(int initialCapacity, int initialBlockSize, ref U allocator) where U : unmanaged, AllocatorManager.IAllocator
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            CheckInitialCapacity(initialCapacity);

            m_Safety = CollectionHelper.CreateSafetyHandle(allocator.ToAllocator);

            CollectionHelper.SetStaticSafetyId<NativeRingBuffer>(ref m_Safety, ref s_staticSafetyId.Data);
#endif
            m_Buffer = UnsafeRingBuffer.Create(initialCapacity, initialBlockSize, ref allocator);
        }
        
        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        static void CheckInitialCapacity(int initialCapacity)
        {
            if (initialCapacity < 0)
                throw new ArgumentOutOfRangeException(nameof(initialCapacity), "Capacity must be >= 0");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        static void CheckTotalSize(int initialCapacity, long totalSize)
        {
            // Make sure we cannot allocate more than int.MaxValue (2,147,483,647 bytes)
            // because the underlying UnsafeUtility.Malloc is expecting a int.
            // TODO: change UnsafeUtility.Malloc to accept a UIntPtr length instead to match C++ API
            if (totalSize > int.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(initialCapacity), $"Capacity * sizeof(T) cannot exceed {int.MaxValue} bytes");
        }
        
        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (!AtomicSafetyHandle.IsDefaultValue(m_Safety))
            {
                AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
            }
#endif
            if (!IsCreated) return;
            CollectionHelper.DisposeSafetyHandle(ref m_Safety);

            UnsafeRingBuffer.Destroy(m_Buffer);
            m_Buffer = null;
        }

        public void Enqueue(void* readonlyData, int readonlyDataLength)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->Enqueue(readonlyData, readonlyDataLength);
        }

        public bool TryPeek(out void* ptr, out int length)
        {
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
            
            return m_Buffer->TryPeek(out ptr, out length);
        }

        public bool TryPeekTail(out void* ptr, out int length)
        {
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
            
            return m_Buffer->TryPeekTail(out ptr, out length);
        }

        public void Rollback(int length)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->Rollback(length);
        }

        public void Clear()
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->Clear();
        }
    }
}
