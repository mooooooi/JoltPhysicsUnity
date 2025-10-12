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
    [NativeContainer]
    [DebuggerTypeProxy(typeof(DebuggerProxy<JPH_PhysicsSystemState>))]
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
        public readonly int SlotCapacity => m_Buffer->SlotCapacity;

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

        public void EnsureSlotCapacity(int value)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->EnsureSlotCapacity(value);
        }

        public void Clear()
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->Clear();
        }

        public bool Contains(uint sequence)
        {
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
            
            return m_Buffer->Contains(sequence);
        }

        public bool TryGetValue(uint sequence, out Span<byte> buffer)
        {
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
            
            return m_Buffer->TryGetValue(sequence, out buffer);
        }

        public void Allocate(uint sequence, int bytes, out Span<byte> buffer)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            m_Buffer->Allocate(sequence, bytes, out buffer);
        }

        public void Allocate(uint sequence, void* src, int bytes)
        {
            Allocate(sequence, bytes, out Span<byte> buffer);
            fixed (void* dst = buffer)
            {
                UnsafeUtility.MemCpy(dst, src, bytes);
            }
        }

        public bool FreeAt(uint sequence)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
            
            return m_Buffer->FreeAt(sequence);
        }

        public void FreeAtSlot(int slot)
        {
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);

            m_Buffer->FreeAtSlot(slot);
        }

        public class DebuggerProxy<T> where T : unmanaged
        {
            public struct Header
            {
                public uint length;
                public ulong hash;
            }

            public struct DisplayEntry
            {
                public Header header;
                public T value;
            }
            
            private NativeRingBuffer m_Buffer;
            public DebuggerProxy(NativeRingBuffer buffer)
            {
                m_Buffer = buffer;
            }
            
            public DisplayEntry?[] Items
            {
                get
                {
                    var items = new DisplayEntry?[m_Buffer.Capacity];
                    for (var i = 0; i < m_Buffer.Capacity; i++)
                    {
                        var entry = m_Buffer.m_Buffer->m_Entries[i];
                        if (entry.SequenceLShift1 == 0)
                        {
                            continue;
                        }

                        var ptr = (byte*)m_Buffer.m_Buffer->m_Buffer;
                        DisplayEntry displayEntry;
                        displayEntry.header.length = *(uint*)ptr;
                        displayEntry.header.hash = *(ulong*)(ptr + 8);
                        displayEntry.value = *(T*)(ptr + i * m_Buffer.SlotCapacity + 16);

                        items[i] = displayEntry;
                    }
                    return items;
                }
            }
        }
    }
}
