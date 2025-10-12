using System;
using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;

namespace Jolt
{
    public unsafe struct UnsafeRingBuffer : IDisposable
    {
        public struct Entry
        {
            public ulong SequenceLShift1;
            public int Bytes;
        }

        private int m_Capacity;
        public readonly int Capacity => m_Capacity;
        private int m_SlotCapacity;
        public readonly int SlotCapacity => m_SlotCapacity;
        internal Entry* m_Entries;
        public readonly bool IsCreated => m_Entries != null;

        [NativeDisableUnsafePtrRestriction] internal void* m_Buffer;
        internal int m_BufferLength;
        public readonly int BufferLength => m_BufferLength;
        
        private AllocatorManager.AllocatorHandle m_Allocator;

        public UnsafeRingBuffer(AllocatorManager.AllocatorHandle allocator, int capacity, int initialSlotCapacity)
        {
            m_Capacity = capacity;
            m_SlotCapacity = 0;
            m_Entries = AllocatorManager.Allocate<Entry>(allocator, capacity);
            UnsafeUtility.MemClear(m_Entries, sizeof(Entry) * m_Capacity);

            m_Buffer = null;
            m_BufferLength = 0;
            
            m_Allocator = allocator;

            EnsureSlotCapacity(initialSlotCapacity);
        }

        public void Dispose()
        {
            if (m_Buffer != null)
            {
                AllocatorManager.Free(m_Allocator, m_Buffer);
                m_Buffer = null;
            }

            if (m_Entries != null)
            {
                AllocatorManager.Free(m_Allocator, m_Entries);
                m_Entries = null;
            }

            m_Capacity = 0;
            m_SlotCapacity = 0;

            m_BufferLength = 0;
            m_Allocator = Allocator.Invalid;
        }

        public void EnsureSlotCapacity(int value)
        {
            var newBufferLength = math.ceilpow2(value * m_Capacity);
            if (newBufferLength <= m_BufferLength) return;

            var newBuffer = AllocatorManager.Allocate<byte>(m_Allocator, newBufferLength);
            if (m_Buffer != null && m_SlotCapacity > 0)
            {
                UnsafeUtility.MemCpyStride(newBuffer, value, m_Buffer, m_SlotCapacity, m_SlotCapacity, m_Capacity);
                AllocatorManager.Free(m_Allocator, m_Buffer);
                m_Buffer = null;
            }

            m_Buffer = newBuffer;
            m_BufferLength = newBufferLength;
            m_SlotCapacity = value;
        }
        
        public void Clear()
        {
            if (!IsCreated) return;
            UnsafeUtility.MemClear(m_Entries, sizeof(Entry) * m_Capacity);
        }

        public bool Contains(uint sequence)
        {
            var index = sequence % m_Capacity;
            var entry = m_Entries[index];
            return entry.SequenceLShift1 == 0 || entry.SequenceLShift1 >> 1 != sequence ;
        }

        public bool TryGetValue(uint sequence, out Span<byte> buffer)
        {
            var index = sequence % m_Capacity;
            var entry = m_Entries[index];
            if (entry.SequenceLShift1 == 0 || entry.SequenceLShift1 >> 1 != sequence)
            {
                buffer = Span<byte>.Empty;
                return false;
            }

            buffer = new Span<byte>((byte*)m_Buffer + m_SlotCapacity * index, entry.Bytes);
            return true;
        }

        public void Allocate(uint sequence, int bytes, out Span<byte> buffer)
        {
            CheckNull(m_Buffer);
            
            var index = sequence % m_Capacity;
            if (m_Entries[index].SequenceLShift1 > 0 && m_Entries[index].SequenceLShift1 >> 1 > sequence)
            {
                buffer = Span<byte>.Empty;
                return;
            }

            EnsureSlotCapacity(bytes);
            m_Entries[index].SequenceLShift1 = sequence << 1;
            m_Entries[index].Bytes = bytes;
            
            buffer = new Span<byte>((byte*)m_Buffer + m_SlotCapacity * index, bytes);
        }

        public bool FreeAt(uint sequence)
        {
            if (!Contains(sequence)) return false;
            var index = sequence % m_Capacity;
            FreeAtSlot((int)index);
            return true;
        }

        public void FreeAtSlot(int slot)
        {
            if (!IsCreated) return;
            m_Entries[slot].SequenceLShift1 = 0;
            m_Entries[slot].Bytes = 0;
        }
#region Internal
        internal static UnsafeRingBuffer* Create<U>(int initialCapacity, int initialBlockSize, ref U allocator)
            where U : unmanaged, AllocatorManager.IAllocator
        {
            var buffer = (UnsafeRingBuffer*)allocator.Allocate(UnsafeUtility.SizeOf<UnsafeRingBuffer>(),
                UnsafeUtility.AlignOf<UnsafeRingBuffer>(), 1);
            *buffer = new UnsafeRingBuffer(allocator.Handle, initialCapacity, initialBlockSize);
            return buffer;
        }

        public static void Destroy(UnsafeRingBuffer* buffer)
        {
            CheckNull(buffer);
            var allocator = buffer->m_Allocator;
            buffer->Dispose();
            AllocatorManager.Free(allocator, buffer);
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        internal static void CheckNull(void* listData)
        {
            if (listData == null)
            {
                throw new InvalidOperationException("UnsafeRingBuffer has yet to be created or has been destroyed!");
            }
        }
#endregion
    }
}
