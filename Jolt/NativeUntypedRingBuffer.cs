using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
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

    public unsafe struct UnsafeUntypedRingBuffer : IDisposable
    {
        private struct Entry
        {
            public int start;
            public int length;
        }
        
        private UnsafeRingQueue<Entry> m_Entries;
        
        private int m_AllocatedBufferLength;
        private int m_BufferLength;
        private void* m_Buffer;
        
        private readonly Allocator m_Allocator;
        
        public readonly Allocator Allocator => m_Allocator;
        public readonly int Capacity => m_Entries.Capacity;
        public int Length => m_Entries.Length;
        
        public readonly int AllocatedBufferLength => m_AllocatedBufferLength;
        public readonly int BufferLength => m_BufferLength;
        
        public UnsafeUntypedRingBuffer(Allocator allocator, int capacity, int initialBlockSize)
        {
            m_Entries = new UnsafeRingQueue<Entry>(capacity, allocator);
            m_AllocatedBufferLength = 0;
            m_BufferLength = NextPowerOfTwo(capacity * initialBlockSize);
            m_Buffer = UnsafeUtility.MallocTracked(
                m_BufferLength, 2, allocator, 0);
    
            m_Allocator = allocator;
        }
        
        public void Dispose()
        {
            m_Entries.Dispose();
            m_BufferLength = 0;
            UnsafeUtility.FreeTracked(m_Buffer, m_Allocator);
            m_Buffer = null;
        }
        
        public static int NextPowerOfTwo(int value)
        {
            --value;
            value |= value >> 16 /*0x10*/;
            value |= value >> 8;
            value |= value >> 4;
            value |= value >> 2;
            value |= value >> 1;
            return value + 1;
        }
        
        private void EnsureRemaining(int required)
        {
            var remain = m_BufferLength - m_AllocatedBufferLength;
            if (remain >= required) return;
            
            var newBufferLength = NextPowerOfTwo(m_AllocatedBufferLength + required);
            var newBuffer = UnsafeUtility.MallocTracked(newBufferLength, 2, Allocator, 0);
            
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);

            var headOffset = 0;
            if (entriesHeader.TryPeek(out var head))
            {
                headOffset = head.start + head.length;
            }
            
            if (headOffset + m_AllocatedBufferLength < m_BufferLength)
            {
                UnsafeUtility.MemCpy(newBuffer, (byte*)m_Buffer + headOffset, m_AllocatedBufferLength);
            }
            else
            {
                var segmentLength = m_BufferLength - headOffset;
                UnsafeUtility.MemCpy(newBuffer, (byte*)m_Buffer + headOffset, segmentLength);
                UnsafeUtility.MemCpy((byte*)newBuffer + segmentLength, m_Buffer, m_AllocatedBufferLength - segmentLength);
            }
            
            
            Debug.Log($"Resize: {m_BufferLength} -> {newBufferLength}");
            UnsafeUtility.FreeTracked(m_Buffer, m_Allocator);
            m_Buffer = newBuffer;
            m_BufferLength = newBufferLength;
            
            var sum = 0;
            for (var i = 0; i < entriesHeader.m_Filled; i++)
            {
                ref var entry = ref entriesHeader.Ptr[(i + entriesHeader.m_Read) % entriesHeader.m_Capacity];
                entry.start = sum;
                sum += entry.length;
            }
            
            Assert.AreEqual(sum, m_AllocatedBufferLength);
            
        }
    
        public void Enqueue(void* readonlyData, int readonlyDataLength)
        {
            if (m_Entries.Length == m_Entries.Capacity)
            {
                var entry = m_Entries.Dequeue();
                m_AllocatedBufferLength -= entry.length;
                Assert.IsTrue(m_AllocatedBufferLength >= 0);
            }
            
            EnsureRemaining(readonlyDataLength);
            
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            
            var start = 0;
            if (entriesHeader.TryPeekTail(out var tail))
            {
                start = (tail.start + tail.length) % m_BufferLength;
            }
            
            if (start + readonlyDataLength < m_BufferLength)
            {
                UnsafeUtility.MemCpy((byte*)m_Buffer + start, readonlyData, readonlyDataLength);
            }
            else
            {
                var segmentLength = m_BufferLength - start;
                UnsafeUtility.MemCpy((byte*)m_Buffer + start, readonlyData, segmentLength);
                UnsafeUtility.MemCpy(m_Buffer, (byte*)readonlyData + segmentLength, readonlyDataLength - segmentLength);
            }
            
            m_Entries.Enqueue(new Entry()
            {
                start = start, length = readonlyDataLength
            });
            m_AllocatedBufferLength += readonlyDataLength;
        }

        public bool TryPeek(out void* ptr, out int length)
        {
            if (m_Entries.Length == 0)
            {
                ptr = null;
                length = 0;
                return false;
            }

            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            Assert.IsTrue(entriesHeader.TryPeek(out var head));

            ptr = (byte*)m_Buffer + head.start;
            length = head.length;
            
            return true;
        }

        public void Rollback(int length)
        {
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            entriesHeader.SetLength(length);

            m_AllocatedBufferLength = 0;
            for (var i = 0; i < entriesHeader.m_Filled; i++)
            {
                ref var entry = ref entriesHeader.Ptr[(i + entriesHeader.m_Read) % entriesHeader.m_Capacity];
                m_AllocatedBufferLength += entry.length;
            }
        }

        public void Clear()
        {
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            entriesHeader.Clear();
        }
    }
    
    [NativeContainer]
    public unsafe struct NativeUntypedRingBuffer : IDisposable
    {
        [NativeDisableUnsafePtrRestriction]
        private UnsafeUntypedRingBuffer* m_Buffer;
        
        public NativeUntypedRingBuffer(
            Allocator allocator, int capacity, int initialBlockSize)
        {
            m_Buffer = (UnsafeUntypedRingBuffer*)UnsafeUtility.MallocTracked(
                UnsafeUtility.SizeOf<IntPtr>(), UnsafeUtility.AlignOf<IntPtr>(), allocator, 0);
            *m_Buffer = new UnsafeUntypedRingBuffer(allocator, capacity, initialBlockSize);
        }
        
        public void Dispose()
        {
            var allocator = m_Buffer->Allocator;
            m_Buffer->Dispose();
            UnsafeUtility.FreeTracked(m_Buffer, allocator);
            m_Buffer = null;
        }
    }
}
