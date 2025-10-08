using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Assertions;

namespace Jolt
{
    public unsafe struct UnsafeRingBuffer : IDisposable
    {
        private struct Entry
        {
            public int padding;
            public int start;
            public int length;
        }
        
        private UnsafeRingQueue<Entry> m_Entries;

        private int m_PayloadAllocatedBytes;
        private int m_PaddingAllocatedBytes;
        private int m_BufferLength;
        
        [NativeDisableUnsafePtrRestriction]
        private void* m_Buffer;
        
        private readonly AllocatorManager.AllocatorHandle m_Allocator;
        
        internal readonly AllocatorManager.AllocatorHandle Allocator => m_Allocator;
        public readonly int Capacity => m_Entries.Capacity;
        public readonly int Length => m_Entries.Length;
        
        public readonly int AllocatedBufferLength => m_PayloadAllocatedBytes + m_PaddingAllocatedBytes;
        public readonly int BufferLength => m_BufferLength;
        
        public UnsafeRingBuffer(AllocatorManager.AllocatorHandle allocator, int capacity, int initialBlockSize)
        {
            m_Entries = new UnsafeRingQueue<Entry>(capacity, allocator);
            m_PayloadAllocatedBytes = 0;
            m_PaddingAllocatedBytes = 0;
            m_BufferLength = NextPowerOfTwo(capacity * initialBlockSize);
            m_Buffer = UnsafeUtility.MallocTracked(
                m_BufferLength, 2, allocator.ToAllocator, 0);
    
            m_Allocator = allocator;
        }
        
        public void Dispose()
        {
            m_Entries.Dispose();
            m_BufferLength = 0;
            UnsafeUtility.FreeTracked(m_Buffer, m_Allocator.ToAllocator);
            m_Buffer = null;
        }
        
        private static int NextPowerOfTwo(int value)
        {
            --value;
            value |= value >> 16 /*0x10*/;
            value |= value >> 8;
            value |= value >> 4;
            value |= value >> 2;
            value |= value >> 1;
            return value + 1;
        }
        
        private void EnsureRemaining(int required, out int start, out int padding)
        {
            var headOffset = 0;
            
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            if (entriesHeader.TryPeek(out var head))
            {
                headOffset = head.start + head.length;
            }

            var remainToTail = m_BufferLength - headOffset;
            if (remainToTail >= required)
            {
                start = headOffset;
                padding = 0;
            }

            if (m_BufferLength - remainToTail - AllocatedBufferLength >= required)
            {
                start = 0;
                padding = remainToTail;
            }
            
            var oldBuffer = (byte*)m_Buffer;
            
            var newBufferLength = NextPowerOfTwo(m_PayloadAllocatedBytes + required);
            var newBuffer = (byte*)UnsafeUtility.MallocTracked(newBufferLength, 2, Allocator.ToAllocator, 0);

            m_PayloadAllocatedBytes = 0;
            m_PaddingAllocatedBytes = 0;
            
            for (var i = 0; i < entriesHeader.m_Filled; i++)
            {
                ref var entry = ref entriesHeader.Ptr[(i + entriesHeader.m_Read) % entriesHeader.m_Capacity];
                UnsafeUtility.MemCpy(newBuffer + m_PayloadAllocatedBytes, oldBuffer + entry.start, entry.length);

                entry.padding = 0;
                entry.start = m_PayloadAllocatedBytes;
                    
                m_PayloadAllocatedBytes += entry.length;
            }

            start = m_PayloadAllocatedBytes;
            padding = 0;

            m_Buffer = newBuffer;
            m_BufferLength = newBufferLength;
            
            UnsafeUtility.FreeTracked(oldBuffer, m_Allocator.ToAllocator);
        }
    
        public void Enqueue(void* readonlyData, int readonlyDataLength)
        {
            if (m_Entries.Length == m_Entries.Capacity)
            {
                var entry = m_Entries.Dequeue();
                m_PayloadAllocatedBytes -= entry.length;
                m_PaddingAllocatedBytes -= entry.padding;
                
                Assert.IsTrue(m_PayloadAllocatedBytes >= 0);
                Assert.IsTrue(m_PaddingAllocatedBytes >= 0);
            }
            
            EnsureRemaining(readonlyDataLength, out var start, out var padding);
            
            UnsafeUtility.MemCpy((byte*)m_Buffer + start, readonlyData, readonlyDataLength);
            
            m_Entries.Enqueue(new Entry()
            {
                start = start, length = readonlyDataLength, padding = padding
            });
            
            m_PayloadAllocatedBytes += readonlyDataLength;
            m_PaddingAllocatedBytes += padding;
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
            var ret = entriesHeader.TryPeek(out var head);
            Assert.IsTrue(ret);

            ptr = (byte*)m_Buffer + head.start;
            length = head.length;
            
            return true;
        }
        
        public bool TryPeekTail(out void* ptr, out int length)
        {
            if (m_Entries.Length == 0)
            {
                ptr = null;
                length = 0;
                return false;
            }

            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            var ret = entriesHeader.TryPeekTail(out var head);
            Assert.IsTrue(ret);

            ptr = (byte*)m_Buffer + head.start;
            length = head.length;
            
            return true;
        }

        public void Rollback(int length)
        {
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            entriesHeader.SetLength(length);

            m_PayloadAllocatedBytes = 0;
            m_PaddingAllocatedBytes = 0;
            
            for (var i = 0; i < entriesHeader.m_Filled; i++)
            {
                ref var entry = ref entriesHeader.Ptr[(i + entriesHeader.m_Read) % entriesHeader.m_Capacity];
                m_PayloadAllocatedBytes += entry.length;
                m_PaddingAllocatedBytes += entry.padding;
            }
        }

        public void Clear()
        {
            ref var entriesHeader = ref UnsafeUtility.As<UnsafeRingQueue<Entry>, UnsafeRingQueueHeader<Entry>>(ref m_Entries);
            entriesHeader.Clear();
        }
    }
}
