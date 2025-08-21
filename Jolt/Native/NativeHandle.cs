using System;
using System.Runtime.CompilerServices;
using Jolt.Internal;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Jolt
{
    /// <summary>
    /// A pointer to a native resource with optional safety checks against use-after-free.
    /// </summary>
    [NativeContainer]
    internal unsafe struct NativeHandle<T> : IDisposable, IEquatable<NativeHandle<T>> where T : unmanaged
    {
        [NativeDisableUnsafePtrRestriction]
        private T* m_Ptr;
        
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        internal AtomicSafetyHandle m_Safety;
        static readonly SharedStatic<int> s_SafetyId = SharedStatic<int>.GetOrCreate<NativeHandle<T>>();
#endif

        public bool IsCreated => m_Ptr != null;
        public nint RawValue => (nint)m_Ptr;

        public NativeHandle(T* ptr)
        {
            m_Ptr = ptr;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            // m_Safety = AtomicSafetyHandle.Create();
            m_Safety = AtomicSafetyHandle.Create();
            CollectionHelper.SetStaticSafetyId<NativeQueue<T>>(ref m_Safety, ref s_SafetyId.Data);
            JoltNativeInternalUtility.LeakRecord(new IntPtr(ptr), 1);
#endif
        }

        /// <summary>
        /// Create a NativeHandle to a new pointer with the same safety handle as this handle. If this handle is disposed of, the owned handle will throw if it is dereferenced.
        /// </summary>
        public NativeHandle<U> CreateOwnedHandle<U>(U* ptr) where U : unmanaged
        {
            #if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
            return new NativeHandle<U> { m_Ptr = ptr, m_Safety = m_Safety };
            #else
            return new NativeHandle<U> { ptr = ptr };
            #endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly NativeHandle<U> Reinterpret<U>() where U : unmanaged
        {
            #if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
            return new NativeHandle<U> { m_Ptr = (U*) m_Ptr, m_Safety = m_Safety };
            #else
            return new NativeHandle<U> { ptr = (U*) ptr };
            #endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly T* IntoPointer()
        {
            #if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckExistsAndThrow(in m_Safety);
            #endif

            return m_Ptr;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator T* (NativeHandle<T> handle)
        {
            return handle.IntoPointer();
        }

        #region IDisposable

        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (!AtomicSafetyHandle.IsDefaultValue(m_Safety))
            {
                AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
            }
            AtomicSafetyHandle.Release(m_Safety);
            JoltNativeInternalUtility.LeakErase(new IntPtr(m_Ptr));
#endif

            m_Ptr = null;
        }

        #endregion

        #region IEquatable

        public bool Equals(NativeHandle<T> other)
        {
            return m_Ptr == other.m_Ptr;
        }

        public override bool Equals(object obj)
        {
            return obj is NativeHandle<T> other && Equals(other);
        }

        public override int GetHashCode()
        {
            return ((nint)m_Ptr).GetHashCode();
        }

        public static bool operator ==(NativeHandle<T> lhs, NativeHandle<T> rhs)
        {
            return lhs.Equals(rhs);
        }

        public static bool operator !=(NativeHandle<T> lhs, NativeHandle<T> rhs)
        {
            return !lhs.Equals(rhs);
        }

        #endregion
    }
}
