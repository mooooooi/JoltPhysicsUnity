using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Jolt
{
    public interface ISerializable
    {
        void Serialize(ref DataStreamWriter writer);
        void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader);
    }
    
    public interface IPackedSerializable
    {
        void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model);
        void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model);
    }
    
    public interface IPackedDeltaSerializable<T>
    {
        void Serialize(ref T baseline, ref DataStreamWriter writer, in StreamCompressionModel model);
        void Deserialize(ref T baseline, NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model);
    }
    
    public static class SerializerUtility
    {
        [StructLayout(LayoutKind.Explicit)]
        internal struct UIntFloat
        {
            [FieldOffset(0)] public float floatValue;

            [FieldOffset(0)] public uint intValue;

            [FieldOffset(0)] public double doubleValue;

            [FieldOffset(0)] public ulong longValue;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFloat3(ref this DataStreamWriter writer, float3 value)
        {
            writer.WriteFloat(value.x);
            writer.WriteFloat(value.y);
            writer.WriteFloat(value.z);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WritePackedFloat3(ref this DataStreamWriter writer, float3 value, in StreamCompressionModel model)
        {
            writer.WritePackedFloat(value.x, model);
            writer.WritePackedFloat(value.y, model);
            writer.WritePackedFloat(value.z, model);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WritePackedFloat3Delta(ref this DataStreamWriter writer, float3 value, float3 baseline, in StreamCompressionModel model)
        {
            writer.WritePackedFloatDelta(value.x, baseline.x, model);
            writer.WritePackedFloatDelta(value.y, baseline.y, model);
            writer.WritePackedFloatDelta(value.z, baseline.z, model);
            
            // writer.WriteFloatDeltaXOR(value.x, baseline.x);
            // writer.WriteFloatDeltaXOR(value.y, baseline.y);
            // writer.WriteFloatDeltaXOR(value.z, baseline.z);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ReadFloat3(ref this DataStreamReader reader)
        {
            float3 value;
            value.x = reader.ReadFloat();
            value.y = reader.ReadFloat();
            value.z = reader.ReadFloat();

            return value;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ReadPackedFloat3(ref this DataStreamReader reader, in StreamCompressionModel model)
        {
            float3 value;
            value.x = reader.ReadPackedFloat(in model);
            value.y = reader.ReadPackedFloat(in model);
            value.z = reader.ReadPackedFloat(in model);

            return value;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ReadPackedFloat3Delta(ref this DataStreamReader reader, float3 baseline, in StreamCompressionModel model)
        {
            float3 value;
            value.x = reader.ReadPackedFloatDelta(baseline.x, in model);
            value.y = reader.ReadPackedFloatDelta(baseline.y, in model);
            value.z = reader.ReadPackedFloatDelta(baseline.z, in model);
            
            // value.x = reader.ReadFloatDeltaXOR(baseline.x);
            // value.y = reader.ReadFloatDeltaXOR(baseline.y);
            // value.z = reader.ReadFloatDeltaXOR(baseline.z);

            return value;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteQuat(ref this DataStreamWriter writer, quaternion value)
        {
            writer.WriteFloat(value.value.x);
            writer.WriteFloat(value.value.y);
            writer.WriteFloat(value.value.z);
            writer.WriteFloat(value.value.w);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WritePackedQuat(ref this DataStreamWriter writer, quaternion value, in StreamCompressionModel model)
        {
            writer.WritePackedFloat(value.value.x, in model);
            writer.WritePackedFloat(value.value.y, in model);
            writer.WritePackedFloat(value.value.z, in model);
            writer.WritePackedFloat(value.value.w, in model);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WritePackedQuatDelta(ref this DataStreamWriter writer, quaternion value, quaternion baseline, in StreamCompressionModel model)
        {
            writer.WritePackedFloatDelta(value.value.x, baseline.value.x, in model);
            writer.WritePackedFloatDelta(value.value.y, baseline.value.y, in model);
            writer.WritePackedFloatDelta(value.value.z, baseline.value.z, in model);
            writer.WritePackedFloatDelta(value.value.w, baseline.value.w, in model);
            
            // writer.WriteFloatDeltaXOR(value.value.x, baseline.value.x);
            // writer.WriteFloatDeltaXOR(value.value.y, baseline.value.y);
            // writer.WriteFloatDeltaXOR(value.value.z, baseline.value.z);
            // writer.WriteFloatDeltaXOR(value.value.w, baseline.value.w);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion ReadQuat(ref this DataStreamReader reader)
        {
            quaternion value;
            value.value.x = reader.ReadFloat();
            value.value.y = reader.ReadFloat();
            value.value.z = reader.ReadFloat();
            value.value.w = reader.ReadFloat();

            return value;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion ReadPackedQuat(ref this DataStreamReader reader, in StreamCompressionModel model)
        {
            quaternion value;
            value.value.x = reader.ReadPackedFloat(in model);
            value.value.y = reader.ReadPackedFloat(in model);
            value.value.z = reader.ReadPackedFloat(in model);
            value.value.w = reader.ReadPackedFloat(in model);

            return value;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion ReadPackedQuatDelta(ref this DataStreamReader reader, quaternion baseline, in StreamCompressionModel model)
        {
            quaternion value;
            value.value.x = reader.ReadPackedFloatDelta(baseline.value.x, in model);
            value.value.y = reader.ReadPackedFloatDelta(baseline.value.y, in model);
            value.value.z = reader.ReadPackedFloatDelta(baseline.value.z, in model);
            value.value.w = reader.ReadPackedFloatDelta(baseline.value.w, in model);
            
            // value.value.x = reader.ReadFloatDeltaXOR(baseline.value.x);
            // value.value.y = reader.ReadFloatDeltaXOR(baseline.value.y);
            // value.value.z = reader.ReadFloatDeltaXOR(baseline.value.z);
            // value.value.w = reader.ReadFloatDeltaXOR(baseline.value.w);

            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WritePackedBlobArrayDelta<T>(
            ref this DataStreamWriter writer, 
            ref JPH_BlobArray<T> array, 
            ref JPH_BlobArray<T> baseline,
            in StreamCompressionModel model)
            where T : unmanaged, IPackedDeltaSerializable<T>, IPackedSerializable
        {
            writer.WritePackedIntDelta(array.Length, baseline.Length, in model);
            
            var minLen = math.min(array.Length, baseline.Length);
            var i = 0;
            for (; i < minLen; i++)
            {
                array[i].Serialize(ref baseline[i], ref writer, in model);
            }

            for (; i < array.Length; i++)
            {
                array[i].Serialize(ref writer, in model);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadPackedBlobArrayDelta<T>(
            ref this DataStreamReader reader,
            NativeBlobBuilder builder,
            ref JPH_BlobArray<T> array,
            ref JPH_BlobArray<T> baseline,
            in StreamCompressionModel model)
            where T : unmanaged, IPackedDeltaSerializable<T>, IPackedSerializable
        {
            var len = reader.ReadPackedIntDelta(baseline.Length, in model);

            var blobArraybuilder = builder.Allocate(ref array, len);
            
            var minLen = math.min(len, baseline.Length);
            var i = 0;
            for (; i < minLen; i++)
            {
                blobArraybuilder[i].Deserialize(ref baseline[i], builder, ref reader, in model);
            }

            for (; i < len; i++)
            {
                blobArraybuilder[i].Deserialize(builder, ref reader, in model);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFloatDeltaXOR(ref this DataStreamWriter writer, float value, float baseline)
        {
            var p = new UIntFloat { floatValue = baseline }.intValue;
            var c = new UIntFloat { floatValue = value }.intValue;

            if (p == c)
            {
                writer.WriteRawBits(0b00, 2);
                return;
            }

            var expPrev = (p >> 23) & 0xFF;
            var expNext = (c >> 23) & 0xFF;

            if (expPrev == expNext)
            {
                var prev = p & 0x7FFFFF;
                var cur = c & 0x7FFFFF;
                var delta = (int)cur - (int)prev;
                var zz = (uint)((delta << 1) ^ (delta >> 31));
                var bits = zz == 0 ? 0 : 32 - math.lzcnt(zz);
                writer.WriteRawBits(0b01, 2);
                writer.WriteRawBits((uint)bits, 5);
                writer.WriteRawBits(zz, bits);
            }
            else
            {
                writer.WriteRawBits(0b10, 2);
                writer.WriteFloat(value);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ReadFloatDeltaXOR(ref this DataStreamReader reader, float baseline)
        {
            var flag = reader.ReadRawBits(2);
            if (flag == 0x00) return baseline;

            var p = new UIntFloat { floatValue = baseline }.intValue;
            if (flag == 0x01)
            {
                var bits = reader.ReadRawBits(5);
                var zz = reader.ReadRawBits((int)bits);
                var delta = (int)((zz >> 1) ^ -(zz & 1));
                var exp = (int)((p >> 23) & 0xFF);
                var mant = (int)(p & 0x7FFFFF);
                mant += delta;

                var cur = (p & 0xFF800000) | ((uint)mant & 0x7FFFFF);
                
                return new UIntFloat { intValue = cur }.floatValue;
            }

            return reader.ReadFloat();
        }
    }
    
    public partial struct JPH_PhysicsSystemState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_PhysicsSystemState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteByte((byte)flags);
            global.Serialize(ref writer);
            writer.WriteInt(bodies.Length);
            for (var i = 0; i < bodies.Length; i++)
            {
                bodies[i].Serialize(ref writer);
            }
            contacts.Serialize(ref writer);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            flags = (JPH_StateRecorderState)reader.ReadByte();
            global.Deserialize(builder, ref reader);
            
            var length = reader.ReadInt();
            var bodyStateArrayBuilder = builder.Allocate(ref bodies, length);
            for (var i = 0; i < length; i++)
            {
                ref JPH_BodyState bodyState = ref bodyStateArrayBuilder[i];
                bodyState.Deserialize(builder, ref reader);
            }
            contacts.Deserialize(builder, ref reader);
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WriteByte((byte)flags);
            global.Serialize(ref writer, in model);
            writer.WritePackedInt(bodies.Length, in model);
            for (var i = 0; i < bodies.Length; i++)
            {
                bodies[i].Serialize(ref writer, in model);
            }
            contacts.Serialize(ref writer, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            flags = (JPH_StateRecorderState)reader.ReadByte();
            global.Deserialize(builder, ref reader, in model);
            
            var length = reader.ReadPackedInt(in model);
            var bodyStateArrayBuilder = builder.Allocate(ref bodies, length);
            for (var i = 0; i < length; i++)
            {
                ref JPH_BodyState bodyState = ref bodyStateArrayBuilder[i];
                bodyState.Deserialize(builder, ref reader, in model);
            }
            contacts.Deserialize(builder, ref reader, in model);
        }

        public void Serialize(ref JPH_PhysicsSystemState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WriteByte((byte)flags);
            global.Serialize(ref baseline.global, ref writer, in model);

            writer.WritePackedBlobArrayDelta(ref bodies, ref baseline.bodies, in model);
            
            contacts.Serialize(ref baseline.contacts, ref writer, in model);
        }

        public void Deserialize(ref JPH_PhysicsSystemState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            flags = (JPH_StateRecorderState)reader.ReadByte();
            global.Deserialize(ref baseline.global, builder, ref reader, in model);

            reader.ReadPackedBlobArrayDelta(builder, ref bodies, ref baseline.bodies, in model);
            
            contacts.Deserialize(ref baseline.contacts, builder, ref reader, in model);
        }
    }

    public partial struct JPH_GlobalState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_GlobalState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat(previousStepDeltaTime);
            writer.WriteFloat3(gravity);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            previousStepDeltaTime = reader.ReadFloat();
            gravity = reader.ReadFloat3();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat(previousStepDeltaTime, in model);
            writer.WritePackedFloat3(gravity, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            previousStepDeltaTime = reader.ReadPackedFloat(in model);
            gravity = reader.ReadPackedFloat3(in model);
        }

        public void Serialize(ref JPH_GlobalState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloatDelta(previousStepDeltaTime, baseline.previousStepDeltaTime, in model);
            writer.WritePackedFloat3Delta(gravity, baseline.gravity, in model);
        }

        public void Deserialize(ref JPH_GlobalState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            previousStepDeltaTime = reader.ReadPackedFloatDelta(baseline.previousStepDeltaTime, in model);
            gravity = reader.ReadPackedFloat3Delta(baseline.gravity, in model);
        }
    }
    
    public partial struct JPH_BodyState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_BodyState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt(id);
            var bits = 0u;
            if (isActive > 0) bits |= 1 << 0;
            writer.WriteRawBits(bits, 1);
            
            writer.WriteFloat3(position);
            
            writer.WriteQuat(rotation);
            
            motionProperties.Serialize(ref writer);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            id = reader.ReadUInt();
            var bits = reader.ReadRawBits(1);
            isActive = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            
            position = reader.ReadFloat3();
            rotation = reader.ReadQuat();
            
            motionProperties.Deserialize(builder, ref reader);
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUInt(id, in model);
            var bits = 0u;
            if (isActive > 0) bits |= 1 << 0;
            writer.WriteRawBits(bits, 1);
            
            writer.WritePackedFloat3(position, in model);
            
            writer.WritePackedQuat(rotation, in model);
            
            motionProperties.Serialize(ref writer, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            id = reader.ReadPackedUInt(in model);
            var bits = reader.ReadRawBits(1);
            isActive = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            
            position = reader.ReadPackedFloat3(in model);
            rotation = reader.ReadPackedQuat(in model);
            
            motionProperties.Deserialize(builder, ref reader, in model);
        }

        public void Serialize(ref JPH_BodyState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUIntDelta(id, baseline.id, in model);
            writer.WriteRawBits(isActive, 1);
            writer.WritePackedFloat3Delta(position, baseline.position, in model);
            writer.WritePackedQuatDelta(rotation, baseline.rotation, in model);
            motionProperties.Serialize(ref baseline.motionProperties, ref writer, in model);
        }

        public void Deserialize(ref JPH_BodyState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            id = reader.ReadPackedUIntDelta(baseline.id, in model);
            isActive = (byte)reader.ReadRawBits(1);
            position = reader.ReadPackedFloat3Delta(baseline.position, in model);
            rotation = reader.ReadPackedQuatDelta(baseline.rotation, in model);
            motionProperties.Deserialize(ref baseline.motionProperties, builder, ref reader, in model);
        }
    }

    public partial struct JPH_MotionPropertiesState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_MotionPropertiesState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat3(linearVelocity);
            writer.WriteFloat3(angularVelocity);
            writer.WriteFloat3(force);
            writer.WriteFloat3(torque);
            sleepTestSpheres.e0.Serialize(ref writer);
            sleepTestSpheres.e1.Serialize(ref writer);
            sleepTestSpheres.e2.Serialize(ref writer);
            writer.WriteFloat(sleepTestTimer);
            writer.WriteRawBits(allowSleeping, 1);
        }
        
        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            linearVelocity = reader.ReadFloat3();
            angularVelocity = reader.ReadFloat3();
            force = reader.ReadFloat3();
            torque = reader.ReadFloat3();
            sleepTestSpheres.e0.Deserialize(builder, ref reader);
            sleepTestSpheres.e1.Deserialize(builder, ref reader);
            sleepTestSpheres.e2.Deserialize(builder, ref reader);
            sleepTestTimer = reader.ReadFloat();
            allowSleeping = (byte)reader.ReadRawBits(1);
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3(linearVelocity, in model);
            writer.WritePackedFloat3(angularVelocity, in model);
            writer.WritePackedFloat3(force, in model);
            writer.WritePackedFloat3(torque, in model);
            sleepTestSpheres.e0.Serialize(ref writer, in model);
            sleepTestSpheres.e1.Serialize(ref writer, in model);
            sleepTestSpheres.e2.Serialize(ref writer, in model);
            writer.WritePackedFloat(sleepTestTimer, in model);
            writer.WriteRawBits(allowSleeping, 1);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            linearVelocity = reader.ReadPackedFloat3(in model);
            angularVelocity = reader.ReadPackedFloat3(in model);
            force = reader.ReadPackedFloat3(in model);
            torque = reader.ReadPackedFloat3(in model);
            sleepTestSpheres.e0.Deserialize(builder, ref reader, in model);
            sleepTestSpheres.e1.Deserialize(builder, ref reader, in model);
            sleepTestSpheres.e2.Deserialize(builder, ref reader, in model);
            sleepTestTimer = reader.ReadPackedFloat(in model);
            allowSleeping = (byte)reader.ReadRawBits(1);
        }

        public void Serialize(ref JPH_MotionPropertiesState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3Delta(linearVelocity, baseline.linearVelocity, in model);
            writer.WritePackedFloat3Delta(angularVelocity, baseline.angularVelocity, in model);
            writer.WritePackedFloat3Delta(force, baseline.force, in model);
            writer.WritePackedFloat3Delta(torque, baseline.torque, in model);
            sleepTestSpheres.e0.Serialize(ref baseline.sleepTestSpheres.e0, ref writer, in model);
            sleepTestSpheres.e1.Serialize(ref baseline.sleepTestSpheres.e1, ref writer, in model);
            sleepTestSpheres.e2.Serialize(ref baseline.sleepTestSpheres.e2, ref writer, in model);
            writer.WritePackedFloatDelta(sleepTestTimer, baseline.sleepTestTimer, in model);
            writer.WriteRawBits(allowSleeping, 1);
        }

        public void Deserialize(ref JPH_MotionPropertiesState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            linearVelocity = reader.ReadPackedFloat3Delta(baseline.linearVelocity, in model);
            angularVelocity = reader.ReadPackedFloat3Delta(baseline.angularVelocity, in model);
            force = reader.ReadPackedFloat3Delta(baseline.force, in model);
            torque = reader.ReadPackedFloat3Delta(baseline.torque, in model);
            sleepTestSpheres.e0.Deserialize(ref baseline.sleepTestSpheres.e0, builder, ref reader, in model);
            sleepTestSpheres.e1.Deserialize(ref baseline.sleepTestSpheres.e1, builder, ref reader, in model);
            sleepTestSpheres.e2.Deserialize(ref baseline.sleepTestSpheres.e2, builder, ref reader, in model);
            sleepTestTimer = reader.ReadPackedFloatDelta(baseline.sleepTestTimer, in model);
            allowSleeping = (byte)reader.ReadRawBits(1);
        }
    }

    public partial struct JPH_Sphere : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_Sphere>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat3(center);
            writer.WriteFloat(radius);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            center = reader.ReadFloat3();
            radius = reader.ReadFloat();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3(center, in model);
            writer.WritePackedFloat(radius, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            center = reader.ReadPackedFloat3(in model);
            radius = reader.ReadPackedFloat(in model);
        }

        public void Serialize(ref JPH_Sphere baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3Delta(center, baseline.center, in model);
            writer.WritePackedFloatDelta(radius, baseline.radius, in model);
        }

        public void Deserialize(ref JPH_Sphere baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            center = reader.ReadPackedFloat3Delta(baseline.center, in model);
            radius = reader.ReadPackedFloatDelta(baseline.radius, in model);
        }
    }

    public partial struct JPH_ContactConstraintState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_ContactConstraintState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            manifold.Serialize(ref writer);
        }
        
        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            manifold.Deserialize(builder, ref reader);
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            manifold.Serialize(ref writer, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            manifold.Deserialize(builder, ref reader, in model);
        }

        public void Serialize(ref JPH_ContactConstraintState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            manifold.Serialize(ref baseline.manifold, ref writer, in model);
        }

        public void Deserialize(ref JPH_ContactConstraintState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            manifold.Deserialize(ref baseline.manifold, builder, ref reader, in model);
        }
    }

    public partial struct JPH_ManifoldCacheState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_ManifoldCacheState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            var bodyLength = bodyPairs.Length;
            writer.WriteInt(bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairs[i].Serialize(ref writer);
            }

            var ccdLength = ccdManifolds.Length;
            writer.WriteInt(ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdManifolds[i].Serialize(ref writer);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            var bodyLength = reader.ReadInt();
            var bodyPairArrayBuilder = builder.Allocate(ref bodyPairs, bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairArrayBuilder[i].Deserialize(builder, ref reader);
            }

            var ccdLength = reader.ReadInt();
            var ccdArrayBuilder = builder.Allocate(ref ccdManifolds, ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdArrayBuilder[i].Deserialize(builder, ref reader);
            }
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            var bodyLength = bodyPairs.Length;
            writer.WriteInt(bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairs[i].Serialize(ref writer, in model);
            }

            var ccdLength = ccdManifolds.Length;
            writer.WriteInt(ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdManifolds[i].Serialize(ref writer, in model);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            var bodyLength = reader.ReadInt();
            var bodyPairArrayBuilder = builder.Allocate(ref bodyPairs, bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }

            var ccdLength = reader.ReadInt();
            var ccdArrayBuilder = builder.Allocate(ref ccdManifolds, ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }
        }

        public void Serialize(ref JPH_ManifoldCacheState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedBlobArrayDelta(ref bodyPairs, ref baseline.bodyPairs, in model);
            writer.WritePackedBlobArrayDelta(ref ccdManifolds, ref baseline.ccdManifolds, in model);
        }

        public void Deserialize(ref JPH_ManifoldCacheState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            reader.ReadPackedBlobArrayDelta(builder, ref bodyPairs, ref baseline.bodyPairs, in model);
            reader.ReadPackedBlobArrayDelta(builder, ref ccdManifolds, ref baseline.ccdManifolds, in model);
        }
    }
    
    public partial struct JPH_BodyPairKeyValueState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_BodyPairKeyValueState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt(key.BodyA);
            writer.WriteUInt(key.BodyB);
            
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            key.BodyA = reader.ReadUInt();
            key.BodyB = reader.ReadUInt();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUInt(key.BodyA, in model);
            writer.WritePackedUInt(key.BodyB, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            key.BodyA = reader.ReadPackedUInt(in model);
            key.BodyB = reader.ReadPackedUInt(in model);
        }

        public void Serialize(ref JPH_BodyPairKeyValueState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUIntDelta(key.BodyA, baseline.key.BodyA, in model);
            writer.WritePackedUIntDelta(key.BodyB, baseline.key.BodyB, in model);
        }

        public void Deserialize(ref JPH_BodyPairKeyValueState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            key.BodyA = reader.ReadPackedUIntDelta(baseline.key.BodyA, in model);
            key.BodyB = reader.ReadPackedUIntDelta(baseline.key.BodyB, in model);
        }
    }

    public partial struct JPH_CachedBodyPairState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_CachedBodyPairState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat3(deltaPosition);
            writer.WriteFloat3(deltaRotation);

            var manifoldLength = (byte)manifolds.Length;
            writer.WriteByte(manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifolds[i].Serialize(ref writer);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            deltaPosition = reader.ReadFloat3();
            deltaRotation = reader.ReadFloat3();

            var manifoldLength = reader.ReadByte();
            var manifoldArrayBuilder = builder.Allocate(ref manifolds, manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifoldArrayBuilder[i].Deserialize(builder, ref reader);
            }
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3(deltaPosition, in model);
            writer.WritePackedFloat3(deltaRotation, in model);

            var manifoldLength = (byte)manifolds.Length;
            writer.WriteByte(manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifolds[i].Serialize(ref writer, in model);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            deltaPosition = reader.ReadPackedFloat3(in model);
            deltaRotation = reader.ReadPackedFloat3(in model);

            var manifoldLength = reader.ReadByte();
            var manifoldArrayBuilder = builder.Allocate(ref manifolds, manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifoldArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }
        }

        public void Serialize(ref JPH_CachedBodyPairState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3Delta(deltaPosition, baseline.deltaPosition, in model);
            writer.WritePackedFloat3Delta(deltaRotation, baseline.deltaRotation, in model);

            var manifoldLength = (byte)manifolds.Length;
            writer.WriteByte(manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifolds[i].Serialize(ref writer, in model);
            }
        }

        public void Deserialize(ref JPH_CachedBodyPairState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            deltaPosition = reader.ReadPackedFloat3(in model);
            deltaRotation = reader.ReadPackedFloat3(in model);

            var manifoldLength = reader.ReadByte();
            var manifoldArrayBuilder = builder.Allocate(ref manifolds, manifoldLength);
            for (var i = 0; i < manifoldLength; i++)
            {
                manifoldArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }
        }
    }

    public partial struct JPH_ManifoldKeyValueState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_ManifoldKeyValueState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            key.Serialize(ref writer);
            value.Serialize(ref writer);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            key.Deserialize(builder, ref reader);
            value.Deserialize(builder, ref reader);
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            key.Serialize(ref writer, in model);
            value.Serialize(ref writer, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            key.Deserialize(builder, ref reader, in model);
            value.Deserialize(builder, ref reader, in model);
        }

        public void Serialize(ref JPH_ManifoldKeyValueState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            key.Serialize(ref baseline.key, ref writer, in model);
            value.Serialize(ref baseline.value, ref writer, in model);
        }

        public void Deserialize(ref JPH_ManifoldKeyValueState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            key.Deserialize(ref baseline.key, builder, ref reader, in model);
            value.Deserialize(ref baseline.value, builder, ref reader, in model);
        }
    }

    public partial struct JPH_SubShapeIDPair : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_SubShapeIDPair>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt(Body1ID);
            writer.WriteUInt(subShapeID1);
            writer.WriteUInt(Body2ID);
            writer.WriteUInt(subShapeID2);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            Body1ID = reader.ReadUInt();
            subShapeID1 = reader.ReadUInt();
            Body2ID = reader.ReadUInt();
            subShapeID2 = reader.ReadUInt();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUInt(Body1ID, in model);
            writer.WritePackedUInt(subShapeID1, in model);
            writer.WritePackedUInt(Body2ID, in model);
            writer.WritePackedUInt(subShapeID2, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            Body1ID = reader.ReadPackedUInt(in model);
            subShapeID1 = reader.ReadPackedUInt(in model);
            Body2ID = reader.ReadPackedUInt(in model);
            subShapeID2 = reader.ReadPackedUInt(in model);
        }

        public void Serialize(ref JPH_SubShapeIDPair baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUIntDelta(Body1ID, baseline.Body1ID, in model);
            writer.WritePackedUIntDelta(subShapeID1, baseline.subShapeID1, in model);
            writer.WritePackedUIntDelta(Body2ID, baseline.Body2ID, in model);
            writer.WritePackedUIntDelta(subShapeID2, baseline.subShapeID2, in model);
        }

        public void Deserialize(ref JPH_SubShapeIDPair baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            Body1ID = reader.ReadPackedUIntDelta(baseline.Body1ID, in model);
            subShapeID1 = reader.ReadPackedUIntDelta(baseline.subShapeID1, in model);
            Body2ID = reader.ReadPackedUIntDelta(baseline.Body2ID, in model);
            subShapeID2 = reader.ReadPackedUIntDelta(baseline.subShapeID2, in model);
        }
    }

    public partial struct JPH_CachedManifoldState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_CachedManifoldState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat3(contactNormal);

            var contactPointLength = (byte)contactPoints.Length;
            writer.WriteByte(contactPointLength);
            for (var i = 0; i < contactPointLength; i++)
            {
                contactPoints[i].Serialize(ref writer);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            contactNormal = reader.ReadFloat3();

            var contactPointLength = reader.ReadByte();
            var contactPointArrayBuilder = builder.Allocate(ref contactPoints, contactPointLength);
            for (var i = 0; i < contactPointLength; i++)
            {
                contactPointArrayBuilder[i].Deserialize(builder, ref reader);
            }
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3(contactNormal, in model);

            var contactPointLength = (byte)contactPoints.Length;
            writer.WriteByte(contactPointLength);
            for (var i = 0; i < contactPointLength; i++)
            {
                contactPoints[i].Serialize(ref writer, in model);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            contactNormal = reader.ReadPackedFloat3(in model);

            var contactPointLength = reader.ReadByte();
            var contactPointArrayBuilder = builder.Allocate(ref contactPoints, contactPointLength);
            for (var i = 0; i < contactPointLength; i++)
            {
                contactPointArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }
        }

        public void Serialize(ref JPH_CachedManifoldState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3Delta(contactNormal, baseline.contactNormal, in model);
            writer.WritePackedBlobArrayDelta(ref contactPoints, ref baseline.contactPoints, in model);
        }

        public void Deserialize(ref JPH_CachedManifoldState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            contactNormal = reader.ReadPackedFloat3Delta(baseline.contactNormal, in model);
            reader.ReadPackedBlobArrayDelta(builder, ref contactPoints, ref baseline.contactPoints, in model);
        }
    }

    public partial struct JPH_CachedContactPointState : ISerializable, IPackedSerializable, IPackedDeltaSerializable<JPH_CachedContactPointState>
    {
        public unsafe void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat3(position1);
            writer.WriteFloat3(position2);
            writer.WriteFloat(nonPenetrationLambda);
            writer.WriteFloat(frictionLambda[0]);
            writer.WriteFloat(frictionLambda[1]);
        }

        public unsafe void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            position1 = reader.ReadFloat3();
            position2 = reader.ReadFloat3();
            nonPenetrationLambda = reader.ReadFloat();
            frictionLambda[0] =  reader.ReadFloat();
            frictionLambda[1] =  reader.ReadFloat();
        }

        public unsafe void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3(position1, in model);
            writer.WritePackedFloat3(position2, in model);
            writer.WritePackedFloat(nonPenetrationLambda, in model);
            writer.WritePackedFloat(frictionLambda[0], in model);
            writer.WritePackedFloat(frictionLambda[1], in model);
        }

        public unsafe void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            position1 = reader.ReadPackedFloat3(in model);
            position2 = reader.ReadPackedFloat3(in model);
            nonPenetrationLambda = reader.ReadPackedFloat(in model);
            frictionLambda[0] =  reader.ReadPackedFloat(in model);
            frictionLambda[1] =  reader.ReadPackedFloat(in model);
        }

        public unsafe void Serialize(ref JPH_CachedContactPointState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedFloat3Delta(position1, baseline.position1, in model);
            writer.WritePackedFloat3Delta(position2, baseline.position2, in model);
            writer.WritePackedFloatDelta(nonPenetrationLambda, baseline.nonPenetrationLambda, in model);
            writer.WritePackedFloatDelta(frictionLambda[0], baseline.frictionLambda[0], in model);
            writer.WritePackedFloatDelta(frictionLambda[1], baseline.frictionLambda[1], in model);
        }

        public unsafe void Deserialize(ref JPH_CachedContactPointState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            position1 = reader.ReadPackedFloat3Delta(baseline.position1, in model);
            position2 = reader.ReadPackedFloat3Delta(baseline.position2, in model);
            nonPenetrationLambda = reader.ReadPackedFloatDelta(baseline.nonPenetrationLambda, in model);
            frictionLambda[0] =  reader.ReadPackedFloatDelta(baseline.frictionLambda[0], in model);
            frictionLambda[1] =  reader.ReadPackedFloatDelta(baseline.frictionLambda[1], in model);
        }
    }

    public partial struct JPH_CharacterBaseState : ISerializable, IPackedSerializable,
        IPackedDeltaSerializable<JPH_CharacterBaseState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt((uint)groundState);
            writer.WriteUInt(groundBodyID);
            writer.WriteUInt(groundBodySubShapeID);
            writer.WriteFloat3(groundPosition);
            writer.WriteFloat3(groundNormal);
            writer.WriteFloat3(groundVelocity);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            groundState = (JPH_GroundState)reader.ReadUInt();
            groundBodyID = reader.ReadUInt();
            groundBodySubShapeID = reader.ReadUInt();
            groundPosition = reader.ReadFloat3();
            groundNormal = reader.ReadFloat3();
            groundVelocity = reader.ReadFloat3();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUInt((uint)groundState, in model);
            writer.WritePackedUInt(groundBodyID, in model);
            writer.WritePackedUInt(groundBodySubShapeID, in model);
            writer.WritePackedFloat3(groundPosition, in model);
            writer.WritePackedFloat3(groundNormal, in model);
            writer.WritePackedFloat3(groundVelocity, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            groundState = (JPH_GroundState)reader.ReadPackedUInt(in model);
            groundBodyID = reader.ReadPackedUInt(in model);
            groundBodySubShapeID = reader.ReadPackedUInt(in model);
            groundPosition = reader.ReadPackedFloat3(in model);
            groundNormal = reader.ReadPackedFloat3(in model);
            groundVelocity = reader.ReadPackedFloat3(in model);
        }

        public void Serialize(ref JPH_CharacterBaseState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUIntDelta((uint)groundState, (uint)baseline.groundState, in model);
            writer.WritePackedUIntDelta(groundBodyID, baseline.groundBodyID, in model);
            writer.WritePackedUIntDelta(groundBodySubShapeID, baseline.groundBodySubShapeID, in model);
            writer.WritePackedFloat3Delta(groundPosition, baseline.groundPosition, in model);
            writer.WritePackedFloat3Delta(groundNormal, baseline.groundNormal, in model);
            writer.WritePackedFloat3Delta(groundVelocity, baseline.groundVelocity, in model);
        }

        public void Deserialize(ref JPH_CharacterBaseState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            groundState = (JPH_GroundState)reader.ReadPackedUIntDelta((uint)baseline.groundState, in model);
            groundBodyID = reader.ReadPackedUIntDelta(baseline.groundBodyID, in model);
            groundBodySubShapeID = reader.ReadPackedUIntDelta(baseline.groundBodySubShapeID, in model);
            groundPosition = reader.ReadPackedFloat3Delta(baseline.groundPosition, in model);
            groundNormal = reader.ReadPackedFloat3Delta(baseline.groundNormal, in model);
            groundVelocity = reader.ReadPackedFloat3Delta(baseline.groundVelocity, in model);
        }
    }

    public partial struct JPH_CharacterVirtualContactKeyState : ISerializable, IPackedSerializable,
        IPackedDeltaSerializable<JPH_CharacterVirtualContactKeyState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt(bodyB);
            writer.WriteUInt(characterIDB);
            writer.WriteUInt(subShapeIDB);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            bodyB = reader.ReadUInt();
            characterIDB = reader.ReadUInt();
            subShapeIDB = reader.ReadUInt();
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUInt(bodyB, in model);
            writer.WritePackedUInt(characterIDB, in model);
            writer.WritePackedUInt(subShapeIDB, in model);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            bodyB = reader.ReadPackedUInt(in model);
            characterIDB = reader.ReadPackedUInt(in model);
            subShapeIDB = reader.ReadPackedUInt(in model);
        }

        public void Serialize(ref JPH_CharacterVirtualContactKeyState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            writer.WritePackedUIntDelta(bodyB, baseline.bodyB, in model);
            writer.WritePackedUIntDelta(characterIDB, baseline.characterIDB, in model);
            writer.WritePackedUIntDelta(subShapeIDB, baseline.subShapeIDB, in model);
        }

        public void Deserialize(ref JPH_CharacterVirtualContactKeyState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            bodyB = reader.ReadPackedUIntDelta(baseline.bodyB, in model);
            characterIDB = reader.ReadPackedUIntDelta(baseline.characterIDB, in model);
            subShapeIDB = reader.ReadPackedUIntDelta(baseline.subShapeIDB, in model);
        }
    }

    public partial struct JPH_CharacterVirtualContactState : ISerializable, IPackedSerializable,
        IPackedDeltaSerializable<JPH_CharacterVirtualContactState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            key.Serialize(ref writer);
            writer.WriteFloat3(position);
            writer.WriteFloat3(linearVelocity);
            writer.WriteFloat3(contactNormal);
            writer.WriteFloat3(surfaceNormal);
            writer.WriteFloat(distance);
            writer.WriteFloat(fraction);
            writer.WriteByte((byte)motionTypeB);
            var bits = 0u;
            if (isSensorB > 0) bits |= 1 << 0;
            if (hadCollision > 0) bits |= 1 << 1;
            if (wasDiscarded > 0) bits |= 1 << 2;
            if (canPushCharacter > 0) bits |= 1 << 3;
            writer.WriteRawBits(bits, 4);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            key.Deserialize(builder, ref reader);
            position = reader.ReadFloat3();
            linearVelocity = reader.ReadFloat3();
            contactNormal = reader.ReadFloat3();
            surfaceNormal = reader.ReadFloat3();
            distance = reader.ReadFloat();
            fraction = reader.ReadFloat();
            motionTypeB = (JPH_MotionType)reader.ReadByte();
            var bits = reader.ReadRawBits(4);
            isSensorB = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            hadCollision = (bits & (1 << 1)) != 0 ? (byte)1 : (byte)0;
            wasDiscarded = (bits & (1 << 2)) != 0 ? (byte)1 : (byte)0;
            canPushCharacter = (bits & (1 << 3)) != 0 ? (byte)1 : (byte)0;
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            key.Serialize(ref writer, in model);
            writer.WritePackedFloat3(position, in model);
            writer.WritePackedFloat3(linearVelocity, in model);
            writer.WritePackedFloat3(contactNormal, in model);
            writer.WritePackedFloat3(surfaceNormal, in model);
            writer.WritePackedFloat(distance, in model);
            writer.WritePackedFloat(fraction, in model);
            writer.WriteByte((byte)motionTypeB);
            var bits = 0u;
            if (isSensorB > 0) bits |= 1 << 0;
            if (hadCollision > 0) bits |= 1 << 1;
            if (wasDiscarded > 0) bits |= 1 << 2;
            if (canPushCharacter > 0) bits |= 1 << 3;
            writer.WriteRawBits(bits, 4);
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            key.Deserialize(builder, ref reader, in model);
            position = reader.ReadPackedFloat3(in model);
            linearVelocity = reader.ReadPackedFloat3(in model);
            contactNormal = reader.ReadPackedFloat3(in model);
            surfaceNormal = reader.ReadPackedFloat3(in model);
            distance = reader.ReadPackedFloat(in model);
            fraction = reader.ReadPackedFloat(in model);
            motionTypeB = (JPH_MotionType)reader.ReadByte();
            var bits = reader.ReadRawBits(4);
            isSensorB = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            hadCollision = (bits & (1 << 1)) != 0 ? (byte)1 : (byte)0;
            wasDiscarded = (bits & (1 << 2)) != 0 ? (byte)1 : (byte)0;
            canPushCharacter = (bits & (1 << 3)) != 0 ? (byte)1 : (byte)0;
        }

        public void Serialize(ref JPH_CharacterVirtualContactState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            key.Serialize(ref baseline.key, ref writer, in model);
            writer.WritePackedFloat3Delta(position, baseline.position, in model);
            writer.WritePackedFloat3Delta(linearVelocity, baseline.linearVelocity, in model);
            writer.WritePackedFloat3Delta(contactNormal, baseline.contactNormal, in model);
            writer.WritePackedFloat3Delta(surfaceNormal, baseline.surfaceNormal, in model);
            writer.WritePackedFloatDelta(distance, baseline.distance, in model);
            writer.WritePackedFloatDelta(fraction, baseline.fraction, in model);
            writer.WriteByte((byte)motionTypeB);
            var bits = 0u;
            if (isSensorB > 0) bits |= 1 << 0;
            if (hadCollision > 0) bits |= 1 << 1;
            if (wasDiscarded > 0) bits |= 1 << 2;
            if (canPushCharacter > 0) bits |= 1 << 3;
            writer.WriteRawBits(bits, 4);
        }

        public void Deserialize(ref JPH_CharacterVirtualContactState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            key.Deserialize(ref baseline.key, builder, ref reader, in model);
            position = reader.ReadPackedFloat3Delta(baseline.position, in model);
            linearVelocity = reader.ReadPackedFloat3Delta(baseline.linearVelocity, in model);
            contactNormal = reader.ReadPackedFloat3Delta(baseline.contactNormal, in model);
            surfaceNormal = reader.ReadPackedFloat3Delta(baseline.surfaceNormal, in model);
            distance = reader.ReadPackedFloatDelta(baseline.distance, in model);
            fraction = reader.ReadPackedFloatDelta(baseline.fraction, in model);
            motionTypeB = (JPH_MotionType)reader.ReadByte();
            var bits = reader.ReadRawBits(4);
            isSensorB = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            hadCollision = (bits & (1 << 1)) != 0 ? (byte)1 : (byte)0;
            wasDiscarded = (bits & (1 << 2)) != 0 ? (byte)1 : (byte)0;
            canPushCharacter = (bits & (1 << 3)) != 0 ? (byte)1 : (byte)0;
        }
    }

    public partial struct JPH_CharacterVirtualState : ISerializable, IPackedSerializable,
        IPackedDeltaSerializable<JPH_CharacterVirtualState>
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            @base.Serialize(ref writer);
            writer.WriteFloat3(position);
            writer.WriteQuat(rotation);
            writer.WriteFloat3(linearVelocity);
            writer.WriteFloat(lastDeltaTime);
            writer.WriteRawBits(maxHitsExceeded, 1);

            var contactLength = (byte)contacts.Length;
            writer.WriteByte(contactLength);
            for (var i = 0; i < contactLength; i++)
            {
                contacts[i].Serialize(ref writer);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            @base.Deserialize(builder, ref reader);
            position = reader.ReadFloat3();
            rotation = reader.ReadQuat();
            linearVelocity = reader.ReadFloat3();
            lastDeltaTime = reader.ReadFloat();
            maxHitsExceeded = (byte)reader.ReadRawBits(1);

            var contactLength = reader.ReadByte();
            var contactArrayBuilder = builder.Allocate(ref contacts, contactLength);
            for (var i = 0; i < contactLength; i++)
            {
                contactArrayBuilder[i].Deserialize(builder, ref reader);
            }
        }

        public void Serialize(ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            @base.Serialize(ref writer, in model);
            writer.WritePackedFloat3(position, in model);
            writer.WritePackedQuat(rotation, in model);
            writer.WritePackedFloat3(linearVelocity, in model);
            writer.WritePackedFloat(lastDeltaTime, in model);
            writer.WriteRawBits(maxHitsExceeded, 1);

            var contactLength = (byte)contacts.Length;
            writer.WriteByte(contactLength);
            for (var i = 0; i < contactLength; i++)
            {
                contacts[i].Serialize(ref writer, in model);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader, in StreamCompressionModel model)
        {
            @base.Deserialize(builder, ref reader, in model);
            position = reader.ReadPackedFloat3(in model);
            rotation = reader.ReadPackedQuat(in model);
            linearVelocity = reader.ReadPackedFloat3(in model);
            lastDeltaTime = reader.ReadPackedFloat(in model);
            maxHitsExceeded = (byte)reader.ReadRawBits(1);

            var contactLength = reader.ReadByte();
            var contactArrayBuilder = builder.Allocate(ref contacts, contactLength);
            for (var i = 0; i < contactLength; i++)
            {
                contactArrayBuilder[i].Deserialize(builder, ref reader, in model);
            }
        }

        public void Serialize(ref JPH_CharacterVirtualState baseline, ref DataStreamWriter writer, in StreamCompressionModel model)
        {
            @base.Serialize(ref baseline.@base, ref writer, in model);
            writer.WritePackedFloat3Delta(position, baseline.position, in model);
            writer.WritePackedQuatDelta(rotation, baseline.rotation, in model);
            writer.WritePackedFloat3Delta(linearVelocity, baseline.linearVelocity, in model);
            writer.WritePackedFloatDelta(lastDeltaTime, baseline.lastDeltaTime, in model);
            writer.WriteRawBits(maxHitsExceeded, 1);

            writer.WritePackedBlobArrayDelta(ref contacts, ref baseline.contacts, in model);
        }

        public void Deserialize(ref JPH_CharacterVirtualState baseline, NativeBlobBuilder builder, ref DataStreamReader reader,
            in StreamCompressionModel model)
        {
            @base.Deserialize(ref baseline.@base, builder, ref reader, in model);
            position = reader.ReadPackedFloat3Delta(baseline.position, in model);
            rotation = reader.ReadPackedQuatDelta(baseline.rotation, in model);
            linearVelocity = reader.ReadPackedFloat3Delta(baseline.linearVelocity, in model);
            lastDeltaTime = reader.ReadPackedFloatDelta(baseline.lastDeltaTime, in model);
            maxHitsExceeded = (byte)reader.ReadRawBits(1);

            reader.ReadPackedBlobArrayDelta(builder, ref contacts, ref baseline.contacts, in model);
        }
    }
}
