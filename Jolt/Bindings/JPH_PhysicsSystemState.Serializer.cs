using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Jolt
{
    public static class SerializerUtility
    {
        public static void WriteFloat3(ref this DataStreamWriter writer, float3 value)
        {
            writer.WriteFloat(value.x);
            writer.WriteFloat(value.y);
            writer.WriteFloat(value.z);
        }
        
        public static float3 ReadFloat3(ref this DataStreamReader reader)
        {
            float3 value;
            value.x = reader.ReadFloat();
            value.y = reader.ReadFloat();
            value.z = reader.ReadFloat();

            return value;
        }
        
        public static void WriteQuat(ref this DataStreamWriter writer, quaternion value)
        {
            writer.WriteFloat(value.value.x);
            writer.WriteFloat(value.value.y);
            writer.WriteFloat(value.value.z);
            writer.WriteFloat(value.value.w);
        }
        
        public static quaternion ReadQuat(ref this DataStreamReader reader)
        {
            quaternion value;
            value.value.x = reader.ReadFloat();
            value.value.y = reader.ReadFloat();
            value.value.z = reader.ReadFloat();
            value.value.w = reader.ReadFloat();

            return value;
        }
    }
    
    public partial struct JPH_PhysicsSystemState
    {
        public readonly void Serialize(ref DataStreamWriter writer)
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

        public void Deserialize(ref DataStreamReader reader)
        {
            flags = (JPH_StateRecorderState)reader.ReadByte();
            global.Deserialize(ref reader);
            var length = reader.ReadInt();
            for (var i = 0; i < length; i++)
            {
                JPH_BodyState bodyState = default;
                bodyState.Deserialize(ref reader);
            }
            contacts.Deserialize(ref reader);
        }
    }

    public partial struct JPH_GlobalState
    {
        public readonly void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteFloat(previousStepDeltaTime);
            writer.WriteFloat3(gravity);
        }

        public void Deserialize(ref DataStreamReader reader)
        {
            previousStepDeltaTime = reader.ReadFloat();
            gravity = reader.ReadFloat3();
        }
    }
    
    public partial struct JPH_BodyState
    {
        public readonly void Serialize(ref DataStreamWriter writer)
        {
            writer.WriteUInt(id);
            var bits = 0u;
            if (isActive > 0) bits |= 1 << 0;
            if (isSoft > 0) bits |= 1 << 1;
            writer.WriteRawBits(bits, 2);
            
            writer.WriteFloat3(position);
            
            writer.WriteQuat(rotation);
            
            motionProperties.Serialize(ref writer);
        }

        public void Deserialize(ref DataStreamReader reader)
        {
            id = reader.ReadUInt();
            var bits = reader.ReadRawBits(2);
            isActive = (bits & (1 << 0)) != 0 ? (byte)1 : (byte)0;
            isSoft = (bits & (1 << 1)) != 0 ? (byte)1 : (byte)0;
            
            position = reader.ReadFloat3();
            rotation = reader.ReadQuat();
            
            motionProperties.Deserialize(ref reader);
        }
    }

    public partial struct JPH_MotionPropertiesState
    {
        public readonly void Serialize(ref DataStreamWriter writer)
        {
            
        }
        
        public void Deserialize(ref DataStreamReader reader)
        {
            
        }
    }

    public partial struct JPH_ContactConstraintState
    {
        public readonly void Serialize(ref DataStreamWriter writer)
        {
            
        }
        
        public void Deserialize(ref DataStreamReader reader)
        {
            
        }
    }
}
