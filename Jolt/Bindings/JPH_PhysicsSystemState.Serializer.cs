using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Jolt
{
    public interface ISerializable
    {
        void Serialize(ref DataStreamWriter writer);
        void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader);
    }
    
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
    
    public partial struct JPH_PhysicsSystemState : ISerializable
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
    }

    public partial struct JPH_GlobalState : ISerializable
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
    }
    
    public partial struct JPH_BodyState : ISerializable
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
    }

    public partial struct JPH_MotionPropertiesState : ISerializable
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
    }

    public partial struct JPH_Sphere : ISerializable
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
    }

    public partial struct JPH_ContactConstraintState : ISerializable
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            manifold.Serialize(ref writer);
        }
        
        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            manifold.Deserialize(builder, ref reader);
        }
    }

    public partial struct JPH_ManifoldCacheState : ISerializable
    {
        public void Serialize(ref DataStreamWriter writer)
        {
            var bodyLength = (byte)bodyPairs.Length;
            writer.WriteByte(bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairs[i].Serialize(ref writer);
            }

            var ccdLength = (byte)ccdManifolds.Length;
            writer.WriteByte(ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdManifolds[i].Serialize(ref writer);
            }
        }

        public void Deserialize(NativeBlobBuilder builder, ref DataStreamReader reader)
        {
            var bodyLength = reader.ReadByte();
            var bodyPairArrayBuilder = builder.Allocate(ref bodyPairs, bodyLength);
            for (var i = 0; i < bodyLength; i++)
            {
                bodyPairArrayBuilder[i].Deserialize(builder, ref reader);
            }

            var ccdLength = reader.ReadByte();
            var ccdArrayBuilder = builder.Allocate(ref ccdManifolds, ccdLength);
            for (var i = 0; i < ccdLength; i++)
            {
                ccdArrayBuilder[i].Deserialize(builder, ref reader);
            }
        }
    }
    
    public partial struct JPH_BodyPairKeyValueState : ISerializable
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
    }

    public partial struct JPH_CachedBodyPairState : ISerializable
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
    }

    public partial struct JPH_ManifoldKeyValueState : ISerializable
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
    }

    public partial struct JPH_SubShapeIDPair : ISerializable
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
    }

    public partial struct JPH_CachedManifoldState : ISerializable
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
    }

    public partial struct JPH_CachedContactPointState : ISerializable
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
    }
}
