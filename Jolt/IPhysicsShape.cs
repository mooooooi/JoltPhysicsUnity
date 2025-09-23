using Jolt;
using Unity.Mathematics;

namespace Jolt
{
    public interface IPhysicsShape
    {
        public Shape GetOrCreateShape();

        public JPH_Plane GetSupportingVolume()
        {
            throw new System.NotImplementedException();
        }
    }
}
