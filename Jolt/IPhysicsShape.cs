using Jolt;

namespace Jolt
{
    public interface IPhysicsShape
    {
        public Shape GetOrCreateShape();
    }
}
