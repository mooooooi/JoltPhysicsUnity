using Unity.Burst;
using Unity.Jobs;

namespace Jolt.Job
{
    [BurstCompile]
    public unsafe struct UpdatePhysicsSystemJob : IJob
    {
        public PhysicsSystem physics;
        public float deltaTime;
        public JobSystem job;

        public void Execute()
        {
            physics.Update(deltaTime, 1, job.ToUnsafePtr());
        }
    }
}
