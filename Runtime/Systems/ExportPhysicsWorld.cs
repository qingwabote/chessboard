using Unity.Burst;
using Unity.Entities;

namespace Unity.Physics.Systems
{
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsSimulationGroup))]
    public partial struct ExportPhysicsWorld : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
        }
    }
}
