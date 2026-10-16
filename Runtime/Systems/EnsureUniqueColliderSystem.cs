using Unity.Burst;
using Unity.Entities;
using Unity.Physics.Extensions;

namespace Unity.Physics.Systems
{
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(BeforePhysicsSystemGroup), OrderFirst = true)]
    public partial struct EnsureUniqueColliderSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = SystemAPI.GetSingleton<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>()
                .CreateCommandBuffer(state.WorldUnmanaged);

            foreach (var (collider, entity) in
                SystemAPI.Query<RefRW<PhysicsCollider>>()
                    .WithAll<EnsureUniqueColliderBlobTag>()
                    .WithEntityAccess())
            {
                if (!collider.ValueRO.IsUnique)
                {
                    collider.ValueRW.MakeUnique(entity, ecb);
                    ecb.RemoveComponent<EnsureUniqueColliderBlobTag>(entity);
                }
            }
        }
    }

    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct DisposeColliderBlobClonesSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<EndInitializationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = SystemAPI.GetSingleton<EndInitializationEntityCommandBufferSystem.Singleton>()
                .CreateCommandBuffer(state.WorldUnmanaged);

            foreach (var (cleanup, entity) in
                SystemAPI.Query<RefRO<ColliderBlobCleanupData>>()
                    .WithNone<PhysicsCollider>()
                    .WithEntityAccess())
            {
                if (cleanup.ValueRO.Value.IsCreated)
                {
                    cleanup.ValueRO.Value.Dispose();
                }
                ecb.RemoveComponent<ColliderBlobCleanupData>(entity);
            }
        }
    }
}
