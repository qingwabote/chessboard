#if UNITY_EDITOR

using Unity.Collections;
using Unity.Entities;

namespace Unity.Physics.Authoring
{
    [UpdateInGroup(typeof(BakingSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial struct UniquePrefabColliderBakingSystem : ISystem
    {
        public unsafe void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (collider, entity) in
                SystemAPI.Query<RefRW<PhysicsCollider>>()
                    .WithAll<Prefab>()
                    .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)
                    .WithEntityAccess())
            {
                if (!collider.ValueRO.IsUnique)
                {
                    continue;
                }
                ((Collider*)collider.ValueRW.Value.GetUnsafePtr())->SetForceUniqueID(ColliderConstants.k_SharedBlobID);
                ecb.AddComponent<EnsureUniqueColliderBlobTag>(entity);
            }
            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}

#endif
