using System;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Authoring
{
#if UNITY_EDITOR
    [BurstCompile]
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup))]
    internal partial struct DisplayColliderAabbsSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
            state.RequireForUpdate<PhysicsWorldSingleton>();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliderAabbs == 0)
                return;

            ReadOnlySpan<Aabb2> eabbs = SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld.Eabbs;

            for (int i = 0; i < eabbs.Length; i++)
            {
                Aabb2 eabb = eabbs[i];
                float2 extents = eabb.Extents;
                float2 center = eabb.Center;
                PhysicsDebugDisplaySystem.Box(new float3(extents.x, 0f, extents.y), new float3(center.x, 0f, center.y), quaternion.identity, Unity.DebugDisplay.ColorIndex.BrightRed);
            }
        }
    }
#endif
}
