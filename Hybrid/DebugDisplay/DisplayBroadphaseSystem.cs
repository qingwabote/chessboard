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
    internal partial struct DisplayBroadphaseAabbsSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
            state.RequireForUpdate<PhysicsWorldSingleton>();
        }

        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawBroadphase == 0)
                return;

            CollisionWorld world = SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
            ReadOnlySpan<int> cellStarts = world.CellBodyIndices;
            int gridWidth = world.Grid.Max.x - world.Grid.Min.x + 1;
            int2 gridMin = world.Grid.Min;
            float cellSize = world.CellSize;

            for (int i = 0; i < cellStarts.Length - 1; i++)
            {
                if (cellStarts[i] == cellStarts[i + 1])
                    continue;

                int2 cell = new int2(gridMin.x + i % gridWidth, gridMin.y + i / gridWidth);
                float3 center = new float3((cell.x + 0.5f) * cellSize, 0f, (cell.y + 0.5f) * cellSize);
                PhysicsDebugDisplaySystem.Box(new float3(cellSize, 0f, cellSize), center, quaternion.identity, Unity.DebugDisplay.ColorIndex.Cyan);
            }

            int gridHeight = world.Grid.Max.y - world.Grid.Min.y + 1;
            float3 boardSize = new float3(gridWidth * cellSize, 0f, gridHeight * cellSize);
            float3 boardCenter = new float3(gridMin.x * cellSize, 0f, gridMin.y * cellSize) + boardSize * 0.5f;
            PhysicsDebugDisplaySystem.Box(boardSize, boardCenter, quaternion.identity, Unity.DebugDisplay.ColorIndex.BrightWhite);
        }
    }
#endif
}
