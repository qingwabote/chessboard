using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct CollisionWorld
    {
        public const float DefaultCollisionTolerance = 0.01f;

        public float CollisionTolerance;

        internal NativeArray<RigidBody> m_Bodies;
        internal NativeArray<Aabb> m_Eabbs;
        internal NativeArray<CollisionFilter> m_Filters;
        internal NativeArray<byte> m_MaterialFlags;
        internal NativeArray<byte> m_Kinematics;
        internal NativeArray<float> m_InvMasses;
        internal NativeArray<float3> m_LinearVelocities;
        internal NativeArray<float> m_GravityFactors;
        internal NativeArray<float> m_DampingLinear;
        internal NativeArray<float> m_DampingAngular;
        internal int m_NumDynamicBodies;
        internal NativeArray<int> m_CellStart;
        internal NativeArray<int> m_CellOrder;
        internal int2 m_GridMinCell;
        internal int2 m_GridCount;
        internal float m_CellSize;

        public int NumBodies => m_Bodies.IsCreated ? m_Bodies.Length : 0;
        public int NumDynamicBodies => m_NumDynamicBodies;
        public int NumStaticBodies => NumBodies - m_NumDynamicBodies;

        public NativeArray<RigidBody> Bodies => m_Bodies;

        public NativeArray<RigidBody> DynamicBodies
        {
            get { return m_Bodies.GetSubArray(0, m_NumDynamicBodies); }
        }

        public NativeArray<RigidBody> StaticBodies
        {
            get { return m_Bodies.GetSubArray(m_NumDynamicBodies, NumStaticBodies); }
        }

        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)
        {
            if (!m_CellStart.IsCreated || m_CellSize <= 0f || input.Filter.IsEmpty)
            {
                return false;
            }
            int hitsBefore = allHits.Length;
            var cells = CellRange(input.Aabb);
            for (int x = cells.x; x <= cells.y; x++)
            {
                for (int z = cells.z; z <= cells.w; z++)
                {
                    int cellIndex = (x - m_GridMinCell.x) + (z - m_GridMinCell.y) * m_GridCount.x;
                    for (int k = m_CellStart[cellIndex]; k < m_CellStart[cellIndex + 1]; k++)
                    {
                        int bodyIndex = m_CellOrder[k];
                        if (!m_Eabbs[bodyIndex].Overlaps(input.Aabb)) continue;
                        if (!CollisionFilter.IsCollisionEnabled(input.Filter, m_Filters[bodyIndex])) continue;
                        bool duplicate = false;
                        for (int d = hitsBefore; d < allHits.Length; d++)
                        {
                            if (allHits[d] == bodyIndex) { duplicate = true; break; }
                        }
                        if (!duplicate) allHits.Add(bodyIndex);
                    }
                }
            }
            return allHits.Length > hitsBefore;
        }

        internal int4 CellRange(Aabb aabb)
        {
            var min = (int2)math.floor(aabb.Min.xz / m_CellSize);
            var max = (int2)math.floor(aabb.Max.xz / m_CellSize);
            var gridMax = m_GridMinCell + m_GridCount - 1;
            min = math.clamp(min, m_GridMinCell, gridMax);
            max = math.clamp(max, m_GridMinCell, gridMax);
            return new int4(min.x, max.x, min.y, max.y);
        }

        public bool OverlapSphere(
            float3 position, float radius, ref NativeList<DistanceHit> outHits,
            CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
        {
            if (!m_Bodies.IsCreated || filter.IsEmpty)
            {
                return false;
            }
            int hitsBefore = outHits.Length;
            var candidates = new NativeList<int>(16, Allocator.Temp);
            GatherCandidates(new Aabb { Min = position - new float3(radius), Max = position + new float3(radius) }, ref candidates);
            for (int i = 0; i < candidates.Length; i++)
            {
                if (TryDistance(candidates[i], position, filter, out var hit) && hit.Fraction <= radius)
                {
                    outHits.Add(hit);
                }
            }
            candidates.Dispose();
            return outHits.Length > hitsBefore;
        }

        public unsafe bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits)
        {
            if (!m_Bodies.IsCreated || input.Collider == null)
            {
                return false;
            }
            int hitsBefore = allHits.Length;
            float scale = input.QueryColliderScale > 0f ? input.QueryColliderScale : 1f;
            var startAabb = Collider.CalculateAabb(input.Collider, new RigidTransform(input.Orientation, input.Start), scale);
            var endAabb = Collider.CalculateAabb(input.Collider, new RigidTransform(input.Orientation, input.End), scale);
            var candidates = new NativeList<int>(16, Allocator.Temp);
            GatherCandidates(Aabb.Union(startAabb, endAabb), ref candidates);
            for (int i = 0; i < candidates.Length; i++)
            {
                int bodyIndex = candidates[i];
                var body = m_Bodies[bodyIndex];
                if (!body.Collider.IsCreated) continue;
                var bodyFilter = ((Collider*)body.Collider.GetUnsafePtr())->GetCollisionFilter();
                if (!CollisionFilter.IsCollisionEnabled(bodyFilter, input.Collider->GetCollisionFilter())) continue;
                if (TryCast(input, scale, bodyIndex, out var hit))
                {
                    allHits.Add(hit);
                }
            }
            candidates.Dispose();
            return allHits.Length > hitsBefore;
        }

        internal void GatherCandidates(Aabb aabb, ref NativeList<int> results)
        {
            if (!m_CellStart.IsCreated || m_CellSize <= 0f)
            {
                return;
            }
            var cells = CellRange(aabb);
            for (int x = cells.x; x <= cells.y; x++)
            {
                for (int z = cells.z; z <= cells.w; z++)
                {
                    int cellIndex = (x - m_GridMinCell.x) + (z - m_GridMinCell.y) * m_GridCount.x;
                    for (int k = m_CellStart[cellIndex]; k < m_CellStart[cellIndex + 1]; k++)
                    {
                        int bodyIndex = m_CellOrder[k];
                        bool duplicate = false;
                        for (int d = 0; d < results.Length; d++)
                        {
                            if (results[d] == bodyIndex) { duplicate = true; break; }
                        }
                        if (!duplicate) results.Add(bodyIndex);
                    }
                }
            }
        }

        internal unsafe bool TryDistance(int bodyIndex, float3 position, CollisionFilter filter, out DistanceHit hit)
        {
            hit = default;
            var body = m_Bodies[bodyIndex];
            if (!body.Collider.IsCreated) return false;
            if (!CollisionFilter.IsCollisionEnabled(filter, m_Filters[bodyIndex])) return false;

            var leaves = new NativeList<CollisionTests.Leaf>(4, Allocator.Temp);
            CollisionTests.CollectLeaves(body, leaves);
            var best = float.MaxValue;
            var bestNormal = float3.zero;
            var bestPoint = float3.zero;
            var found = false;
            for (int i = 0; i < leaves.Length; i++)
            {
                var distance = CollisionTests.PointLeafDistance(leaves[i], position, out var normal, out var point);
                if (!found || distance < best)
                {
                    best = distance;
                    bestNormal = normal;
                    bestPoint = point;
                    found = true;
                }
            }
            leaves.Dispose();
            if (!found) return false;
            hit = new DistanceHit
            {
                Fraction = best,
                RigidBodyIndex = bodyIndex,
                Entity = body.Entity,
                Position = bestPoint,
                SurfaceNormal = bestNormal
            };
            return true;
        }

        internal unsafe bool TryCast(ColliderCastInput input, float scale, int bodyIndex, out ColliderCastHit hit)
        {
            hit = default;
            var body = m_Bodies[bodyIndex];
            if (!body.Collider.IsCreated) return false;

            var castLeaves = new NativeList<CollisionTests.Leaf>(4, Allocator.Temp);
            var bodyLeaves = new NativeList<CollisionTests.Leaf>(4, Allocator.Temp);
            CollisionTests.CollectLeaves(input.Collider, new RigidTransform(input.Orientation, input.Start), scale, castLeaves);
            CollisionTests.CollectLeaves(body, bodyLeaves);

            var displacement = input.End - input.Start;
            float length = math.length(displacement);
            float tolerance = 1e-3f;
            var direction = length > 1e-9f ? displacement / length : float3.zero;

            float travelled = 0f;
            for (int iteration = 0; iteration < 10; iteration++)
            {
                for (int c = 0; c < castLeaves.Length; c++)
                {
                    var castLeaf = castLeaves[c];
                    castLeaf.World.pos += direction * travelled;
                    castLeaf.ShadowCenter += new float2(direction.x, direction.z) * travelled;
                    for (int b = 0; b < bodyLeaves.Length; b++)
                    {
                        if (CollisionTests.TestLeaves(castLeaf, bodyLeaves[b], out var normal, out _))
                        {
                            hit = new ColliderCastHit
                            {
                                Fraction = length > 1e-9f ? travelled / length : 0f,
                                RigidBodyIndex = bodyIndex,
                                Entity = body.Entity,
                                Position = castLeaf.World.pos,
                                SurfaceNormal = normal
                            };
                            castLeaves.Dispose();
                            bodyLeaves.Dispose();
                            return true;
                        }
                    }
                }

                var bestDistance = float.MaxValue;
                var bestNormal = new float3(0f, 1f, 0f);
                for (int c = 0; c < castLeaves.Length; c++)
                {
                    var castLeaf = castLeaves[c];
                    castLeaf.World.pos += direction * travelled;
                    castLeaf.ShadowCenter += new float2(direction.x, direction.z) * travelled;
                    for (int b = 0; b < bodyLeaves.Length; b++)
                    {
                        var distance = CollisionTests.LeafPairDistance(castLeaf, bodyLeaves[b], out var normal);
                        if (distance < bestDistance)
                        {
                            bestDistance = distance;
                            bestNormal = -normal;
                        }
                    }
                }

                if (bestDistance < tolerance)
                {
                    hit = new ColliderCastHit
                    {
                        Fraction = length > 1e-9f ? travelled / length : 0f,
                        RigidBodyIndex = bodyIndex,
                        Entity = body.Entity,
                        Position = body.WorldFromBody.pos,
                        SurfaceNormal = bestNormal
                    };
                    castLeaves.Dispose();
                    bodyLeaves.Dispose();
                    return true;
                }

                if (length <= 1e-9f) break;
                float projection = math.dot(-bestNormal, direction);
                if (projection <= 1e-6f) break;
                // 0.5f: fraction of tolerance left as the gap after advancing, so the next-iteration distance (≈ tolerance / 2) strictly clears the `bestDistance < tolerance` hit threshold instead of landing exactly on it and being decided by float rounding. Any fraction safely below 1 works; 0.5 is the half-tolerance idiom (same as the CollisionTolerance margins), not tuned — it caps the hit-fraction error at 0.5 mm.
                travelled += (bestDistance - tolerance * 0.5f) / projection;
                if (travelled > length) break;
            }

            castLeaves.Dispose();
            bodyLeaves.Dispose();
            return false;
        }
    }

    public struct PhysicsWorld
    {
        public CollisionWorld CollisionWorld;

        public int NumBodies => CollisionWorld.NumBodies;
        public int NumDynamicBodies => CollisionWorld.NumDynamicBodies;
        public int NumStaticBodies => CollisionWorld.NumStaticBodies;

        public NativeArray<RigidBody> Bodies => CollisionWorld.Bodies;
        public NativeArray<RigidBody> DynamicBodies => CollisionWorld.DynamicBodies;
        public NativeArray<RigidBody> StaticBodies => CollisionWorld.StaticBodies;

        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits) =>
            CollisionWorld.OverlapAabb(input, ref allHits);

        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) =>
            CollisionWorld.CastCollider(input, ref allHits);

        public bool OverlapSphere(
            float3 position, float radius, ref NativeList<DistanceHit> outHits,
            CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) =>
            CollisionWorld.OverlapSphere(position, radius, ref outHits, filter, queryInteraction);
    }

    public struct PhysicsWorldSingleton : IComponentData
    {
        public PhysicsWorld PhysicsWorld;

        public CollisionWorld CollisionWorld => PhysicsWorld.CollisionWorld;

        public int NumBodies => PhysicsWorld.NumBodies;
        public int NumStaticBodies => PhysicsWorld.NumStaticBodies;
        public int NumDynamicBodies => PhysicsWorld.NumDynamicBodies;

        public NativeArray<RigidBody> Bodies => PhysicsWorld.Bodies;
        public NativeArray<RigidBody> StaticBodies => PhysicsWorld.StaticBodies;
        public NativeArray<RigidBody> DynamicBodies => PhysicsWorld.DynamicBodies;

        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits) =>
            PhysicsWorld.OverlapAabb(input, ref allHits);

        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) =>
            PhysicsWorld.CastCollider(input, ref allHits);

        public bool OverlapSphere(
            float3 position, float radius, ref NativeList<DistanceHit> outHits,
            CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) =>
            PhysicsWorld.OverlapSphere(position, radius, ref outHits, filter, queryInteraction);
    }

    public struct SimulationSingleton : IComponentData
    {
        public SimulationType Type;
        internal NativeList<TriggerEventData> TriggerEvents;
        internal NativeList<CollisionEventData> CollisionEvents;
        internal JobHandle EventDependency;
    }
}
