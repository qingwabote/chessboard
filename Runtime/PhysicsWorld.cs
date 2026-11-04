using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct CollisionWorld
    {
        public ref struct BodyVisitor
        {
            CollisionWorld m_World;
            readonly int m_Epoch;

            internal BodyVisitor(CollisionWorld world)
            {
#if UNITY_EDITOR
                UnityEngine.Debug.Assert(!world.m_BodyVisiting.Value, "Cannot open a BodyVisitor while another BodyVisitor is open");
                world.m_BodyVisiting.Value = true;
#endif
                m_Epoch = ++world.m_QueryEpoch.Value;
                m_World = world;
            }

            public BodyRange Visit(Range2 cells)
            {
#if UNITY_EDITOR
                UnityEngine.Debug.Assert(m_World.m_BodyVisiting.Value, "BodyVisitor is no longer the open visitor");
#endif
                return new BodyRange(m_World.m_CellBodyIndices, m_World.m_BodyIndices, m_World.m_BodyEpochs, m_Epoch, cells, m_World.Grid.Min, m_World.Grid.Max.x - m_World.Grid.Min.x + 1);
            }

            public void Dispose()
            {
#if UNITY_EDITOR
                m_World.m_BodyVisiting.Value = false;
#endif
            }
        }

        public const float DefaultCollisionTolerance = 0.01f;

        public int NumBodies => m_Bodies.Length;
        public int NumStaticBodies => NumBodies - NumDynamicBodies;

        public NativeArray<RigidBody> Bodies => m_Bodies;
        public NativeArray<RigidBody> DynamicBodies => m_Bodies.GetSubArray(0, NumDynamicBodies);
        public NativeArray<RigidBody> StaticBodies => m_Bodies.GetSubArray(NumDynamicBodies, NumStaticBodies);

        internal ReadOnlySpan<Aabb2> Eabbs => m_Eabbs.AsSpan();
        public ReadOnlySpan<CollisionFilter> Filters => m_Filters.AsSpan();
        internal ReadOnlySpan<byte> MaterialFlags => m_MaterialFlags.AsSpan();

        private NativeArray<RigidBody> m_Bodies;
        private NativeArray<int> m_BodyIndices;
        private NativeArray<int> m_CellBodyIndices;

        private NativeArray<Aabb2> m_Eabbs;
        private NativeArray<CollisionFilter> m_Filters;
        private NativeArray<byte> m_MaterialFlags;
        private NativeArray<int> m_BodyEpochs;
        private NativeReference<int> m_QueryEpoch;
#if UNITY_EDITOR
        private NativeReference<bool> m_BodyVisiting;
#endif

        public readonly Range2 Grid;
        public readonly float CellSize;
        public readonly int NumDynamicBodies;

        internal CollisionWorld(
            NativeArray<RigidBody> bodies, NativeArray<Aabb2> eabbs,
            NativeArray<CollisionFilter> filters, NativeArray<byte> materialFlags,
            NativeArray<int> cellBodyIndices, NativeArray<int> bodyIndices,
            NativeArray<int> bodyEpochs, NativeReference<int> queryEpoch,
#if UNITY_EDITOR
            NativeReference<bool> bodyVisiting,
#endif
            int numDynamicBodies, Range2 grid, float cellSize)
        {
            m_Bodies = bodies;
            m_Eabbs = eabbs;
            m_Filters = filters;
            m_MaterialFlags = materialFlags;
            m_CellBodyIndices = cellBodyIndices;
            m_BodyIndices = bodyIndices;
            m_BodyEpochs = bodyEpochs;
            m_QueryEpoch = queryEpoch;
#if UNITY_EDITOR
            m_BodyVisiting = bodyVisiting;
#endif
            NumDynamicBodies = numDynamicBodies;
            Grid = grid;
            CellSize = cellSize;
        }

        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)
        {
            if (input.Filter.IsEmpty)
            {
                return false;
            }
            int hitsBefore = allHits.Length;
            Aabb2 aabb = input.Aabb;
            foreach (int bodyIndex in BodyRange(aabb))
            {
                if (!m_Eabbs[bodyIndex].Overlaps(aabb)) continue;
                if (!CollisionFilter.IsCollisionEnabled(input.Filter, m_Filters[bodyIndex])) continue;
                allHits.Add(bodyIndex);
            }
            return allHits.Length > hitsBefore;
        }

        public bool OverlapSphere(
            float3 position, float radius, ref NativeList<DistanceHit> outHits,
            CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
        {
            if (m_Bodies.Length == 0 || filter.IsEmpty)
            {
                return false;
            }
            int hitsBefore = outHits.Length;
            foreach (int bodyIndex in BodyRange(new Aabb2 { Min = position.xz - radius, Max = position.xz + radius }))
            {
                if (TryDistance(bodyIndex, position, filter, out var hit) && hit.Fraction <= radius)
                {
                    outHits.Add(hit);
                }
            }
            return outHits.Length > hitsBefore;
        }

        internal BodyRange BodyRange(Aabb2 aabb)
        {
#if UNITY_EDITOR
            UnityEngine.Debug.Assert(!m_BodyVisiting.Value, "Cannot run a collision query while a BodyVisitor is open");
#endif
            return new BodyRange(m_CellBodyIndices, m_BodyIndices, m_BodyEpochs, ++m_QueryEpoch.Value, math.clamp((int4)math.floor(aabb.Val / CellSize), Grid.Min.xyxy, Grid.Max.xyxy), Grid.Min, Grid.Max.x - Grid.Min.x + 1);
        }

        public BodyVisitor CreateBodyVisitor()
        {
            return new BodyVisitor(this);
        }

        public unsafe bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits)
        {
            if (m_Bodies.Length == 0 || input.Collider == null)
            {
                return false;
            }
            int hitsBefore = allHits.Length;
            float scale = input.QueryColliderScale > 0f ? input.QueryColliderScale : 1f;
            var startAabb = Collider.CalculateAabb(input.Collider, new RigidTransform(input.Orientation, input.Start), scale);
            var endAabb = Collider.CalculateAabb(input.Collider, new RigidTransform(input.Orientation, input.End), scale);
            foreach (int bodyIndex in BodyRange(Aabb2.Union(startAabb, endAabb)))
            {
                var body = m_Bodies[bodyIndex];
                var bodyFilter = ((Collider*)body.Collider.GetUnsafePtr())->GetCollisionFilter();
                if (!CollisionFilter.IsCollisionEnabled(bodyFilter, input.Collider->GetCollisionFilter())) continue;
                if (TryCast(input, scale, bodyIndex, out var hit))
                {
                    allHits.Add(hit);
                }
            }
            return allHits.Length > hitsBefore;
        }

        internal bool TryDistance(int bodyIndex, float3 position, CollisionFilter filter, out DistanceHit hit)
        {
            hit = default;
            var body = m_Bodies[bodyIndex];
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

    public struct BodyRange
    {
        Range2.Enumerator m_Cells;
        NativeArray<int> m_CellBodyIndices;
        NativeArray<int> m_BodyIndices;
        NativeArray<int> m_BodyEpochs;
        readonly int m_Epoch;
        readonly int2 m_GridMin;
        readonly int m_GridWidth;
        int m_Head;
        int m_Tail;
        int m_Current;

        internal BodyRange(
            NativeArray<int> cellBodyIndices, NativeArray<int> bodyIndices, NativeArray<int> bodyEpochs,
            int epoch, Range2 cells, int2 gridMin, int gridWidth)
        {
            m_CellBodyIndices = cellBodyIndices;
            m_BodyIndices = bodyIndices;
            m_BodyEpochs = bodyEpochs;
            m_Epoch = epoch;
            m_GridMin = gridMin;
            m_GridWidth = gridWidth;
            m_Cells = cells.GetEnumerator();
            m_Head = 0;
            m_Tail = 0;
            m_Current = 0;
        }

        public BodyRange GetEnumerator() => this;

        public int Current => m_Current;

        public bool MoveNext()
        {
            while (true)
            {
                if (m_Head < m_Tail)
                {
                    int bodyIndex = m_BodyIndices[m_Head++];
                    if (m_BodyEpochs[bodyIndex] == m_Epoch) continue;
                    m_BodyEpochs[bodyIndex] = m_Epoch;
                    m_Current = bodyIndex;
                    return true;
                }
                if (!m_Cells.MoveNext()) return false;
                int cellIndex = (m_Cells.Current.x - m_GridMin.x) + (m_Cells.Current.y - m_GridMin.y) * m_GridWidth;
                m_Head = m_CellBodyIndices[cellIndex];
                m_Tail = m_CellBodyIndices[cellIndex + 1];
            }
        }
    }

    public struct DynamicsWorld
    {
        internal ReadOnlySpan<byte> Kinematics => m_Kinematics.AsSpan();
        internal ReadOnlySpan<float> InvMasses => m_InvMasses.AsSpan();
        public NativeArray<MotionVelocity> MotionVelocities => m_MotionVelocities;
        internal ReadOnlySpan<float> DampingLinear => m_DampingLinear.AsSpan();
        internal ReadOnlySpan<float> DampingAngular => m_DampingAngular.AsSpan();

        private NativeArray<byte> m_Kinematics;
        private NativeArray<float> m_InvMasses;
        private NativeArray<MotionVelocity> m_MotionVelocities;
        private NativeArray<float> m_DampingLinear;
        private NativeArray<float> m_DampingAngular;

        internal DynamicsWorld(
            NativeArray<byte> kinematics, NativeArray<float> invMasses,
            NativeArray<MotionVelocity> motionVelocities, NativeArray<float> dampingLinear,
            NativeArray<float> dampingAngular)
        {
            m_Kinematics = kinematics;
            m_InvMasses = invMasses;
            m_MotionVelocities = motionVelocities;
            m_DampingLinear = dampingLinear;
            m_DampingAngular = dampingAngular;
        }
    }

    public struct PhysicsWorld
    {
        public CollisionWorld CollisionWorld;
        public DynamicsWorld DynamicsWorld;

        public int NumBodies => CollisionWorld.NumBodies;
        public int NumDynamicBodies => CollisionWorld.NumDynamicBodies;
        public int NumStaticBodies => CollisionWorld.NumStaticBodies;

        public NativeArray<RigidBody> Bodies => CollisionWorld.Bodies;
        public NativeArray<RigidBody> DynamicBodies => CollisionWorld.DynamicBodies;
        public NativeArray<RigidBody> StaticBodies => CollisionWorld.StaticBodies;

        public NativeArray<MotionVelocity> MotionVelocities => DynamicsWorld.MotionVelocities;

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

        public NativeArray<MotionVelocity> MotionVelocities => PhysicsWorld.MotionVelocities;

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
        internal NativeList<TriggerEventData> TriggerEvents;
        internal NativeList<CollisionEventData> CollisionEvents;
    }
}
