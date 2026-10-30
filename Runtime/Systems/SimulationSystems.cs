using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    internal struct Contact
    {
        public int A;
        public int B;
        public float3 Normal;
        public float Depth;
    }

    internal struct SpeculativeContact
    {
        public int A;
        public int B;
        public float3 Normal;
        public float Distance;
        public bool RaisesEvents;
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    public partial struct StepPhysicsWorld : ISystem
    {
        NativeList<int2> m_Pairs;
        NativeList<Contact> m_Contacts;
        NativeList<SpeculativeContact> m_Speculative;
        NativeArray<int> m_Seen;
        int m_Epoch;
        NativeList<CollisionTests.Leaf> m_LeavesA;
        NativeList<CollisionTests.Leaf> m_LeavesB;
        NativeList<float3> m_Positions;
        NativeList<TriggerEventData> m_TriggerEvents;
        NativeList<CollisionEventData> m_CollisionEvents;

        public void OnCreate(ref SystemState state)
        {
            var persistent = Allocator.Persistent;
            m_Pairs = new NativeList<int2>(256, persistent);
            m_Contacts = new NativeList<Contact>(128, persistent);
            m_Speculative = new NativeList<SpeculativeContact>(64, persistent);
            m_Seen = new NativeArray<int>(256, persistent, NativeArrayOptions.ClearMemory);
            m_Epoch = 0;
            m_LeavesA = new NativeList<CollisionTests.Leaf>(4, persistent);
            m_LeavesB = new NativeList<CollisionTests.Leaf>(4, persistent);
            m_Positions = new NativeList<float3>(256, persistent);
            m_TriggerEvents = new NativeList<TriggerEventData>(64, persistent);
            m_CollisionEvents = new NativeList<CollisionEventData>(64, persistent);
            var entity = state.EntityManager.CreateEntity(typeof(SimulationSingleton));
            state.EntityManager.SetName(entity, "SimulationSingleton");
            SystemAPI.SetSingleton(new SimulationSingleton
            {
                TriggerEvents = m_TriggerEvents,
                CollisionEvents = m_CollisionEvents
            });
        }

        public void OnDestroy(ref SystemState state)
        {
            if (m_Pairs.IsCreated) m_Pairs.Dispose();
            if (m_Contacts.IsCreated) m_Contacts.Dispose();
            if (m_Speculative.IsCreated) m_Speculative.Dispose();
            if (m_Seen.IsCreated) m_Seen.Dispose();
            if (m_LeavesA.IsCreated) m_LeavesA.Dispose();
            if (m_LeavesB.IsCreated) m_LeavesB.Dispose();
            if (m_Positions.IsCreated) m_Positions.Dispose();
            if (m_TriggerEvents.IsCreated) m_TriggerEvents.Dispose();
            if (m_CollisionEvents.IsCreated) m_CollisionEvents.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.CompleteDependency();
            var step = SystemAPI.TryGetSingleton(out PhysicsStep physicsStep) ? physicsStep : PhysicsStep.Default;
            m_TriggerEvents.Clear();
            m_CollisionEvents.Clear();

            var collisionWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld.CollisionWorld;
            int numBodies = collisionWorld.NumBodies;
            if (numBodies == 0)
            {
                return;
            }

            int numDynamic = collisionWorld.m_NumDynamicBodies;
            var bodies = collisionWorld.m_Bodies;
            var eabbs = collisionWorld.m_Eabbs;
            var filters = collisionWorld.m_Filters;
            var materialFlags = collisionWorld.m_MaterialFlags;
            var kinematics = collisionWorld.m_Kinematics;
            var invMasses = collisionWorld.m_InvMasses;
            float cellSize = collisionWorld.m_CellSize;
            float dt = SystemAPI.Time.DeltaTime;
            var simulation = SystemAPI.GetSingleton<SimulationSingleton>();

            if (m_Seen.Length < numBodies)
            {
                m_Seen.Dispose();
                m_Seen = new NativeArray<int>(math.max(256, math.ceilpow2(numBodies)), Allocator.Persistent, NativeArrayOptions.ClearMemory);
                m_Epoch = 0;
            }

            m_Pairs.Clear();
            for (int i = 0; i < numDynamic; i++)
            {
                m_Epoch++;
                uint belongsI = filters[i].BelongsTo;
                uint collidesI = filters[i].CollidesWith;
                byte flagsI = materialFlags[i];
                var eabbI = eabbs[i];
                int2 min = (int2)math.floor(eabbI.Min / cellSize);
                int2 max = (int2)math.floor(eabbI.Max / cellSize);
                var gridMax = collisionWorld.m_GridMin + collisionWorld.m_GridSize - 1;
                min = math.clamp(min, collisionWorld.m_GridMin, gridMax);
                max = math.clamp(max, collisionWorld.m_GridMin, gridMax);
                for (int x = min.x; x <= max.x; x++)
                {
                    for (int z = min.y; z <= max.y; z++)
                    {
                        int cellIndex = (x - collisionWorld.m_GridMin.x) + (z - collisionWorld.m_GridMin.y) * collisionWorld.m_GridSize.x;
                        for (int k = collisionWorld.m_CellBodyIndices[cellIndex]; k < collisionWorld.m_CellBodyIndices[cellIndex + 1]; k++)
                        {
                            int j = collisionWorld.m_BodyIndices[k];
                            if (j == i) continue;
                            if (j < numDynamic && j < i) continue;
                            if (m_Seen[j] == m_Epoch) continue;
                            var filterJ = filters[j];
                            if ((collidesI & filterJ.BelongsTo) == 0) continue;
                            if ((filterJ.CollidesWith & belongsI) == 0) continue;
                            m_Seen[j] = m_Epoch;
                            var eabbJ = eabbs[j];
                            if (math.any(eabbI.Max < eabbJ.Min) || math.any(eabbI.Min > eabbJ.Max)) continue;
                            if (((MaterialFlags)(flagsI | materialFlags[j]) & MaterialFlags.DisableCollisions) != 0) continue;
                            m_Pairs.Add(new int2(i, j));
                        }
                    }
                }
            }

            m_Contacts.Clear();
            m_Speculative.Clear();
            for (int p = 0; p < m_Pairs.Length; p++)
            {
                int i = m_Pairs[p].x;
                int j = m_Pairs[p].y;
                bool bothKinematic = kinematics[i] != 0 && kinematics[j] != 0;

                m_LeavesA.Clear();
                m_LeavesB.Clear();
                CollisionTests.CollectLeaves(bodies[i], m_LeavesA);
                CollisionTests.CollectLeaves(bodies[j], m_LeavesB);

                for (int la = 0; la < m_LeavesA.Length; la++)
                {
                    for (int lb = 0; lb < m_LeavesB.Length; lb++)
                    {
                        var combined = Material.FlagsToCollisionResponse(m_LeavesA[la].Flags | m_LeavesB[lb].Flags);
                        if (combined == CollisionResponsePolicy.None) continue;
                        bool raising = combined == CollisionResponsePolicy.RaiseTriggerEvents
                            || combined == CollisionResponsePolicy.CollideRaiseCollisionEvents;
                        if (bothKinematic && !raising) continue;

                        if (!CollisionTests.TestLeaves(m_LeavesA[la], m_LeavesB[lb], out var normal, out var depth))
                        {
                            if (step.SpeculativeMargin > 0f && !bothKinematic && combined != CollisionResponsePolicy.RaiseTriggerEvents)
                            {
                                var gap = CollisionTests.LeafPairDistance(m_LeavesA[la], m_LeavesB[lb], out var gapNormal);
                                if (gap <= step.SpeculativeMargin)
                                {
                                    m_Speculative.Add(new SpeculativeContact
                                    {
                                        A = i,
                                        B = j,
                                        Normal = -gapNormal,
                                        Distance = gap,
                                        RaisesEvents = combined == CollisionResponsePolicy.CollideRaiseCollisionEvents
                                    });
                                }
                            }
                            continue;
                        }

                        int indexA = j >= numDynamic ? j : i;
                        int indexB = j >= numDynamic ? i : j;
                        if (combined == CollisionResponsePolicy.RaiseTriggerEvents)
                        {
                            simulation.TriggerEvents.Add(new TriggerEventData
                            {
                                EntityA = bodies[indexA].Entity,
                                EntityB = bodies[indexB].Entity,
                                BodyIndexA = indexA,
                                BodyIndexB = indexB
                            });
                        }
                        else
                        {
                            if (combined == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                            {
                                simulation.CollisionEvents.Add(new CollisionEventData
                                {
                                    EntityA = bodies[indexA].Entity,
                                    EntityB = bodies[indexB].Entity,
                                    BodyIndexA = indexA,
                                    BodyIndexB = indexB,
                                    Normal = normal
                                });
                            }
                            if (!bothKinematic)
                            {
                                m_Contacts.Add(new Contact { A = i, B = j, Normal = normal, Depth = depth });
                                if (step.SpeculativeMargin > 0f)
                                {
                                    m_Speculative.Add(new SpeculativeContact { A = i, B = j, Normal = normal, Distance = -depth, RaisesEvents = false });
                                }
                            }
                        }
                    }
                }
            }

            bool hasContacts = m_Contacts.Length > 0;
            m_Positions.Clear();
            if (hasContacts)
            {
                for (int b = 0; b < numBodies; b++)
                {
                    m_Positions.Add(bodies[b].WorldFromBody.pos);
                }
                for (int c = 0; c < m_Contacts.Length; c++)
                {
                    var contact = m_Contacts[c];
                    float invA = kinematics[contact.A] != 0 ? 0f : invMasses[contact.A];
                    float invB = kinematics[contact.B] != 0 ? 0f : invMasses[contact.B];
                    float sum = invA + invB;
                    if (sum <= 0f) continue;
                    bool staticPair = kinematics[contact.A] != 0 || kinematics[contact.B] != 0;
                    float cap = staticPair ? float.MaxValue : step.MaxDynamicDepenetrationVelocity * dt;
                    float correction = math.min(contact.Depth, cap);
                    m_Positions[contact.A] += contact.Normal * (correction * invA / sum);
                    m_Positions[contact.B] -= contact.Normal * (correction * invB / sum);
                }
            }

            bool hasSpeculative = m_Speculative.Length > 0;
            var linearVelocities = collisionWorld.m_LinearVelocities;
            if (hasSpeculative)
            {
                for (int c = 0; c < m_Speculative.Length; c++)
                {
                    var contact = m_Speculative[c];
                    float invA = kinematics[contact.A] != 0 ? 0f : invMasses[contact.A];
                    float invB = kinematics[contact.B] != 0 ? 0f : invMasses[contact.B];
                    float sum = invA + invB;
                    if (sum <= 0f) continue;
                    float gapRate = math.dot(linearVelocities[contact.A] - linearVelocities[contact.B], contact.Normal);
                    float endGap = contact.Distance + gapRate * dt;
                    if (endGap >= 0f) continue;
                    float impulse = -endGap / (dt * sum);
                    if (impulse * sum > step.MaxDynamicDepenetrationVelocity) impulse = step.MaxDynamicDepenetrationVelocity / sum;
                    linearVelocities[contact.A] += contact.Normal * (impulse * invA);
                    linearVelocities[contact.B] -= contact.Normal * (impulse * invB);
                    if (contact.RaisesEvents)
                    {
                        int indexA = contact.B >= numDynamic ? contact.B : contact.A;
                        int indexB = contact.B >= numDynamic ? contact.A : contact.B;
                        simulation.CollisionEvents.Add(new CollisionEventData
                        {
                            EntityA = bodies[indexA].Entity,
                            EntityB = bodies[indexB].Entity,
                            BodyIndexA = indexA,
                            BodyIndexB = indexB,
                            Normal = contact.Normal
                        });
                    }
                }
            }

            var dampingLinear = collisionWorld.m_DampingLinear;
            var dampingAngular = collisionWorld.m_DampingAngular;
            var transformHandle = SystemAPI.GetComponentTypeHandle<LocalTransform>();
            var velocityHandle = SystemAPI.GetComponentTypeHandle<PhysicsVelocity>();
            var massHandle = SystemAPI.GetComponentTypeHandle<PhysicsMass>(true);
            int bodyIndex = 0;
            foreach (var chunk in SystemAPI.QueryBuilder().WithAll<PhysicsCollider, LocalTransform, PhysicsVelocity>().Build().ToArchetypeChunkArray(Allocator.Temp))
            {
                var transforms = chunk.GetNativeArray(ref transformHandle);
                var velocities = chunk.GetNativeArray(ref velocityHandle);
                var masses = chunk.GetNativeArray(ref massHandle);
                bool hasMass = masses.IsCreated;
                for (int i = 0; i < chunk.Count; i++)
                {
                    int bi = bodyIndex++;

                    var v = velocities[i];
                    if (hasSpeculative) v.Linear = linearVelocities[bi];

                    var tr = transforms[i];
                    float3 position = hasContacts ? m_Positions[bi] : tr.Position;
                    position += v.Linear * dt;

                    var rotation = tr.Rotation;
                    if (math.any(v.Angular))
                    {
                        var mass = hasMass ? masses[i] : PhysicsMass.CreateKinematic(MassProperties.UnitSphere);
                        var worldFromMotion = math.mul(rotation, mass.Transform.rot);
                        var angularWorld = math.rotate(worldFromMotion, v.Angular);
                        var angle = math.length(angularWorld) * dt;
                        if (angle > 1e-8f)
                        {
                            rotation = math.normalize(math.mul(
                                quaternion.AxisAngle(angularWorld / math.length(angularWorld), angle), rotation));
                        }
                    }

                    v.Linear *= math.clamp(1f - dampingLinear[bi] * dt, 0f, 1f);
                    v.Angular *= math.clamp(1f - dampingAngular[bi] * dt, 0f, 1f);

                    tr.Position = position;
                    tr.Rotation = rotation;
                    transforms[i] = tr;
                    velocities[i] = v;
                }
            }
        }
    }
}
