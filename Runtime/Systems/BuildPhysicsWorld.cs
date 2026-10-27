using Bastard;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    [UpdateInGroup(typeof(PhysicsInitializeGroup), OrderLast = true)]
    public partial struct BuildPhysicsWorld : ISystem
    {
        const float k_CellSize = 1.5f;

        NativeArray<RigidBody> m_AllBodies;
        NativeArray<Aabb> m_AllEabbs;
        NativeArray<CollisionFilter> m_AllFilters;
        NativeArray<byte> m_AllFlags;
        NativeArray<byte> m_AllKinematics;
        NativeArray<float> m_AllInvMasses;
        NativeArray<float3> m_AllLinearVelocities;
        NativeArray<float> m_AllDampingLinear;
        NativeArray<float> m_AllDampingAngular;
        NativeArray<int> m_CellStart;
        NativeArray<int> m_CellCursor;
        NativeArray<int> m_CellOrder;
        NativeArray<int2> m_CellSpanMin;
        NativeArray<int2> m_CellSpanMax;
        internal int2 m_GridMinCell;
        internal int2 m_GridCount;


        public void OnCreate(ref SystemState state)
        {
            var persistent = Allocator.Persistent;
            m_AllBodies = new NativeArray<RigidBody>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllEabbs = new NativeArray<Aabb>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllFilters = new NativeArray<CollisionFilter>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllFlags = new NativeArray<byte>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllKinematics = new NativeArray<byte>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllInvMasses = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllLinearVelocities = new NativeArray<float3>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllDampingLinear = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_AllDampingAngular = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellStart = new NativeArray<int>(1024, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellCursor = new NativeArray<int>(1024, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellOrder = new NativeArray<int>(256, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellSpanMin = new NativeArray<int2>(256, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellSpanMax = new NativeArray<int2>(256, persistent, NativeArrayOptions.UninitializedMemory);
            m_GridMinCell = int2.zero;
            m_GridCount = int2.zero;

            var entity = state.EntityManager.CreateEntity(typeof(PhysicsWorldSingleton));
            state.EntityManager.SetName(entity, "PhysicsWorldSingleton");
            SystemAPI.SetSingleton(new PhysicsWorldSingleton
            {
                PhysicsWorld = new PhysicsWorld
                {
                    CollisionWorld = new CollisionWorld
                    {
                        CollisionTolerance = CollisionWorld.DefaultCollisionTolerance,
                        m_CellStart = m_CellStart,
                        m_CellOrder = m_CellOrder,
                        m_GridMinCell = int2.zero,
                        m_GridCount = int2.zero,
                        m_CellSize = k_CellSize
                    }
                }
            });
        }

        public void OnDestroy(ref SystemState state)
        {
            if (m_AllBodies.IsCreated) m_AllBodies.Dispose();
            if (m_AllEabbs.IsCreated) m_AllEabbs.Dispose();
            if (m_AllFilters.IsCreated) m_AllFilters.Dispose();
            if (m_AllFlags.IsCreated) m_AllFlags.Dispose();
            if (m_AllKinematics.IsCreated) m_AllKinematics.Dispose();
            if (m_AllInvMasses.IsCreated) m_AllInvMasses.Dispose();
            if (m_AllLinearVelocities.IsCreated) m_AllLinearVelocities.Dispose();
            if (m_AllDampingLinear.IsCreated) m_AllDampingLinear.Dispose();
            if (m_AllDampingAngular.IsCreated) m_AllDampingAngular.Dispose();
            if (m_CellStart.IsCreated) m_CellStart.Dispose();
            if (m_CellCursor.IsCreated) m_CellCursor.Dispose();
            if (m_CellOrder.IsCreated) m_CellOrder.Dispose();
            if (m_CellSpanMin.IsCreated) m_CellSpanMin.Dispose();
            if (m_CellSpanMax.IsCreated) m_CellSpanMax.Dispose();
        }

        [BurstCompile]
        unsafe public void OnUpdate(ref SystemState state)
        {
            var step = SystemAPI.TryGetSingleton(out PhysicsStep physicsStep) ? physicsStep : PhysicsStep.Default;
            float dt = SystemAPI.Time.DeltaTime;

            int totalCount = SystemAPI.QueryBuilder()
                .WithAll<PhysicsCollider, LocalTransform>()
                .Build().CalculateEntityCount();
            if (totalCount == 0)
            {
                return;
            }

            EnsureCapacity(ref m_AllBodies, totalCount);
            EnsureCapacity(ref m_AllEabbs, totalCount);
            EnsureCapacity(ref m_AllFilters, totalCount);
            EnsureCapacity(ref m_AllFlags, totalCount);
            EnsureCapacity(ref m_AllKinematics, totalCount);
            EnsureCapacity(ref m_AllInvMasses, totalCount);
            EnsureCapacity(ref m_AllLinearVelocities, totalCount);
            EnsureCapacity(ref m_AllDampingLinear, totalCount);
            EnsureCapacity(ref m_AllDampingAngular, totalCount);
            if (m_CellSpanMin.Length < totalCount)
            {
                m_CellSpanMin.Dispose();
                m_CellSpanMax.Dispose();
                m_CellSpanMin = new NativeArray<int2>(math.ceilpow2(math.max(256, totalCount)), Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                m_CellSpanMax = new NativeArray<int2>(m_CellSpanMin.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            var bodies = m_AllBodies;
            var eabbs = m_AllEabbs;
            var filters = m_AllFilters;
            var flags = m_AllFlags;
            var kinematics = m_AllKinematics;
            var invMasses = m_AllInvMasses;
            var linearVelocities = m_AllLinearVelocities;
            var dampingLinear = m_AllDampingLinear;
            var dampingAngular = m_AllDampingAngular;

            const float k_WindowMargin = 24f;
            CameraExtensions.GetGroundAABB(out var windowMin, out var windowMax);
            windowMin -= k_WindowMargin;
            windowMax += k_WindowMargin;
            var cellMin = (int2)math.floor(windowMin / k_CellSize);
            var cellMax = (int2)math.floor(windowMax / k_CellSize);

            int index = 0;
            int dynamicTotal = 0;
            var colliderHandle = SystemAPI.GetComponentTypeHandle<PhysicsCollider>(true);
            var transformHandle = SystemAPI.GetComponentTypeHandle<LocalTransform>(true);
            var entityHandle = SystemAPI.GetEntityTypeHandle();
            var parentHandle = SystemAPI.GetComponentTypeHandle<Parent>(true);
            var localToWorldHandle = SystemAPI.GetComponentTypeHandle<LocalToWorld>(true);
            {
                var velocityHandle = SystemAPI.GetComponentTypeHandle<PhysicsVelocity>(true);
                var massHandle = SystemAPI.GetComponentTypeHandle<PhysicsMass>(true);
                var dampingHandle = SystemAPI.GetComponentTypeHandle<PhysicsDamping>(true);
                foreach (var chunk in SystemAPI.QueryBuilder().WithAll<PhysicsCollider, LocalTransform, PhysicsVelocity>().Build().ToArchetypeChunkArray(Allocator.Temp))
                {
                    var colliders = chunk.GetNativeArray(ref colliderHandle);
                    var transforms = chunk.GetNativeArray(ref transformHandle);
                    var velocities = chunk.GetNativeArray(ref velocityHandle);
                    var entities = chunk.GetNativeArray(entityHandle);
                    var masses = chunk.GetNativeArray(ref massHandle);
                    var dampings = chunk.GetNativeArray(ref dampingHandle);
                    var worlds = chunk.GetNativeArray(ref localToWorldHandle);
                    bool hasLocal = transforms.IsCreated;
                    bool hasWorld = worlds.IsCreated;
                    bool hasMass = masses.IsCreated;
                    bool hasDamping = dampings.IsCreated;
                    bool hasParent = chunk.Has(ref parentHandle);
                    for (int i = 0; i < chunk.Count; i++)
                    {
                        var collider = colliders[i];
                        if (!collider.IsValid) continue;
                        // We support rigid body entities with various different transformation data components.
                        // Here, we are extracting their world space transformation and feed it into the underlying
                        // physics engine for processing in the pipeline (e.g., collision detection, solver, integration).
                        // If the rigid body has a Parent, we obtain their world space transformation from their up-to date
                        // LocalToWorld matrix. Any shear or scale in this matrix is ignored and only the rigid body transformation,
                        // i.e., the position and orientation, is extracted.
                        // If the rigid body has no Parent, we use the position and orientation of its LocalTransform component as world transform
                        // if present. Otherwise, we again extract the transformation from the LocalToWorld matrix.
                        // Copy from Unity.Physics 1.4.5 (com.unity.physics@0dfa150f2a43), PhysicsWorldBuilder.cs:680-697.
                        RigidTransform bodyTransform = RigidTransform.identity;
                        if (hasParent || !hasLocal)
                        {
                            if (hasWorld)
                            {
                                bodyTransform = DecomposeRigidBodyTransform(worlds[i].Value);
                            }
                        }
                        else
                        {
                            bodyTransform.pos = transforms[i].Position;
                            bodyTransform.rot = transforms[i].Rotation;
                        }
                        float scale = hasLocal ? transforms[i].Scale : 1f;
                        var aabb = collider.Value.CalculateAabb(bodyTransform, scale);

                        var mass = hasMass ? masses[i] : PhysicsMass.CreateKinematic(MassProperties.UnitSphere);

                        var uniform = math.min(
                            math.length(velocities[i].Angular) * dt * mass.AngularExpansionFactor,
                            mass.AngularExpansionFactor);
                        var margin = uniform + step.CollisionTolerance * 0.5f + step.SpeculativeMargin;
                        var eabb = new Aabb
                        {
                            Min = math.min(aabb.Min, aabb.Min + velocities[i].Linear * dt) - margin,
                            Max = math.max(aabb.Max, aabb.Max + velocities[i].Linear * dt) + margin
                        };

                        var blob = (Collider*)collider.Value.GetUnsafePtr();
                        bodies[index] = new RigidBody
                        {
                            Collider = collider.Value,
                            WorldFromBody = bodyTransform,
                            Entity = entities[i],
                            Scale = scale
                        };
                        eabbs[index] = eabb;
                        filters[index] = blob->m_Header.Filter;
                        flags[index] = (byte)blob->m_Header.Material.Flags;
                        kinematics[index] = mass.IsKinematic ? (byte)1 : (byte)0;
                        invMasses[index] = mass.InverseMass;
                        linearVelocities[index] = velocities[i].Linear;
                        if (hasDamping)
                        {
                            dampingLinear[index] = dampings[i].Linear;
                            dampingAngular[index] = dampings[i].Angular;
                        }
                        else
                        {
                            dampingLinear[index] = 0f;
                            dampingAngular[index] = 0f;
                        }
                        var dmin = (int2)math.floor(eabb.Min.xz / k_CellSize);
                        var dmax = (int2)math.floor(eabb.Max.xz / k_CellSize);
                        m_CellSpanMin[index] = dmin;
                        m_CellSpanMax[index] = dmax;
                        index++;
                    }
                }
                dynamicTotal = index;
            }

            {
                foreach (var chunk in SystemAPI.QueryBuilder().WithAll<PhysicsCollider, LocalTransform>().WithNone<PhysicsVelocity>().Build().ToArchetypeChunkArray(Allocator.Temp))
                {
                    var colliders = chunk.GetNativeArray(ref colliderHandle);
                    var transforms = chunk.GetNativeArray(ref transformHandle);
                    var entities = chunk.GetNativeArray(entityHandle);
                    var worlds = chunk.GetNativeArray(ref localToWorldHandle);
                    bool hasLocal = transforms.IsCreated;
                    bool hasWorld = worlds.IsCreated;
                    bool hasParent = chunk.Has(ref parentHandle);
                    for (int i = 0; i < chunk.Count; i++)
                    {
                        var collider = colliders[i];
                        if (!collider.IsValid) continue;
                        // We support rigid body entities with various different transformation data components.
                        // Here, we are extracting their world space transformation and feed it into the underlying
                        // physics engine for processing in the pipeline (e.g., collision detection, solver, integration).
                        // If the rigid body has a Parent, we obtain their world space transformation from their up-to date
                        // LocalToWorld matrix. Any shear or scale in this matrix is ignored and only the rigid body transformation,
                        // i.e., the position and orientation, is extracted.
                        // If the rigid body has no Parent, we use the position and orientation of its LocalTransform component as world transform
                        // if present. Otherwise, we again extract the transformation from the LocalToWorld matrix.
                        // Copy from Unity.Physics 1.4.5 (com.unity.physics@0dfa150f2a43), PhysicsWorldBuilder.cs:680-697.
                        RigidTransform bodyTransform = RigidTransform.identity;
                        if (hasParent || !hasLocal)
                        {
                            if (hasWorld)
                            {
                                bodyTransform = DecomposeRigidBodyTransform(worlds[i].Value);
                            }
                        }
                        else
                        {
                            bodyTransform.pos = transforms[i].Position;
                            bodyTransform.rot = transforms[i].Rotation;
                        }
                        float scale = hasLocal ? transforms[i].Scale : 1f;
                        var aabb = collider.Value.CalculateAabb(bodyTransform, scale);
                        aabb.Expand(step.CollisionTolerance * 0.5f);

                        var blob = (Collider*)collider.Value.GetUnsafePtr();
                        bodies[index] = new RigidBody
                        {
                            Collider = collider.Value,
                            WorldFromBody = bodyTransform,
                            Entity = entities[i],
                            Scale = scale
                        };
                        eabbs[index] = aabb;
                        filters[index] = blob->m_Header.Filter;
                        flags[index] = (byte)blob->m_Header.Material.Flags;
                        kinematics[index] = 1;
                        invMasses[index] = 0f;
                        linearVelocities[index] = float3.zero;
                        dampingLinear[index] = 0f;
                        dampingAngular[index] = 0f;
                        var smin = (int2)math.floor(aabb.Min.xz / k_CellSize);
                        var smax = (int2)math.floor(aabb.Max.xz / k_CellSize);
                        m_CellSpanMin[index] = smin;
                        m_CellSpanMax[index] = smax;
                        index++;
                    }
                }
            }
            int validTotal = index;

            var gridCount = cellMax - cellMin + 1;
            int cellTotal = gridCount.x * gridCount.y;

            if (m_CellStart.Length < cellTotal + 1)
            {
                m_CellStart.Dispose();
                m_CellStart = new NativeArray<int>(math.ceilpow2(cellTotal + 1), Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                m_CellCursor.Dispose();
                m_CellCursor = new NativeArray<int>(m_CellStart.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
            if (m_CellOrder.Length < index)
            {
                m_CellOrder.Dispose();
                m_CellOrder = new NativeArray<int>(math.ceilpow2(math.max(256, index)), Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
            UnsafeUtility.MemClear(m_CellStart.GetUnsafePtr(), (cellTotal + 1) * sizeof(int));

            int CellIndex(int2 cell)
            {
                return (cell.x - cellMin.x) + (cell.y - cellMin.y) * gridCount.x;
            }

            for (int i = 0; i < validTotal; i++)
            {
                var cmin = math.max(m_CellSpanMin[i], cellMin);
                var cmax = math.min(m_CellSpanMax[i], cellMax);
                if (cmin.x > cmax.x || cmin.y > cmax.y)
                {
                    m_CellSpanMin[i] = new int2(1);
                    m_CellSpanMax[i] = new int2(0);
                    continue;
                }
                m_CellSpanMin[i] = cmin;
                m_CellSpanMax[i] = cmax;
                for (int x = cmin.x; x <= cmax.x; x++)
                {
                    for (int z = cmin.y; z <= cmax.y; z++)
                    {
                        m_CellStart[CellIndex(new int2(x, z)) + 1]++;
                    }
                }
            }
            for (int c = 0; c < cellTotal; c++)
            {
                m_CellStart[c + 1] += m_CellStart[c];
                m_CellCursor[c] = m_CellStart[c];
            }
            EnsureCapacity(ref m_CellOrder, m_CellStart[cellTotal]);
            for (int i = 0; i < validTotal; i++)
            {
                var cmin = m_CellSpanMin[i];
                var cmax = m_CellSpanMax[i];
                for (int x = cmin.x; x <= cmax.x; x++)
                {
                    for (int z = cmin.y; z <= cmax.y; z++)
                    {
                        m_CellOrder[m_CellCursor[CellIndex(new int2(x, z))]++] = i;
                    }
                }
            }

            SystemAPI.SetSingleton(new PhysicsWorldSingleton
            {
                PhysicsWorld = new PhysicsWorld
                {
                    CollisionWorld = new CollisionWorld
                    {
                        CollisionTolerance = CollisionWorld.DefaultCollisionTolerance,
                        m_Bodies = m_AllBodies.GetSubArray(0, validTotal),
                        m_NumDynamicBodies = dynamicTotal,
                        m_Eabbs = m_AllEabbs.GetSubArray(0, validTotal),
                        m_Filters = m_AllFilters.GetSubArray(0, validTotal),
                        m_MaterialFlags = m_AllFlags.GetSubArray(0, validTotal),
                        m_Kinematics = m_AllKinematics.GetSubArray(0, validTotal),
                        m_InvMasses = m_AllInvMasses.GetSubArray(0, validTotal),
                        m_LinearVelocities = m_AllLinearVelocities.GetSubArray(0, validTotal),
                        m_DampingLinear = m_AllDampingLinear.GetSubArray(0, validTotal),
                        m_DampingAngular = m_AllDampingAngular.GetSubArray(0, validTotal),
                        m_CellStart = m_CellStart,
                        m_CellOrder = m_CellOrder,
                        m_GridMinCell = cellMin,
                        m_GridCount = gridCount,
                        m_CellSize = k_CellSize
                    }
                }
            });
        }


        // Physics internally represents all rigid bodies in world space. If a static body is in a
        // hierarchy, its local-to-world matrix must be decomposed when building the physics world. This
        // method returns a world-space RigidTransform that would be decomposed for such a rigid body.
        // Copy from Unity.Physics 1.4.5 (com.unity.physics@0dfa150f2a43), Unity.Physics/Base/Math/Math.cs:578-583.
        static RigidTransform DecomposeRigidBodyTransform(in float4x4 localToWorld) =>
            new(quaternion.LookRotationSafe(localToWorld.c2.xyz, localToWorld.c1.xyz), localToWorld.c3.xyz);

        static void EnsureCapacity<T>(ref NativeArray<T> array, int capacity) where T : unmanaged
        {
            if (array.Length < capacity)
            {
                array.Dispose();
                array = new NativeArray<T>(
                    math.max(256, math.ceilpow2(capacity)), Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
        }
    }
}
