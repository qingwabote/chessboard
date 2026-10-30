using Bastard;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    [UpdateInGroup(typeof(PhysicsInitializeGroup), OrderLast = true)]
    public partial struct BuildPhysicsWorld : ISystem
    {
        const float k_CellSize = 1.5f;

        NativeArray<RigidBody> m_Bodies;
        NativeArray<Aabb2> m_Eabbs;
        NativeArray<CollisionFilter> m_Filters;
        NativeArray<byte> m_MaterialFlags;
        NativeArray<int> m_CellBodyIndices;
        NativeArray<int> m_BodyIndices;
        NativeArray<byte> m_Kinematics;
        NativeArray<float> m_InvMasses;
        NativeArray<float3> m_LinearVelocities;
        NativeArray<float> m_DampingLinear;
        NativeArray<float> m_DampingAngular;

        public void OnCreate(ref SystemState state)
        {
            var persistent = Allocator.Persistent;
            m_Bodies = new NativeArray<RigidBody>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_Eabbs = new NativeArray<Aabb2>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_Filters = new NativeArray<CollisionFilter>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_MaterialFlags = new NativeArray<byte>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_CellBodyIndices = new NativeArray<int>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_BodyIndices = new NativeArray<int>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_Kinematics = new NativeArray<byte>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_InvMasses = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_LinearVelocities = new NativeArray<float3>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_DampingLinear = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);
            m_DampingAngular = new NativeArray<float>(0, persistent, NativeArrayOptions.UninitializedMemory);

            var entity = state.EntityManager.CreateEntity(typeof(PhysicsWorldSingleton));
            state.EntityManager.SetName(entity, "PhysicsWorldSingleton");
            state.EntityManager.SetComponentData(entity, new PhysicsWorldSingleton
            {
                PhysicsWorld = new PhysicsWorld
                {
                    CollisionWorld = new CollisionWorld(
                        m_Bodies, m_Eabbs, m_Filters, m_MaterialFlags, m_CellBodyIndices, m_BodyIndices,
                        0, default, default, k_CellSize),
                    DynamicsWorld = new DynamicsWorld(
                        m_Kinematics, m_InvMasses, m_LinearVelocities, m_DampingLinear, m_DampingAngular)
                }
            });
        }

        [BurstCompile]
        unsafe public void OnUpdate(ref SystemState state)
        {
            var step = SystemAPI.TryGetSingleton(out PhysicsStep physicsStep) ? physicsStep : PhysicsStep.Default;
            float dt = SystemAPI.Time.DeltaTime;

            int bodyCount = SystemAPI.QueryBuilder().WithAll<PhysicsCollider, LocalTransform>().Build().CalculateEntityCount();

            const float k_GroundMargin = 24f;
            CameraExtensions.GetGroundAABB(out var groundMin, out var groundMax);
            var gridMin = (int2)math.floor((groundMin - k_GroundMargin) / k_CellSize);
            var gridMax = (int2)math.floor((groundMax + k_GroundMargin) / k_CellSize);
            var gridSize = gridMax - gridMin + 1;
            var cellTotal = gridSize.x * gridSize.y;

            static ref NativeArray<T> Grow<T>(ref NativeArray<T> array, int capacity) where T : unmanaged
            {
                if (array.Length >= capacity) return ref array;
                array.Dispose();
                array = new NativeArray<T>(capacity, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                return ref array;
            }

            var bodies = Grow(ref m_Bodies, bodyCount);
            var eabbs = Grow(ref m_Eabbs, bodyCount);
            var filters = Grow(ref m_Filters, bodyCount);
            var flags = Grow(ref m_MaterialFlags, bodyCount);
            var kinematics = Grow(ref m_Kinematics, bodyCount);
            var invMasses = Grow(ref m_InvMasses, bodyCount);
            var linearVelocities = Grow(ref m_LinearVelocities, bodyCount);
            var dampingLinear = Grow(ref m_DampingLinear, bodyCount);
            var dampingAngular = Grow(ref m_DampingAngular, bodyCount);

            int bodyIndex = 0;
            int numDynamicBodies = 0;
            int CellIndex(int2 cell)
            {
                return (cell.x - gridMin.x) + (cell.y - gridMin.y) * gridSize.x;
            }
            var occupancy = new NativeArray<int>(cellTotal, Allocator.Temp, NativeArrayOptions.ClearMemory);
            var occupiedMin = new NativeArray<int2>(bodyCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var occupiedMax = new NativeArray<int2>(bodyCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            void Occupy(float2 min, float2 max)
            {
                var cellMin = math.max((int2)math.floor(min / k_CellSize), gridMin);
                var cellMax = math.min((int2)math.floor(max / k_CellSize), gridMax);
                for (int x = cellMin.x; x <= cellMax.x; x++)
                {
                    for (int y = cellMin.y; y <= cellMax.y; y++)
                    {
                        occupancy[CellIndex(new int2(x, y))]++;
                    }
                }
                occupiedMin[bodyIndex] = cellMin;
                occupiedMax[bodyIndex] = cellMax;
            }

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
                        Aabb2 aabb = collider.Value.CalculateAabb(bodyTransform, scale);

                        var mass = hasMass ? masses[i] : PhysicsMass.CreateKinematic(MassProperties.UnitSphere);

                        var uniform = math.min(
                            math.length(velocities[i].Angular) * dt * mass.AngularExpansionFactor,
                            mass.AngularExpansionFactor);
                        var margin = uniform + step.CollisionTolerance * 0.5f + step.SpeculativeMargin;
                        Aabb2 eabb = new Aabb2
                        {
                            Min = math.min(aabb.Min, aabb.Min + velocities[i].Linear.xz * dt) - margin,
                            Max = math.max(aabb.Max, aabb.Max + velocities[i].Linear.xz * dt) + margin
                        };

                        var blob = (Collider*)collider.Value.GetUnsafePtr();
                        bodies[bodyIndex] = new RigidBody
                        {
                            Collider = collider.Value,
                            WorldFromBody = bodyTransform,
                            Entity = entities[i],
                            Scale = scale
                        };
                        eabbs[bodyIndex] = eabb;
                        filters[bodyIndex] = blob->m_Header.Filter;
                        flags[bodyIndex] = (byte)blob->m_Header.Material.Flags;
                        kinematics[bodyIndex] = mass.IsKinematic ? (byte)1 : (byte)0;
                        invMasses[bodyIndex] = mass.InverseMass;
                        linearVelocities[bodyIndex] = velocities[i].Linear;
                        if (hasDamping)
                        {
                            dampingLinear[bodyIndex] = dampings[i].Linear;
                            dampingAngular[bodyIndex] = dampings[i].Angular;
                        }
                        else
                        {
                            dampingLinear[bodyIndex] = 0f;
                            dampingAngular[bodyIndex] = 0f;
                        }
                        Occupy(eabb.Min, eabb.Max);
                        bodyIndex++;
                    }
                }
                numDynamicBodies = bodyIndex;
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
                        Aabb2 aabb = collider.Value.CalculateAabb(bodyTransform, scale);
                        aabb.Expand(step.CollisionTolerance * 0.5f);

                        var blob = (Collider*)collider.Value.GetUnsafePtr();
                        bodies[bodyIndex] = new RigidBody
                        {
                            Collider = collider.Value,
                            WorldFromBody = bodyTransform,
                            Entity = entities[i],
                            Scale = scale
                        };
                        eabbs[bodyIndex] = aabb;
                        filters[bodyIndex] = blob->m_Header.Filter;
                        flags[bodyIndex] = (byte)blob->m_Header.Material.Flags;
                        kinematics[bodyIndex] = 1;
                        invMasses[bodyIndex] = 0f;
                        linearVelocities[bodyIndex] = float3.zero;
                        dampingLinear[bodyIndex] = 0f;
                        dampingAngular[bodyIndex] = 0f;
                        Occupy(aabb.Min, aabb.Max);
                        bodyIndex++;
                    }
                }
            }

            var cellBodyIndices = occupancy;
            int cellBodyIndex = 0;
            for (int i = 0; i < cellTotal; i++)
            {
                int count = cellBodyIndices[i];
                cellBodyIndices[i] = cellBodyIndex;
                cellBodyIndex += count;
            }

            Grow(ref m_CellBodyIndices, cellTotal + 1);
            m_CellBodyIndices[cellTotal] = cellBodyIndex;
            NativeArray<int>.Copy(cellBodyIndices, m_CellBodyIndices, cellTotal);

            var bodyIndices = Grow(ref m_BodyIndices, cellBodyIndex);
            for (int i = 0; i < bodyIndex; i++)
            {
                var min = occupiedMin[i];
                var max = occupiedMax[i];
                for (int x = min.x; x <= max.x; x++)
                {
                    for (int y = min.y; y <= max.y; y++)
                    {
                        bodyIndices[cellBodyIndices[CellIndex(new int2(x, y))]++] = i;
                    }
                }
            }

            ref var physicsWorld = ref SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
            physicsWorld.CollisionWorld = new CollisionWorld(
                bodies.GetSubArray(0, bodyIndex), eabbs.GetSubArray(0, bodyIndex),
                filters.GetSubArray(0, bodyIndex), flags.GetSubArray(0, bodyIndex),
                m_CellBodyIndices.GetSubArray(0, cellTotal + 1), bodyIndices.GetSubArray(0, cellBodyIndex),
                numDynamicBodies, gridMin, gridMax, k_CellSize);
            physicsWorld.DynamicsWorld = new DynamicsWorld(
                kinematics.GetSubArray(0, bodyIndex), invMasses.GetSubArray(0, bodyIndex),
                linearVelocities.GetSubArray(0, bodyIndex), dampingLinear.GetSubArray(0, bodyIndex),
                dampingAngular.GetSubArray(0, bodyIndex));
        }

        public void OnDestroy(ref SystemState state)
        {
            if (m_Bodies.IsCreated) m_Bodies.Dispose();
            if (m_Eabbs.IsCreated) m_Eabbs.Dispose();
            if (m_Filters.IsCreated) m_Filters.Dispose();
            if (m_MaterialFlags.IsCreated) m_MaterialFlags.Dispose();
            if (m_CellBodyIndices.IsCreated) m_CellBodyIndices.Dispose();
            if (m_BodyIndices.IsCreated) m_BodyIndices.Dispose();
            if (m_Kinematics.IsCreated) m_Kinematics.Dispose();
            if (m_InvMasses.IsCreated) m_InvMasses.Dispose();
            if (m_LinearVelocities.IsCreated) m_LinearVelocities.Dispose();
            if (m_DampingLinear.IsCreated) m_DampingLinear.Dispose();
            if (m_DampingAngular.IsCreated) m_DampingAngular.Dispose();
        }

        // Physics internally represents all rigid bodies in world space. If a static body is in a
        // hierarchy, its local-to-world matrix must be decomposed when building the physics world. This
        // method returns a world-space RigidTransform that would be decomposed for such a rigid body.
        // Copy from Unity.Physics 1.4.5 (com.unity.physics@0dfa150f2a43), Unity.Physics/Base/Math/Math.cs:578-583.
        static RigidTransform DecomposeRigidBodyTransform(in float4x4 localToWorld) =>
            new(quaternion.LookRotationSafe(localToWorld.c2.xyz, localToWorld.c1.xyz), localToWorld.c3.xyz);
    }
}
