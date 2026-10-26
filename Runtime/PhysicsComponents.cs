using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct PhysicsCollider : IComponentData
    {
        public BlobAssetReference<Collider> Value;

        public bool IsValid => Value.IsCreated;

        public unsafe Collider* ColliderPtr => (Collider*)Value.GetUnsafePtr();

        public MassProperties MassProperties => Value.IsCreated ? Value.Value.MassProperties : MassProperties.UnitSphere;

        public bool IsUnique => Value.IsCreated && Value.Value.IsUnique;
    }

    public struct PhysicsMass : IComponentData
    {
        public RigidTransform Transform;
        public float InverseMass;
        public float3 InverseInertia;
        public float AngularExpansionFactor;

        public float3 CenterOfMass
        {
            get => Transform.pos;
            set => Transform.pos = value;
        }

        public quaternion InertiaOrientation
        {
            get => Transform.rot;
            set => Transform.rot = value;
        }

        public bool HasInfiniteMass => InverseMass == 0.0f;
        public bool HasInfiniteInertia => !math.any(InverseInertia);
        public bool IsKinematic => HasInfiniteMass && HasInfiniteInertia;

        public static PhysicsMass CreateDynamic(MassProperties massProperties, float mass)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(math.max(math.abs(mass), float.MinValue)),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = 0f,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }
    }

    public struct PhysicsVelocity : IComponentData
    {
        public float3 Linear;
        public float3 Angular;

        public static readonly PhysicsVelocity Zero = new PhysicsVelocity
        {
            Linear = float3.zero,
            Angular = float3.zero
        };
    }

    public struct PhysicsDamping : IComponentData
    {
        public float Linear;
        public float Angular;
    }

    public struct PhysicsGravityFactor : IComponentData
    {
        public float Value;
    }

    public struct EnsureUniqueColliderBlobTag : IComponentData
    {
    }

    public struct ColliderBlobCleanupData : ICleanupComponentData
    {
        public BlobAssetReference<Collider> Value;
    }

    public enum SimulationType : byte
    {
        NoPhysics = 0,
        UnityPhysics = 1
    }

    public struct PhysicsStep : IComponentData
    {
        public SimulationType SimulationType;
        public float3 Gravity;
        public int SubstepCount;
        public int SolverIterationCount;
        public byte MultiThreaded;
        public float CollisionTolerance;
        public byte SynchronizeCollisionWorld;
        public float MaxDynamicDepenetrationVelocity;
        public float MaxStaticDepenetrationVelocity;
        /// <summary>Speculative contact distance: pairs closer than this produce contacts whose velocity constraints prevent predicted penetration this step.</summary>
        public float SpeculativeMargin;

        public static readonly PhysicsStep Default = new PhysicsStep
        {
            SimulationType = SimulationType.UnityPhysics,
            Gravity = new float3(0f, -9.81f, 0f),
            SubstepCount = 1,
            SolverIterationCount = 4,
            MultiThreaded = 1,
            CollisionTolerance = CollisionWorld.DefaultCollisionTolerance,
            SynchronizeCollisionWorld = 0,
            MaxDynamicDepenetrationVelocity = 3f,
            MaxStaticDepenetrationVelocity = float.PositiveInfinity,
            SpeculativeMargin = 0.1f
        };
    }
}
