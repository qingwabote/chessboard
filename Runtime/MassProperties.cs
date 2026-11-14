using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct MassDistribution : IEquatable<MassDistribution>
    {
        public RigidTransform Transform;
        public float3 InertiaTensor;

        public static readonly MassDistribution Zero = new MassDistribution
        {
            Transform = RigidTransform.identity,
            InertiaTensor = float3.zero
        };

        public bool Equals(MassDistribution other)
        {
            return Transform.Equals(other.Transform) && InertiaTensor.Equals(other.InertiaTensor);
        }

        public override int GetHashCode() => Transform.GetHashCode() * 397 ^ InertiaTensor.GetHashCode();
    }

    public struct MassProperties : IEquatable<MassProperties>
    {
        public MassDistribution MassDistribution;
        public float Volume;

        public static readonly MassProperties UnitSphere = new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = RigidTransform.identity,
                InertiaTensor = new float3(2f / 5f)
            },
            Volume = 4f / 3f * math.PI
        };

        public MassProperties Scale(float scale)
        {
            var result = this;
            result.MassDistribution.Transform.pos *= scale;
            result.MassDistribution.InertiaTensor *= scale * scale;
            result.Volume *= math.abs(scale * scale * scale);
            return result;
        }

        public static MassProperties CreateBox(float3 size)
        {
            return new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = RigidTransform.identity,
                    InertiaTensor = new float3(
                        size.y * size.y + size.z * size.z,
                        size.x * size.x + size.z * size.z,
                        size.x * size.x + size.y * size.y) / 12f
                },
                Volume = size.x * size.y * size.z
            };
        }

        public static MassProperties CreateSphere(float radius)
        {
            return new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = RigidTransform.identity,
                    InertiaTensor = new float3(2f / 5f * radius * radius)
                },
                Volume = 4f / 3f * math.PI * radius * radius * radius
            };
        }

        public bool Equals(MassProperties other)
        {
            return MassDistribution.Equals(other.MassDistribution)
                && Volume.Equals(other.Volume);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = MassDistribution.GetHashCode();
                hashCode = (hashCode * 397) ^ Volume.GetHashCode();
                return hashCode;
            }
        }
    }
}
