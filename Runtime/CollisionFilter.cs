using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct CollisionFilter : IEquatable<CollisionFilter>
    {
        public uint BelongsTo;
        public uint CollidesWith;
        public int GroupIndex;

        public static readonly CollisionFilter Default = new CollisionFilter
        {
            BelongsTo = 0xffffffff,
            CollidesWith = 0xffffffff,
            GroupIndex = 0
        };

        public static readonly CollisionFilter Zero = new CollisionFilter
        {
            BelongsTo = 0,
            CollidesWith = 0,
            GroupIndex = 0
        };

        public bool IsEmpty => (BelongsTo == 0 || CollidesWith == 0) && GroupIndex <= 0;

        public bool Equals(CollisionFilter other)
        {
            return BelongsTo == other.BelongsTo && CollidesWith == other.CollidesWith && GroupIndex == other.GroupIndex;
        }

        public override bool Equals(object obj) => obj is CollisionFilter other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                return (int)(BelongsTo * 397 ^ CollidesWith * 31 ^ (uint)GroupIndex);
            }
        }

        public static bool IsCollisionEnabled(CollisionFilter filterA, CollisionFilter filterB)
        {
            if (filterA.GroupIndex != 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return filterA.GroupIndex > 0;
            }
            return (filterA.BelongsTo & filterB.CollidesWith) != 0 && (filterB.BelongsTo & filterA.CollidesWith) != 0;
        }

        public static CollisionFilter CreateUnion(CollisionFilter filterA, CollisionFilter filterB)
        {
            return new CollisionFilter
            {
                BelongsTo = filterA.BelongsTo | filterB.BelongsTo,
                CollidesWith = filterA.CollidesWith | filterB.CollidesWith,
                GroupIndex = math.select(filterA.GroupIndex, filterB.GroupIndex, filterB.GroupIndex != 0)
            };
        }
    }

    public enum CollisionResponsePolicy : byte
    {
        Collide = 0,
        CollideRaiseCollisionEvents = 1,
        RaiseTriggerEvents = 3,
        None = byte.MaxValue - 1
    }

    internal enum MaterialFlags : byte
    {
        None = 0,
        DisableCollisions = 1 << 0,
        IsTrigger = 1 << 1,
        EnableCollisionEvents = 1 << 2
    }

    public enum CombinePolicy : byte
    {
        GeometricMean = 0,
        Minimum,
        Maximum,
        ArithmeticMean
    }

    public struct Material : IEquatable<Material>
    {
        internal MaterialFlags Flags;
        public CombinePolicy FrictionCombinePolicy;
        public CombinePolicy RestitutionCombinePolicy;
        public byte CustomTags;
        public float Friction;
        public float Restitution;

        public CollisionResponsePolicy CollisionResponse
        {
            get => FlagsToCollisionResponse(Flags);
            set => Flags = (Flags & ~(MaterialFlags.DisableCollisions | MaterialFlags.IsTrigger | MaterialFlags.EnableCollisionEvents))
                | CollisionResponseToFlags(value);
        }

        public static Material Default => new Material
        {
            FrictionCombinePolicy = CombinePolicy.GeometricMean,
            RestitutionCombinePolicy = CombinePolicy.GeometricMean,
            Friction = 0.5f,
            Restitution = 0.0f
        };

        internal static CollisionResponsePolicy FlagsToCollisionResponse(MaterialFlags flags)
        {
            if ((flags & MaterialFlags.DisableCollisions) != 0) return CollisionResponsePolicy.None;
            if ((flags & MaterialFlags.IsTrigger) != 0) return CollisionResponsePolicy.RaiseTriggerEvents;
            if ((flags & MaterialFlags.EnableCollisionEvents) != 0) return CollisionResponsePolicy.CollideRaiseCollisionEvents;
            return CollisionResponsePolicy.Collide;
        }

        internal static MaterialFlags CollisionResponseToFlags(CollisionResponsePolicy policy)
        {
            switch (policy)
            {
                case CollisionResponsePolicy.None: return MaterialFlags.DisableCollisions;
                case CollisionResponsePolicy.RaiseTriggerEvents: return MaterialFlags.IsTrigger;
                case CollisionResponsePolicy.CollideRaiseCollisionEvents: return MaterialFlags.EnableCollisionEvents;
                default: return 0;
            }
        }

        public static CollisionResponsePolicy GetCombinedCollisionResponse(Material materialA, Material materialB)
        {
            return FlagsToCollisionResponse(materialA.Flags | materialB.Flags);
        }

        public bool Equals(Material other)
        {
            return Flags == other.Flags
                && FrictionCombinePolicy == other.FrictionCombinePolicy
                && RestitutionCombinePolicy == other.RestitutionCombinePolicy
                && CustomTags == other.CustomTags
                && Friction == other.Friction
                && Restitution == other.Restitution;
        }

        public override bool Equals(object obj) => obj is Material other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = (int)Flags;
                hashCode = (hashCode * 397) ^ (int)FrictionCombinePolicy;
                hashCode = (hashCode * 397) ^ (int)RestitutionCombinePolicy;
                hashCode = (hashCode * 397) ^ CustomTags;
                hashCode = (hashCode * 397) ^ Friction.GetHashCode();
                hashCode = (hashCode * 397) ^ Restitution.GetHashCode();
                return hashCode;
            }
        }
    }
}
