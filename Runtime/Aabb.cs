using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct Aabb : IEquatable<Aabb>
    {
        public float3 Min;
        public float3 Max;

        public float3 Center => 0.5f * (Min + Max);
        public float3 Extents => Max - Min;
        public float3 HalfExtents => 0.5f * (Max - Min);

        public static readonly Aabb Empty = new Aabb
        {
            Min = new float3(float.MaxValue),
            Max = new float3(float.MinValue)
        };

        public bool IsEmpty => math.any(Min > Max);

        public bool Overlaps(Aabb other)
        {
            return math.all(Max >= other.Min) && math.all(other.Max >= Min);
        }

        public bool Contains(float3 point)
        {
            return math.all(Min <= point) && math.all(point <= Max);
        }

        public void Expand(float amount)
        {
            Min -= amount;
            Max += amount;
        }

        public static Aabb Union(Aabb a, Aabb b)
        {
            return new Aabb { Min = math.min(a.Min, b.Min), Max = math.max(a.Max, b.Max) };
        }

        public Aabb Transform(in RigidTransform transform)
        {
            var center = math.transform(transform, Center);
            var half = HalfExtents;
            var r = new float3x3(transform.rot);
            var ac0 = math.abs(r.c0);
            var ac1 = math.abs(r.c1);
            var ac2 = math.abs(r.c2);
            var worldHalf = ac0 * half.x + ac1 * half.y + ac2 * half.z;
            return new Aabb { Min = center - worldHalf, Max = center + worldHalf };
        }

        public bool Equals(Aabb other) => Min.Equals(other.Min) && Max.Equals(other.Max);

        public override int GetHashCode() => Min.GetHashCode() * 397 ^ Max.GetHashCode();
    }
}
