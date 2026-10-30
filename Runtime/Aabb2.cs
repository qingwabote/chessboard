using Unity.Mathematics;

namespace Unity.Physics
{
    internal struct Aabb2
    {
        public float2 Min;
        public float2 Max;

        public bool Overlaps(Aabb2 other)
        {
            return math.all(Max >= other.Min) && math.all(other.Max >= Min);
        }

        public static Aabb2 Union(Aabb2 a, Aabb2 b)
        {
            return new Aabb2 { Min = math.min(a.Min, b.Min), Max = math.max(a.Max, b.Max) };
        }

        public void Expand(float amount)
        {
            Min -= amount;
            Max += amount;
        }

        public static implicit operator Aabb2(Aabb a)
        {
            return new Aabb2 { Min = a.Min.xz, Max = a.Max.xz };
        }
    }
}
