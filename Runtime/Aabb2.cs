using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Unity.Physics
{
    [StructLayout(LayoutKind.Explicit)]
    internal struct Aabb2
    {
        [FieldOffset(0)] public float4 Val;

        [FieldOffset(0)] public float2 Min;
        [FieldOffset(8)] public float2 Max;

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

    [StructLayout(LayoutKind.Explicit)]
    internal struct Range2
    {
        [FieldOffset(0)] public int4 Val;

        [FieldOffset(0)] public int2 Min;
        [FieldOffset(8)] public int2 Max;

        public static implicit operator Range2(int4 a)
        {
            return new Range2 { Val = a };
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(Min, Max);
        }

        public struct Enumerator
        {
            readonly int2 m_Min;
            readonly int2 m_Max;
            int2 m_Index;

            public Enumerator(int2 min, int2 max)
            {
                m_Min = min;
                m_Max = max;
                m_Index = new int2(min.x - 1, max.y);
            }

            public int2 Current
            {
                get { return m_Index; }
            }

            public bool MoveNext()
            {
                if (++m_Index.y <= m_Max.y)
                {
                    return true;
                }
                m_Index.y = m_Min.y;
                if (++m_Index.x > m_Max.x)
                {
                    return false;
                }
                return m_Index.y <= m_Max.y;
            }
        }
    }
}
