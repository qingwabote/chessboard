#if UNITY_EDITOR

using System;
using Unity.Burst;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    internal struct Boxes : IDisposable
    {
        private Lines m_Lines;

        internal Boxes(int count)
        {
            m_Lines = new Lines(count * 12);
        }

        internal void Draw(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            float3x3 mat = math.float3x3(Orientation);
            float3 x = mat.c0 * Size.x * 0.5f;
            float3 y = mat.c1 * Size.y * 0.5f;
            float3 z = mat.c2 * Size.z * 0.5f;
            float3 c0 = Center - x - y - z;
            float3 c1 = Center - x - y + z;
            float3 c2 = Center - x + y - z;
            float3 c3 = Center - x + y + z;
            float3 c4 = Center + x - y - z;
            float3 c5 = Center + x - y + z;
            float3 c6 = Center + x + y - z;
            float3 c7 = Center + x + y + z;

            m_Lines.Draw(c0, c1, color); // ring 0
            m_Lines.Draw(c1, c3, color);
            m_Lines.Draw(c3, c2, color);
            m_Lines.Draw(c2, c0, color);

            m_Lines.Draw(c4, c5, color); // ring 1
            m_Lines.Draw(c5, c7, color);
            m_Lines.Draw(c7, c6, color);
            m_Lines.Draw(c6, c4, color);

            m_Lines.Draw(c0, c4, color); // between rings
            m_Lines.Draw(c1, c5, color);
            m_Lines.Draw(c2, c6, color);
            m_Lines.Draw(c3, c7, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Box
    {
        internal static void Draw(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            new Boxes(1).Draw(Size, Center, Orientation, color);
        }
    }

    internal struct Lines : IDisposable
    {
        Unit m_Unit;
        internal Lines(int count)
        {
            DebugDisplay.Instantiate();
            m_Unit = Unmanaged.Instance.Data.m_LineBufferAllocations.AllocateAtomic(count);
        }

        internal void Draw(float3 begin, float3 end, ColorIndex color)
        {
            if (m_Unit.m_Next < m_Unit.m_End)
            {
                Unmanaged.Instance.Data.m_LineBuffer.SetLine(begin, end, color, m_Unit.m_Next++);
            }
            else
            {
                Unmanaged.Instance.Data.m_LineBuffer.RequestResize();
            }
        }

        public void Dispose()
        {
            while (m_Unit.m_Next < m_Unit.m_End)
                Unmanaged.Instance.Data.m_LineBuffer.ClearLine(m_Unit.m_Next++);
        }
    }

    internal class DebugDisplay
    {
        internal static void Render()
        {
            Managed.Instance.CopyFromCpuToGpu();
            Managed.Instance.Render();
        }

        internal static void Clear()
        {
            Managed.Instance.Clear();
        }

        [BurstDiscard]
        internal static void Instantiate()
        {
            if (Managed.Instance == null)
                Managed.Instance = new Managed();
        }

        internal static void Dispose()
        {
            if (Managed.Instance == null)
                Managed.Instance.Dispose();
        }
    }
}

#endif
