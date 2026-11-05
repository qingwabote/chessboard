#if UNITY_EDITOR
using Unity.DebugDisplay;
using Unity.Mathematics;
using UnityEngine;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Authoring
{

    /// <summary>
    /// A component system group that contains the physics debug display systems.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(LateSimulationSystemGroup))]
    public partial class PhysicsDebugDisplayGroup : ComponentSystemGroup
    {
    }

    /// <summary>
    /// A component system group that contains the physics debug display systems while in edit mode.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateAfter(typeof(TransformSystemGroup))]
    public partial class PhysicsDebugDisplayGroup_Editor : ComponentSystemGroup
    {
    }

    /// <summary>
    /// <para> A system which is responsible for drawing physics debug display data.
    /// Create a singleton entity with <see cref="PhysicsDebugDisplayData"/> and select what you want to be drawn.</para>
    ///
    /// If you want custom debug draw, you need to:<para/>
    /// 1) Create a system which updates before PhysicsDebugDisplaySystem.<br/>
    /// 2) In OnUpdate() of that system, call GetSingleton <see cref="PhysicsDebugDisplayData"/> (even if you are not using it, it is important to do so to properly chain dependencies).<br/>
    /// 3) Afterwards, in OnUpdate() or in scheduled jobs, call one of the exposed draw methods (Line, Box, ...).<br/>
    /// IMPORTANT: Drawing works only in the Editor.
    /// </summary>
    public abstract partial class PhysicsDebugDisplaySystem : SystemBase
    {
        GameObject m_DrawComponentGameObject;

        class DrawComponent : MonoBehaviour
        {
            public PhysicsDebugDisplaySystem System;
            public void OnDrawGizmos()
            {
                System?.CompleteDisplayDataDependencies();
                Unity.DebugDisplay.DebugDisplay.Render();
            }
        }

        protected override void OnCreate()
        {
            RequireForUpdate<PhysicsDebugDisplayData>();
            Unity.DebugDisplay.DebugDisplay.Instantiate();
        }

        protected override void OnStartRunning()
        {
            if (m_DrawComponentGameObject == null)
            {
                m_DrawComponentGameObject = new GameObject("PhysicsDebugDisplaySystem")
                {
                    hideFlags = HideFlags.DontSave | HideFlags.HideInHierarchy
                };

                // Note: we are adding an additional child here so that we can hide the parent in the hierarchy
                // without hiding the child, which would prevent the OnDrawGizmos method on the DrawComponent in the child
                // from being called.
                var childGameObject = new GameObject("DrawComponent")
                {
                    hideFlags = HideFlags.DontSave
                };
                childGameObject.transform.parent = m_DrawComponentGameObject.transform;

                var drawComponent = childGameObject.AddComponent<DrawComponent>();

                drawComponent.System = this;
            }
        }

        static void DestroyGameObject(GameObject gameObject)
        {
            if (Application.isPlaying)
                UnityEngine.Object.Destroy(gameObject);
            else
                UnityEngine.Object.DestroyImmediate(gameObject);
        }

        protected override void OnStopRunning()
        {
            if (m_DrawComponentGameObject != null)
            {
                var drawComponent = m_DrawComponentGameObject.GetComponentInChildren<DrawComponent>();
                if (drawComponent)
                {
                    drawComponent.System = null;
                }

                while (m_DrawComponentGameObject.transform.childCount > 0)
                {
                    var child = m_DrawComponentGameObject.transform.GetChild(0);
                    child.parent = null;
                    DestroyGameObject(child.gameObject);
                }

                DestroyGameObject(m_DrawComponentGameObject);

                m_DrawComponentGameObject = null;
            }
        }

        /// <summary>
        /// Draws a point.
        /// </summary>
        /// <param name="x"> World space position. </param>
        /// <param name="size"> Extents. </param>
        /// <param name="color"> Color. </param>
        public static void Point(float3 x, float size, ColorIndex color)
        {
            var lines = new Lines(3);

            lines.Draw(x - new float3(size, 0, 0), x + new float3(size, 0, 0), color);
            lines.Draw(x - new float3(0, size, 0), x + new float3(0, size, 0), color);
            lines.Draw(x - new float3(0, 0, size), x + new float3(0, 0, size), color);
        }

        /// <summary>
        /// Draws a line between 2 points.
        /// </summary>
        /// <param name="x0"> Point 0 in world space. </param>
        /// <param name="x1"> Point 1 in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Line(float3 x0, float3 x1, Unity.DebugDisplay.ColorIndex color)
        {
            new Lines(1).Draw(x0, x1, color);
        }

        /// <summary>
        /// Draws a box.
        /// </summary>
        /// <param name="Size"> Size of the box. </param>
        /// <param name="Center"> Center of the box in world space. </param>
        /// <param name="Orientation"> Orientation of the box in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Box(float3 Size, float3 Center, quaternion Orientation, ColorIndex color)
        {
            Unity.DebugDisplay.Box.Draw(Size, Center, Orientation, color);
        }

        protected override void OnUpdate()
        {
        }

        /// <summary>
        ///  Completes dependencies for all systems in the physics debug display groups, which ensures that the debug display
        ///  data is fully produced by the corresponding debug display data systems before it is being prepared for rendering by this system.
        /// </summary>
        void CompleteDisplayDataDependencies()
        {
            var displayGroup = World.GetExistingSystemManaged<PhysicsDebugDisplayGroup>();
            if (displayGroup != null)
            {
                using var systemHandles = displayGroup.GetAllSystems();
                foreach (var handle in systemHandles)
                {
                    World.Unmanaged.ResolveSystemStateRef(handle).CompleteDependency();
                }
            }

            var editorDisplayGroup = World.GetExistingSystemManaged<PhysicsDebugDisplayGroup_Editor>();
            if (editorDisplayGroup != null)
            {
                using var systemHandles = editorDisplayGroup.GetAllSystems();
                foreach (var handle in systemHandles)
                {
                    World.Unmanaged.ResolveSystemStateRef(handle).CompleteDependency();
                }
            }
        }
    }

    /// <summary>
    /// Draws physics debug display data.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup), OrderLast = true)]
    [RequireMatchingQueriesForUpdate]
    public partial class PhysicsDebugDisplaySystem_Default : PhysicsDebugDisplaySystem
    { }

    /// <summary>
    /// Draws physics debug display data while in edit mode.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup_Editor), OrderLast = true)]
    [RequireMatchingQueriesForUpdate]
    public partial class PhysicsDebugDisplaySystem_Editor : PhysicsDebugDisplaySystem
    { }
}
#endif
