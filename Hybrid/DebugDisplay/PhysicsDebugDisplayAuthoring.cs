#if UNITY_EDITOR
using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Physics Debug Display Data.
    /// Component data containing physics debug display settings.
    /// </summary>
    public struct PhysicsDebugDisplayData : IComponentData
    {
        /// <summary>
        /// Enable or disable debug display of the colliders' bounding volume used during the Broadphase.
        /// </summary>
        public int DrawBroadphase;

        public int DrawColliderAabbs;
    }

    /// <summary>
    /// Physics Debug Display Authoring.
    /// GameObject component containing physics debug display settings.
    /// </summary>
    [AddComponentMenu("Entities/Physics/Physics Debug Display")]
    [DisallowMultipleComponent]
    public sealed class PhysicsDebugDisplayAuthoring : MonoBehaviour
    {
        PhysicsDebugDisplayAuthoring() { }

        /// <summary>
        /// Enables the debug display of the colliders' bounding volumes used during the Broadphase.
        /// </summary>
        [Tooltip("Enables the debug display of the colliders' bounding volumes used during the Broadphase.")]
        public bool DrawBroadphase = false;

        [Tooltip("Enables the debug display of the colliders' AABBs as consumed by the engine (velocity-swept and margin-expanded Eabbs — not the raw collider AABBs).")]
        public bool DrawColliderAabbs = false;

        internal PhysicsDebugDisplayData AsComponent => new PhysicsDebugDisplayData
        {
            DrawBroadphase = DrawBroadphase ? 1 : 0,
            DrawColliderAabbs = DrawColliderAabbs ? 1 : 0
        };
    }

    /// <summary>
    /// Baker for the <see cref="PhysicsDebugDisplayAuthoring>"/>
    /// </summary>
    internal class PhysicsDebugDisplayDataBaker : Baker<PhysicsDebugDisplayAuthoring>
    {
        /// <summary>
        /// Baking method to create the <see cref="PhysicsDebugDisplayAuthoring"/> component data.
        /// </summary>
        public override void Bake(PhysicsDebugDisplayAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, authoring.AsComponent);
        }
    }
}
#endif
