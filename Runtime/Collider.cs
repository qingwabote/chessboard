using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public enum ColliderType : byte
    {
        Convex = 0,
        Sphere = 1,
        Capsule = 2,
        Triangle = 3,
        Quad = 4,
        Box = 5,
        Cylinder = 6,
        Mesh = 7,
        Compound = 8,
        Terrain = 9
    }

    public enum CollisionType : byte
    {
        Convex = 0,
        Composite = 1,
        Terrain = 2
    }

    public static class ColliderConstants
    {
        public const uint k_SharedBlobID = 0u;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ColliderHeader
    {
        public ColliderType Type;
        public CollisionType CollisionType;
        public byte Version;
        public byte Magic;
        public uint ForceUniqueBlobID;
        public CollisionFilter Filter;
        public Material Material;
        public MassProperties MassProperties;
    }

    public unsafe struct Collider
    {
        internal ColliderHeader m_Header;

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public CollisionFilter GetCollisionFilter() => m_Header.Filter;
        public void SetCollisionFilter(CollisionFilter filter) => m_Header.Filter = filter;
        public MassProperties MassProperties => m_Header.MassProperties;
        public bool IsUnique => m_Header.ForceUniqueBlobID != ColliderConstants.k_SharedBlobID;
        public void SetForceUniqueID(uint id) => m_Header.ForceUniqueBlobID = id;
        internal uint ForceUniqueBlobID => m_Header.ForceUniqueBlobID;
        internal bool RespondsToCollision => Material.FlagsToCollisionResponse(m_Header.Material.Flags) != CollisionResponsePolicy.None;

        internal static int SizeOf(Collider* collider)
        {
            switch (collider->Type)
            {
                case ColliderType.Box: return sizeof(BoxCollider);
                case ColliderType.Sphere: return sizeof(SphereCollider);
                case ColliderType.Compound: return sizeof(CompoundCollider) + ((CompoundCollider*)collider)->NumChildren * sizeof(CompoundCollider.Child);
                default: return sizeof(Collider);
            }
        }

        public BlobAssetReference<Collider> Clone()
        {
            var ptr = (Collider*)System.Runtime.CompilerServices.Unsafe.AsPointer(ref this);
            return BlobAssetReference<Collider>.Create(ptr, SizeOf(ptr));
        }

        internal static Aabb CalculateAabb(Collider* collider, RigidTransform transform, float uniformScale = 1f)
        {
            switch (collider->Type)
            {
                case ColliderType.Box: return BoxCollider.CalculateAabb((BoxCollider*)collider, transform, uniformScale);
                case ColliderType.Sphere: return SphereCollider.CalculateAabb((SphereCollider*)collider, transform, uniformScale);
                case ColliderType.Compound: return CompoundCollider.CalculateAabb((CompoundCollider*)collider, transform, uniformScale);
                default: return Aabb.Empty;
            }
        }
    }
}
