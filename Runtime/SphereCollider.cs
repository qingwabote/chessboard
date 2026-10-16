using System;
using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct SphereGeometry : IEquatable<SphereGeometry>
    {
        public float3 Center;
        public float Radius;

        public bool Equals(SphereGeometry other) => Center.Equals(other.Center) && Radius.Equals(other.Radius);

        public override bool Equals(object obj) => obj is SphereGeometry other && Equals(other);

        public override int GetHashCode() => Center.GetHashCode() * 397 ^ Radius.GetHashCode();
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct SphereCollider
    {
        internal ColliderHeader m_Header;
        internal float3 m_Center;
        internal float m_Radius;

        public float3 Center => m_Center;
        public float Radius => m_Radius;

        public SphereGeometry Geometry => new SphereGeometry { Center = m_Center, Radius = m_Radius };

        public CollisionFilter GetCollisionFilter() => m_Header.Filter;

        public static BlobAssetReference<Collider> Create(
            SphereGeometry geometry, CollisionFilter filter, Material material,
            uint forceUniqueBlobID = ColliderConstants.k_SharedBlobID)
        {
            var collider = new SphereCollider
            {
                m_Header = new ColliderHeader
                {
                    Type = ColliderType.Sphere,
                    CollisionType = CollisionType.Convex,
                    Version = 1,
                    Magic = 0,
                    ForceUniqueBlobID = forceUniqueBlobID,
                    Filter = filter,
                    Material = material,
                    MassProperties = MassProperties.CreateSphere(geometry.Radius)
                },
                m_Center = geometry.Center,
                m_Radius = geometry.Radius
            };
            return BlobAssetReference<Collider>.Create(&collider, sizeof(SphereCollider));
        }

        internal static Aabb CalculateAabb(SphereCollider* collider, RigidTransform transform, float uniformScale = 1f)
        {
            var center = math.transform(transform, collider->m_Center * uniformScale);
            var radius = collider->m_Radius * math.abs(uniformScale);
            return new Aabb { Min = center - radius, Max = center + radius };
        }
    }
}
