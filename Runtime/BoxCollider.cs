using System;
using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct BoxGeometry : IEquatable<BoxGeometry>
    {
        public float3 Center;
        public quaternion Orientation;
        public float3 Size;
        public float BevelRadius;

        public static readonly BoxGeometry Default = new BoxGeometry
        {
            Center = float3.zero,
            Orientation = quaternion.identity,
            Size = new float3(1f),
            BevelRadius = 0.05f
        };

        public bool Equals(BoxGeometry other)
        {
            return Center.Equals(other.Center)
                && Orientation.value.Equals(other.Orientation.value)
                && Size.Equals(other.Size)
                && BevelRadius.Equals(other.BevelRadius);
        }

        public override bool Equals(object obj) => obj is BoxGeometry other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = Center.GetHashCode();
                hashCode = (hashCode * 397) ^ Orientation.value.GetHashCode();
                hashCode = (hashCode * 397) ^ Size.GetHashCode();
                hashCode = (hashCode * 397) ^ BevelRadius.GetHashCode();
                return hashCode;
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct BoxCollider
    {
        internal ColliderHeader m_Header;
        internal float3 m_Center;
        internal float3 m_Size;
        internal quaternion m_Orientation;

        public BoxGeometry Geometry
        {
            get => new BoxGeometry { Center = m_Center, Orientation = m_Orientation, Size = m_Size, BevelRadius = 0f };
            set
            {
                m_Center = value.Center;
                m_Size = value.Size;
                m_Orientation = value.Orientation;
            }
        }

        public CollisionFilter GetCollisionFilter() => m_Header.Filter;

        public static BlobAssetReference<Collider> Create(
            BoxGeometry geometry, CollisionFilter filter, Material material,
            uint forceUniqueBlobID = ColliderConstants.k_SharedBlobID)
        {
            var collider = new BoxCollider
            {
                m_Header = new ColliderHeader
                {
                    Type = ColliderType.Box,
                    CollisionType = CollisionType.Convex,
                    Version = 1,
                    Magic = 0,
                    ForceUniqueBlobID = forceUniqueBlobID,
                    Filter = filter,
                    Material = material,
                    MassProperties = MassProperties.CreateBox(geometry.Size)
                },
                m_Center = geometry.Center,
                m_Size = geometry.Size,
                m_Orientation = geometry.Orientation
            };
            return BlobAssetReference<Collider>.Create(&collider, sizeof(BoxCollider));
        }

        internal static Aabb CalculateAabb(BoxCollider* collider, RigidTransform transform, float uniformScale = 1f)
        {
            var rot = math.mul(transform.rot, collider->m_Orientation);
            var center = math.transform(transform, collider->m_Center * uniformScale);
            var half = 0.5f * collider->m_Size * uniformScale;
            var r = new float3x3(rot);
            var ac0 = math.abs(r.c0);
            var ac1 = math.abs(r.c1);
            var ac2 = math.abs(r.c2);
            var worldHalf = ac0 * half.x + ac1 * half.y + ac2 * half.z;
            return new Aabb { Min = center - worldHalf, Max = center + worldHalf };
        }
    }
}
