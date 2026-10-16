using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct ColliderBlobInstance : IEquatable<ColliderBlobInstance>
    {
        public RigidTransform CompoundFromChild;
        public BlobAssetReference<Collider> Collider;

        public bool Equals(ColliderBlobInstance other)
        {
            return CompoundFromChild.Equals(other.CompoundFromChild) && Collider.Equals(other.Collider);
        }

        public override bool Equals(object obj) => obj is ColliderBlobInstance other && Equals(other);

        public override int GetHashCode() => CompoundFromChild.GetHashCode() * 397 ^ Collider.GetHashCode();
    }

    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct CompoundCollider
    {
        [StructLayout(LayoutKind.Sequential)]
        public struct Child
        {
            public RigidTransform CompoundFromChild;
            public ColliderType Type;
            public BoxCollider Box;
            public SphereCollider Sphere;

            public Collider* GetColliderPtr()
            {
                var p = (Child*)System.Runtime.CompilerServices.Unsafe.AsPointer(ref this);
                return p->Type == ColliderType.Box ? (Collider*)&p->Box : (Collider*)&p->Sphere;
            }
        }

        internal ColliderHeader m_Header;
        internal int m_ChildCount;

        public int NumChildren => m_ChildCount;

        public Child* Children
        {
            get
            {
                var p = (CompoundCollider*)System.Runtime.CompilerServices.Unsafe.AsPointer(ref this);
                return (Child*)((byte*)p + sizeof(CompoundCollider));
            }
        }

        public CollisionFilter GetCollisionFilter() => m_Header.Filter;

        public static BlobAssetReference<Collider> Create(NativeArray<ColliderBlobInstance> children)
        {
            var filter = CollisionFilter.Zero;
            var massProperties = default(MassProperties);
            massProperties.MassDistribution = new MassDistribution
            {
                Transform = RigidTransform.identity,
                InertiaTensor = float3.zero
            };
            for (int i = 0; i < children.Length; i++)
            {
                filter = CollisionFilter.CreateUnion(filter, children[i].Collider.Value.GetCollisionFilter());
                if (children[i].Collider.IsCreated)
                {
                    var mp = children[i].Collider.Value.MassProperties;
                    massProperties.MassDistribution.InertiaTensor += mp.MassDistribution.InertiaTensor;
                    massProperties.Volume += mp.Volume;
                    massProperties.AngularExpansionFactor = math.max(massProperties.AngularExpansionFactor, mp.AngularExpansionFactor);
                }
            }

            var size = sizeof(CompoundCollider) + children.Length * sizeof(Child);
            var buffer = (byte*)UnsafeUtility.Malloc(size, 16, Allocator.Temp);
            UnsafeUtility.MemClear(buffer, size);

            var root = (CompoundCollider*)buffer;
            root->m_Header = new ColliderHeader
            {
                Type = ColliderType.Compound,
                CollisionType = CollisionType.Composite,
                Version = 1,
                Magic = 0,
                ForceUniqueBlobID = ColliderConstants.k_SharedBlobID,
                Filter = filter,
                Material = Material.Default,
                MassProperties = massProperties
            };
            root->m_ChildCount = children.Length;

            var dst = (Child*)(buffer + sizeof(CompoundCollider));
            for (int i = 0; i < children.Length; i++)
            {
                dst[i] = new Child { CompoundFromChild = children[i].CompoundFromChild };
                var src = (Collider*)children[i].Collider.GetUnsafePtr();
                if (src->Type == ColliderType.Box)
                {
                    dst[i].Type = ColliderType.Box;
                    dst[i].Box = *(BoxCollider*)src;
                }
                else if (src->Type == ColliderType.Sphere)
                {
                    dst[i].Type = ColliderType.Sphere;
                    dst[i].Sphere = *(SphereCollider*)src;
                }
            }

            var result = BlobAssetReference<Collider>.Create((Collider*)buffer, size);
            UnsafeUtility.Free(buffer, Allocator.Temp);
            return result;
        }

        internal static Aabb CalculateAabb(CompoundCollider* collider, RigidTransform transform, float uniformScale = 1f)
        {
            var result = Aabb.Empty;
            var children = (Child*)((byte*)collider + sizeof(CompoundCollider));
            for (int i = 0; i < collider->m_ChildCount; i++)
            {
                var childTransform = math.mul(transform, children[i].CompoundFromChild);
                var childAabb = Collider.CalculateAabb(children[i].GetColliderPtr(), childTransform, uniformScale);
                result = i == 0 ? childAabb : Aabb.Union(result, childAabb);
            }
            return result;
        }
    }
}
