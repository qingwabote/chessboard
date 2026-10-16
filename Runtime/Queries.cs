using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public enum QueryInteraction
    {
        Default = 0,
        IgnoreTriggers = 1,
        IgnoreColliders = 2
    }

    public struct OverlapAabbInput
    {
        public Aabb Aabb;
        public CollisionFilter Filter;
    }

    internal struct PointDistanceInput
    {
        public float3 Position;
        public float MaxDistance;
        public CollisionFilter Filter;
    }

    public struct DistanceHit
    {
        public float Fraction;
        public int RigidBodyIndex;
        public Entity Entity;
        public float3 Position;
        public float3 SurfaceNormal;

        public float Distance => Fraction;
    }

    public unsafe struct ColliderCastInput
    {
        public Collider* Collider;
        public quaternion Orientation;
        public float3 Start;
        public float3 End;
        public float QueryColliderScale;

        public ColliderCastInput(BlobAssetReference<Collider> collider, float3 start, float3 end) : this()
        {
            Collider = (Collider*)collider.GetUnsafePtr();
            Start = start;
            End = end;
            Orientation = quaternion.identity;
            QueryColliderScale = 1f;
        }
    }

    public struct ColliderCastHit
    {
        public float Fraction;
        public int RigidBodyIndex;
        public Entity Entity;
        public float3 Position;
        public float3 SurfaceNormal;
    }
}
