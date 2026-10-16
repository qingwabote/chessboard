using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct RigidBody
    {
        public BlobAssetReference<Collider> Collider;
        public RigidTransform WorldFromBody;
        public Entity Entity;
        public float Scale;

        public static readonly RigidBody Zero = new RigidBody
        {
            Collider = BlobAssetReference<Collider>.Null,
            WorldFromBody = RigidTransform.identity,
            Entity = Entity.Null,
            Scale = 1f
        };

        public unsafe Aabb CalculateAabb()
        {
            if (!Collider.IsCreated) return Aabb.Empty;
            return Collider.CalculateAabb(WorldFromBody, Scale);
        }
    }

    public static class RigidBodyExtensions
    {
        public static Aabb CalculateAabb(this BlobAssetReference<Collider> collider, RigidTransform transform, float uniformScale = 1f)
        {
            unsafe
            {
                if (!collider.IsCreated) return Aabb.Empty;
                return Collider.CalculateAabb((Collider*)collider.GetUnsafePtr(), transform, uniformScale);
            }
        }
    }
}
