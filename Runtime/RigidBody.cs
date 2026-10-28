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

        public unsafe Aabb CalculateAabb()
        {
            return Collider.CalculateAabb(WorldFromBody, Scale);
        }
    }

    public static class RigidBodyExtensions
    {
        public static Aabb CalculateAabb(this BlobAssetReference<Collider> collider, RigidTransform transform, float uniformScale = 1f)
        {
            unsafe
            {
                return Collider.CalculateAabb((Collider*)collider.GetUnsafePtr(), transform, uniformScale);
            }
        }
    }
}
