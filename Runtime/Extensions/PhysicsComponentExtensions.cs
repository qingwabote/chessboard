using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics.Extensions
{
    public static class PhysicsComponentExtensions
    {
        public static PhysicsMass ApplyScale(in this PhysicsMass pm, float uniformScale)
        {
            var scaledBodyMass = pm;
            if (math.abs(uniformScale - 1.0f) > 1e-6f)
            {
                scaledBodyMass.CenterOfMass *= uniformScale;

                float absScale = math.abs(uniformScale);
                scaledBodyMass.InverseInertia *= math.rcp(math.pow(absScale, 5));
            scaledBodyMass.InverseMass *= math.rcp(math.pow(absScale, 3));
        }
        return scaledBodyMass;
    }

    public static void MakeUnique(ref this PhysicsCollider collider, in Entity entity, EntityManager entityManager)
        {
            if (CloneAndCreateCleanupDataIfRequired(ref collider, out var data))
            {
                entityManager.AddComponentData(entity, data);
            }
        }

        public static void MakeUnique(ref this PhysicsCollider collider, in Entity entity, EntityCommandBuffer ecb)
        {
            if (CloneAndCreateCleanupDataIfRequired(ref collider, out var data))
            {
                ecb.AddComponent(entity, data);
            }
        }

        public static void MakeUnique(
            ref this PhysicsCollider collider, in Entity entity,
            EntityCommandBuffer.ParallelWriter ecbParallelWriter, int sortKey)
        {
            if (CloneAndCreateCleanupDataIfRequired(ref collider, out var data))
            {
                ecbParallelWriter.AddComponent(sortKey, entity, data);
            }
        }

        static bool CloneAndCreateCleanupDataIfRequired(ref PhysicsCollider collider, out ColliderBlobCleanupData data)
        {
            if (collider.IsUnique)
            {
                data = default;
                return false;
            }
            var blobClone = collider.Value.Value.Clone();
            collider.Value = blobClone;
            data = new ColliderBlobCleanupData { Value = blobClone };
            return true;
        }
    }
}
