using Unity.Collections;
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
                scaledBodyMass.AngularExpansionFactor *= absScale;
            }
            return scaledBodyMass;
        }

        public static void Integrate(
            this in PhysicsVelocity physicsVelocity, in PhysicsMass physicsMass, float timestep,
            ref float3 position, ref quaternion orientation)
        {
            position += physicsVelocity.Linear * timestep;
            if (math.any(physicsVelocity.Angular))
            {
                var worldFromMotion = math.mul(orientation, physicsMass.Transform.rot);
                var angularVelocityWS = math.rotate(worldFromMotion, physicsVelocity.Angular);
                orientation = IntegrateOrientation(orientation, angularVelocityWS, timestep);
            }
        }

        static quaternion IntegrateOrientation(quaternion orientation, float3 angularVelocity, float timestep)
        {
            float angle = math.length(angularVelocity) * timestep;
            if (angle < 1e-8f)
            {
                return orientation;
            }
            var axis = math.normalizesafe(angularVelocity, math.up());
            int numSteps = (int)math.max(1f, angle * (4f / math.PI));
            float stepAngle = angle / numSteps;
            var step = quaternion.AxisAngle(axis, stepAngle);
            var result = orientation;
            for (int i = 0; i < numSteps; i++)
            {
                result = math.mul(step, result);
            }
            return math.normalizesafe(result, orientation);
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
