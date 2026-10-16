#if UNITY_EDITOR

using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Extensions;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    internal abstract class BasePhysicsBaker<TAuthoring> : Baker<TAuthoring>
        where TAuthoring : Component
    {
        static readonly List<UnityEngine.Collider> s_Colliders = new List<UnityEngine.Collider>();

        protected BlobAssetReference<Collider> BakeAllColliders(GameObject body, Entity bodyEntity, bool includeRigidbodyOwned)
        {
            var bodyTransform = body.transform;
            body.GetComponentsInChildren(true, s_Colliders);

            var forceUnique = body.GetComponent<ForceUniqueColliderAuthoring>();
            uint forceUniqueBase = forceUnique != null && forceUnique.enabled ? forceUnique.ForceUniqueID : 0u;

            var instances = new List<ColliderBlobInstance>();
            var singleIsOnBody = false;
            uint index = 0;
            try
            {
                foreach (var c in s_Colliders)
                {
                    if (!c.enabled) continue;
                    if (!includeRigidbodyOwned && c.GetComponentInParent<Rigidbody>(true) != null) continue;
                    DependsOn(c);

                    var blob = BakeSingleCollider(c, bodyTransform, forceUniqueBase + index);
                    instances.Add(new ColliderBlobInstance
                    {
                        Collider = blob,
                        CompoundFromChild = GetCompoundFromChild(c.transform, bodyTransform)
                    });
                    if (c.gameObject == body) singleIsOnBody = true;
                    ++index;
                }
            }
            finally
            {
                s_Colliders.Clear();
            }

            if (instances.Count == 0)
            {
                return BlobAssetReference<Collider>.Null;
            }

            BlobAssetReference<Collider> result;
            if (instances.Count == 1 && singleIsOnBody)
            {
                result = instances[0].Collider;
            }
            else
            {
                var array = new NativeArray<ColliderBlobInstance>(instances.Count, Allocator.Temp);
                for (int i = 0; i < instances.Count; i++) array[i] = instances[i];
                result = CompoundCollider.Create(array);
                array.Dispose();
                AddBlobAsset(ref result, out _);
            }
            AddComponent(bodyEntity, new PhysicsCollider { Value = result });
            return result;
        }

        BlobAssetReference<Collider> BakeSingleCollider(
            UnityEngine.Collider collider, Transform bodyTransform, uint forceUniqueID)
        {
            var filter = ProduceCollisionFilter(collider, bodyTransform.gameObject);
            var material = ProduceMaterial(collider);
            var bakeMatrix = ComputeBakeMatrix(collider.transform, bodyTransform);

            if (collider is UnityEngine.BoxCollider box)
            {
                var geometry = new BoxGeometry
                {
                    Center = bakeMatrix.TransformsPoint(box.center),
                    Orientation = quaternion.identity,
                    Size = box.size * BakeMatrixScale(bakeMatrix),
                    BevelRadius = 0f
                };
                var blob = BoxCollider.Create(geometry, filter, material, forceUniqueID);
                AddBlobAsset(ref blob, out _);
                return blob;
            }
            if (collider is UnityEngine.SphereCollider sphere)
            {
                var scale = BakeMatrixScale(bakeMatrix);
                var geometry = new SphereGeometry
                {
                    Center = bakeMatrix.TransformsPoint(sphere.center),
                    Radius = sphere.radius * math.cmax(math.abs(scale))
                };
                var blob = SphereCollider.Create(geometry, filter, material, forceUniqueID);
                AddBlobAsset(ref blob, out _);
                return blob;
            }
            return BlobAssetReference<Collider>.Null;
        }

        static CollisionFilter ProduceCollisionFilter(UnityEngine.Collider collider, GameObject body)
        {
            var layer = collider.gameObject.layer;
            var filter = new CollisionFilter { BelongsTo = 1u << layer };

            uint includeMask = 0u;
            for (var i = 0; i < 32; ++i)
            {
                includeMask |= UnityEngine.Physics.GetIgnoreLayerCollision(layer, i) ? 0 : 1u << i;
            }
            includeMask |= (uint)collider.includeLayers.value;
            var excludeMask = (uint)collider.excludeLayers.value;

            var rigidBody = body.GetComponent<Rigidbody>();
            if (rigidBody)
            {
                includeMask |= (uint)rigidBody.includeLayers.value;
                excludeMask |= (uint)rigidBody.excludeLayers.value;
            }

            filter.CollidesWith = includeMask & ~excludeMask;
            return filter;
        }

        static Material ProduceMaterial(UnityEngine.Collider collider)
        {
            var material = Material.Default;
            material.CollisionResponse = collider.isTrigger
                ? CollisionResponsePolicy.RaiseTriggerEvents
                : collider.providesContacts
                    ? CollisionResponsePolicy.CollideRaiseCollisionEvents
                    : CollisionResponsePolicy.Collide;
            return material;
        }

        static float4x4 ComputeBakeMatrix(Transform shape, Transform body)
        {
            var shapeL2W = (float4x4)shape.localToWorldMatrix;
            var bodyL2W = (float4x4)body.localToWorldMatrix;
            var relative = math.mul(math.inverse(bodyL2W), shapeL2W);

            var hasScaleOrShear = !relative.IsRigidScale() || !shapeL2W.IsRigidScale();
            if (!hasScaleOrShear)
            {
                return float4x4.identity;
            }

            if (!bodyL2W.HasShear() && !bodyL2W.HasNonUniformScale())
            {
                var bodyScale = bodyL2W.DecomposeScale();
                shapeL2W = math.mul(shapeL2W, float4x4.Scale(1f / bodyScale));
            }

            var rotation = quaternion.LookRotationSafe(
                math.normalize(((float3x3)shapeL2W).c2), math.normalize(((float3x3)shapeL2W).c1));
            var rigid = new RigidTransform(rotation, shapeL2W.c3.xyz);
            var bakeMatrix = math.mul(math.inverse(new float4x4(rigid)), shapeL2W);
            bakeMatrix.c0.w = 0f;
            bakeMatrix.c1.w = 0f;
            bakeMatrix.c2.w = 0f;
            bakeMatrix.c3.w = 1f;
            return bakeMatrix;
        }

        static float3 BakeMatrixScale(float4x4 m)
        {
            return new float3(
                math.length(((float3x3)m).c0),
                math.length(((float3x3)m).c1),
                math.length(((float3x3)m).c2));
        }

        static RigidTransform GetCompoundFromChild(Transform shape, Transform body)
        {
            var bodyL2W = (float4x4)body.localToWorldMatrix;
            var shapeL2W = (float4x4)shape.localToWorldMatrix;
            var relative = math.mul(math.inverse(bodyL2W), shapeL2W);
            var rotation = quaternion.LookRotationSafe(
                math.normalize(((float3x3)relative).c2), math.normalize(((float3x3)relative).c1));
            return new RigidTransform(rotation, relative.c3.xyz);
        }
    }

    internal class RigidbodyBaker : BasePhysicsBaker<Rigidbody>
    {
        public override void Bake(Rigidbody authoring)
        {
            DependsOn(authoring);
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var colliderBlob = BakeAllColliders(authoring.gameObject, entity, true);

            if (authoring.gameObject.isStatic)
            {
                return;
            }

            var scale = authoring.transform.lossyScale[0];
            var massProperties = colliderBlob.IsCreated ? colliderBlob.Value.MassProperties : MassProperties.UnitSphere;
            AddComponent(entity, authoring.isKinematic
                ? PhysicsMass.CreateKinematic(massProperties.Scale(scale))
                : PhysicsMass.CreateDynamic(massProperties.Scale(scale), authoring.mass).ApplyScale(math.rcp(scale)));

            AddComponent(entity, new PhysicsVelocity());

            if (!authoring.isKinematic)
            {
                AddComponent(entity, new PhysicsDamping
                {
                    Linear = authoring.linearDamping,
                    Angular = authoring.angularDamping
                });
                if (!authoring.useGravity)
                {
                    AddComponent(entity, new PhysicsGravityFactor { Value = 0f });
                }
            }
            else
            {
                AddComponent(entity, new PhysicsGravityFactor { Value = 0f });
            }
        }
    }

    internal abstract class ColliderBaker<TCollider> : BasePhysicsBaker<TCollider>
        where TCollider : UnityEngine.Collider
    {
        public override void Bake(TCollider authoring)
        {
            if (!authoring.enabled) return;
            if (authoring.GetComponentInParent<Rigidbody>(true) != null) return;

            var body = FindTopmostColliderBody(authoring.gameObject);
            if (body != authoring.gameObject) return;
            if (!IsFirstEnabledCollider(authoring)) return;

            DependsOn(authoring);
            var entity = GetEntity(body, TransformUsageFlags.Dynamic);
            BakeAllColliders(body, entity, false);
        }

        static bool IsFirstEnabledCollider(UnityEngine.Collider self)
        {
            var colliders = self.gameObject.GetComponents<UnityEngine.Collider>();
            foreach (var c in colliders)
            {
                if (c.enabled) return c == self;
            }
            return false;
        }

        GameObject FindTopmostColliderBody(GameObject shape)
        {
            var parents = shape.GetComponentsInParent<UnityEngine.Collider>(true);
            GameObject result = null;
            for (int i = parents.Length - 1; i >= 0; --i)
            {
                if (parents[i].enabled)
                {
                    result = parents[i].gameObject;
                    break;
                }
            }
            return result ?? shape;
        }
    }

    internal class BoxBaker : ColliderBaker<UnityEngine.BoxCollider>
    {
    }

    internal class SphereBaker : ColliderBaker<UnityEngine.SphereCollider>
    {
    }

    internal static class BakeMatrixExtensions
    {
        public static bool IsRigidScale(this float4x4 m)
        {
            var r = (float3x3)m;
            return math.all(math.abs(new float3(
                math.length(r.c0), math.length(r.c1), math.length(r.c2)) - 1f) < 1e-5f);
        }

        public static bool HasShear(this float4x4 m)
        {
            var r = (float3x3)m;
            var c0 = math.normalizesafe(r.c0, new float3(1, 0, 0));
            var c1 = math.normalizesafe(r.c1, new float3(0, 1, 0));
            var c2 = math.normalizesafe(r.c2, new float3(0, 0, 1));
            return math.abs(math.dot(c0, c1)) > 1e-5f || math.abs(math.dot(c0, c2)) > 1e-5f || math.abs(math.dot(c1, c2)) > 1e-5f;
        }

        public static bool HasNonUniformScale(this float4x4 m)
        {
            var r = (float3x3)m;
            var lengths = new float3(math.length(r.c0), math.length(r.c1), math.length(r.c2));
            return math.any(math.abs(lengths - lengths.x) > 1e-5f);
        }

        public static float DecomposeScale(this float4x4 m)
        {
            var r = (float3x3)m;
            return math.length(r.c0);
        }

        public static float3 TransformsPoint(this float4x4 m, float3 point)
        {
            return math.transform(m, point);
        }
    }
}

#endif
