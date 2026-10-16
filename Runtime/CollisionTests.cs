using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Physics
{
    internal static class CollisionTests
    {
        internal struct Leaf
        {
            public ColliderType Type;
            public RigidTransform World;
            public float3 HalfExtents;
            public float Radius;
            public MaterialFlags Flags;
        }

        internal static unsafe void CollectLeaves(RigidBody body, NativeList<Leaf> leaves)
        {
            if (!body.Collider.IsCreated) return;
            var collider = (Collider*)body.Collider.GetUnsafePtr();
            CollectLeaves(collider, body.WorldFromBody, body.Scale, leaves);
        }

        internal static unsafe void CollectLeaves(Collider* collider, RigidTransform world, float scale, NativeList<Leaf> leaves)
        {
            if (collider->Type == ColliderType.Compound)
            {
                var compound = (CompoundCollider*)collider;
                var children = compound->Children;
                for (int i = 0; i < compound->NumChildren; i++)
                {
                    CollectLeaves(
                        children[i].GetColliderPtr(),
                        math.mul(world, children[i].CompoundFromChild), scale, leaves);
                }
                return;
            }
            if (collider->Type == ColliderType.Box)
            {
                var box = (BoxCollider*)collider;
                leaves.Add(new Leaf
                {
                    Type = ColliderType.Box,
                    World = new RigidTransform(
                        math.mul(world.rot, box->m_Orientation),
                        world.pos + math.rotate(world.rot, box->m_Center * scale)),
                    HalfExtents = 0.5f * box->m_Size * scale,
                    Flags = box->m_Header.Material.Flags
                });
                return;
            }
            if (collider->Type == ColliderType.Sphere)
            {
                var sphere = (SphereCollider*)collider;
                leaves.Add(new Leaf
                {
                    Type = ColliderType.Sphere,
                    World = new RigidTransform(
                        world.rot, world.pos + math.rotate(world.rot, sphere->m_Center * scale)),
                    Radius = sphere->m_Radius * scale,
                    Flags = sphere->m_Header.Material.Flags
                });
            }
        }

        internal static bool TestLeaves(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            if (a.Type == ColliderType.Sphere && b.Type == ColliderType.Sphere)
                return SphereSphere(a, b, out normal, out depth);
            if (a.Type == ColliderType.Sphere)
                return SphereBox(a, b, out normal, out depth);
            if (b.Type == ColliderType.Sphere)
            {
                var hit = SphereBox(b, a, out normal, out depth);
                normal = -normal;
                return hit;
            }
            return BoxBox(a, b, out normal, out depth);
        }

        static bool SphereSphere(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var delta = a.World.pos - b.World.pos;
            var distSq = math.lengthsq(delta);
            var sum = a.Radius + b.Radius;
            if (distSq >= sum * sum) return false;
            var dist = math.sqrt(distSq);
            normal = dist > 1e-9f ? delta / dist : new float3(0f, 1f, 0f);
            depth = sum - dist;
            return true;
        }

        static bool SphereBox(in Leaf sphere, in Leaf box, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var local = math.rotate(math.inverse(box.World.rot), sphere.World.pos - box.World.pos);
            var clamped = math.clamp(local, -box.HalfExtents, box.HalfExtents);
            var delta = local - clamped;
            var distSq = math.lengthsq(delta);
            if (distSq > 1e-18f)
            {
                if (distSq >= sphere.Radius * sphere.Radius) return false;
                var dist = math.sqrt(distSq);
                normal = math.rotate(box.World.rot, delta / dist);
                depth = sphere.Radius - dist;
                return true;
            }
            var penetration = box.HalfExtents - math.abs(local);
            int axis = 0;
            var minP = penetration.x;
            if (penetration.y < minP) { minP = penetration.y; axis = 1; }
            if (penetration.z < minP) { minP = penetration.z; axis = 2; }
            var localNormal = new float3(
                axis == 0 ? math.sign(local.x) : 0f,
                axis == 1 ? math.sign(local.y) : 0f,
                axis == 2 ? math.sign(local.z) : 0f);
            normal = math.rotate(box.World.rot, localNormal);
            depth = sphere.Radius + minP;
            return true;
        }

        static bool BoxBox(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var ra = new float3x3(a.World.rot);
            var rb = new float3x3(b.World.rot);
            if (IsAxisPermutation(ra) && IsAxisPermutation(rb))
            {
                return AxisAlignedBoxes(a, b, out normal, out depth);
            }
            return SeparatingAxis(a, b, ra, rb, out normal, out depth);
        }

        static bool IsAxisPermutation(float3x3 r)
        {
            return HasSingleDominantAxis(r.c0) && HasSingleDominantAxis(r.c1) && HasSingleDominantAxis(r.c2);
        }

        static bool HasSingleDominantAxis(float3 v)
        {
            var a = math.abs(v);
            var max = math.cmax(a);
            return max > 0.999f && (a.x + a.y + a.z) - max < 0.05f;
        }

        static float3 WorldHalf(quaternion rot, float3 half)
        {
            var r = new float3x3(rot);
            return math.abs(r.c0) * half.x + math.abs(r.c1) * half.y + math.abs(r.c2) * half.z;
        }

        static bool AxisAlignedBoxes(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var ha = WorldHalf(a.World.rot, a.HalfExtents);
            var hb = WorldHalf(b.World.rot, b.HalfExtents);
            var d = a.World.pos - b.World.pos;
            var overlap = ha + hb - math.abs(d);
            if (math.any(overlap < 0f)) return false;
            int axis = 0;
            var minO = overlap.x;
            if (overlap.y < minO) { minO = overlap.y; axis = 1; }
            if (overlap.z < minO) { minO = overlap.z; axis = 2; }
            normal = new float3(
                axis == 0 ? math.sign(d.x) : 0f,
                axis == 1 ? math.sign(d.y) : 0f,
                axis == 2 ? math.sign(d.z) : 0f);
            if (math.all(normal == float3.zero)) normal = new float3(0f, 1f, 0f);
            depth = minO;
            return true;
        }

        static bool SeparatingAxis(in Leaf a, in Leaf b, float3x3 ra, float3x3 rb, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var d = b.World.pos - a.World.pos;
            var aHalf = a.HalfExtents;
            var bHalf = b.HalfExtents;
            var minOverlap = float.MaxValue;
            var bestAxis = new float3(0f, 1f, 0f);

            bool TestAxis(float3 axis)
            {
                var length = math.length(axis);
                if (length < 1e-6f) return true;
                axis /= length;
                var dist = math.abs(math.dot(d, axis));
                var projA = RadiusOnAxis(ra, aHalf, axis);
                var projB = RadiusOnAxis(rb, bHalf, axis);
                var overlap = projA + projB - dist;
                if (overlap <= 0f) return false;
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    bestAxis = axis;
                }
                return true;
            }

            if (!TestAxis(ra.c0)) return false;
            if (!TestAxis(ra.c1)) return false;
            if (!TestAxis(ra.c2)) return false;
            if (!TestAxis(rb.c0)) return false;
            if (!TestAxis(rb.c1)) return false;
            if (!TestAxis(rb.c2)) return false;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (!TestAxis(math.cross(ra[i], rb[j]))) return false;
                }
            }

            if (math.dot(bestAxis, d) > 0f)
            {
                bestAxis = -bestAxis;
            }
            normal = bestAxis;
            depth = minOverlap;
            return true;
        }

        static float RadiusOnAxis(float3x3 r, float3 half, float3 axis)
        {
            return math.abs(math.dot(axis, r.c0)) * half.x
                + math.abs(math.dot(axis, r.c1)) * half.y
                + math.abs(math.dot(axis, r.c2)) * half.z;
        }

        internal static Aabb LeafAabb(in Leaf leaf)
        {
            if (leaf.Type == ColliderType.Sphere)
            {
                return new Aabb { Min = leaf.World.pos - leaf.Radius, Max = leaf.World.pos + leaf.Radius };
            }
            var r = new float3x3(leaf.World.rot);
            var worldHalf = math.abs(r.c0) * leaf.HalfExtents.x + math.abs(r.c1) * leaf.HalfExtents.y + math.abs(r.c2) * leaf.HalfExtents.z;
            return new Aabb { Min = leaf.World.pos - worldHalf, Max = leaf.World.pos + worldHalf };
        }

        internal static float PointLeafDistance(in Leaf leaf, float3 point, out float3 surfaceNormal, out float3 closestPoint)
        {
            surfaceNormal = new float3(0f, 1f, 0f);
            closestPoint = point;
            if (leaf.Type == ColliderType.Sphere)
            {
                var delta = point - leaf.World.pos;
                var dist = math.length(delta);
                if (dist > 1e-9f)
                {
                    surfaceNormal = delta / dist;
                }
                closestPoint = leaf.World.pos + surfaceNormal * leaf.Radius;
                return dist - leaf.Radius;
            }
            var local = math.rotate(math.inverse(leaf.World.rot), point - leaf.World.pos);
            var clamped = math.clamp(local, -leaf.HalfExtents, leaf.HalfExtents);
            var delta2 = local - clamped;
            if (math.lengthsq(delta2) > 1e-18f)
            {
                var d = math.length(delta2);
                surfaceNormal = math.rotate(leaf.World.rot, delta2 / d);
                closestPoint = leaf.World.pos + math.rotate(leaf.World.rot, clamped);
                return d;
            }
            var penetration = leaf.HalfExtents - math.abs(local);
            int axis = 0;
            var minP = penetration.x;
            if (penetration.y < minP) { minP = penetration.y; axis = 1; }
            if (penetration.z < minP) { minP = penetration.z; axis = 2; }
            var localNormal = new float3(
                axis == 0 ? math.sign(local.x) : 0f,
                axis == 1 ? math.sign(local.y) : 0f,
                axis == 2 ? math.sign(local.z) : 0f);
            if (math.all(localNormal == float3.zero)) localNormal = new float3(0f, 1f, 0f);
            surfaceNormal = math.rotate(leaf.World.rot, localNormal);
            return -minP;
        }

        internal static float LeafPairDistance(in Leaf a, in Leaf b, out float3 normalAToB)
        {
            normalAToB = new float3(0f, 1f, 0f);
            var delta = b.World.pos - a.World.pos;
            if (a.Type == ColliderType.Sphere && b.Type == ColliderType.Sphere)
            {
                var dist = math.length(delta);
                if (dist > 1e-9f) normalAToB = delta / dist;
                return dist - a.Radius - b.Radius;
            }
            if (a.Type == ColliderType.Sphere || b.Type == ColliderType.Sphere)
            {
                var sphere = a.Type == ColliderType.Sphere ? a : b;
                var box = a.Type == ColliderType.Sphere ? b : a;
                var dist = PointLeafDistance(box, sphere.World.pos, out var boxNormal, out _);
                normalAToB = a.Type == ColliderType.Sphere ? -boxNormal : boxNormal;
                return dist - sphere.Radius;
            }
            var ra = new float3x3(a.World.rot);
            var rb = new float3x3(b.World.rot);
            if (IsAxisPermutation(ra) && IsAxisPermutation(rb))
            {
                var ha = WorldHalf(a.World.rot, a.HalfExtents);
                var hb = WorldHalf(b.World.rot, b.HalfExtents);
                var gap = math.abs(delta) - ha - hb;
                var separating = math.max(gap, 0f);
                var overlapping = math.max(-gap, 0f);
                if (math.any(overlapping > 0f))
                {
                    return -math.cmax(overlapping);
                }
                var d = math.length(separating);
                if (d > 1e-9f)
                {
                    var dir = math.select(float3.zero, math.sign(delta), gap > 0f);
                    normalAToB = math.normalizesafe(dir, new float3(0f, 1f, 0f));
                }
                return d;
            }
            var aabbA = LeafAabb(a);
            var aabbB = LeafAabb(b);
            var aabbGap = math.max(aabbB.Min - aabbA.Max, aabbA.Min - aabbB.Max);
            var aabbOverlap = math.max(-aabbGap, 0f);
            if (math.any(aabbOverlap > 0f))
            {
                return -math.cmax(aabbOverlap);
            }
            return math.length(math.max(aabbGap, 0f));
        }
    }
}
