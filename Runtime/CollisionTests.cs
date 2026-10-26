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
            public float2 ShadowCenter;
            public float2 E0;
            public float2 E1;
            public float2 E2;
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
                var rot = math.mul(world.rot, box->m_Orientation);
                var pos = world.pos + math.rotate(world.rot, box->m_Center * scale);
                var half = 0.5f * box->m_Size * scale;
                var r = new float3x3(rot);
                leaves.Add(new Leaf
                {
                    Type = ColliderType.Box,
                    World = new RigidTransform(rot, pos),
                    HalfExtents = half,
                    ShadowCenter = pos.xz,
                    E0 = r.c0.xz * half.x,
                    E1 = r.c1.xz * half.y,
                    E2 = r.c2.xz * half.z,
                    Flags = box->m_Header.Material.Flags
                });
                return;
            }
            if (collider->Type == ColliderType.Sphere)
            {
                var sphere = (SphereCollider*)collider;
                var pos = world.pos + math.rotate(world.rot, sphere->m_Center * scale);
                leaves.Add(new Leaf
                {
                    Type = ColliderType.Sphere,
                    World = new RigidTransform(world.rot, pos),
                    Radius = sphere->m_Radius * scale,
                    ShadowCenter = pos.xz,
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
            var delta = a.ShadowCenter - b.ShadowCenter;
            var distSq = math.lengthsq(delta);
            var sum = a.Radius + b.Radius;
            if (distSq >= sum * sum) return false;
            var dist = math.sqrt(distSq);
            normal = dist > 1e-9f ? new float3(delta.x / dist, 0f, delta.y / dist) : new float3(0f, 0f, 1f);
            depth = sum - dist;
            return true;
        }

        static bool SphereBox(in Leaf sphere, in Leaf box, out float3 normal, out float depth)
        {
            return CircleShadow(box, sphere.ShadowCenter - box.ShadowCenter, sphere.Radius,
                out normal, out depth, out _, out _);
        }

        static float ShadowHalf(in Leaf box, float2 axis)
        {
            return math.abs(math.dot(axis, box.E0))
                + math.abs(math.dot(axis, box.E1))
                + math.abs(math.dot(axis, box.E2));
        }

        static bool CircleShadow(in Leaf box, float2 c, float radius, out float3 normal, out float depth, out bool inside, out float2 closestXz)
        {
            normal = new float3(0f, 0f, 1f);
            depth = radius;
            inside = true;
            closestXz = box.ShadowCenter + c;

            var minFacetDepth = float.MaxValue;
            var bestFacet = new float3(0f, 0f, 1f);
            var bestFacetLocal = new float2(0f, 1f);
            var generatorCount = 0;

            for (int i = 0; i < 3; i++)
            {
                var e = i == 0 ? box.E0 : i == 1 ? box.E1 : box.E2;
                var length = math.length(e);
                if (length < 1e-6f) continue;
                generatorCount++;
                var axis = new float2(-e.y, e.x) / length;
                var half = ShadowHalf(box, axis);
                var proj = math.dot(axis, c);
                var absProj = math.abs(proj);
                if (absProj >= half) inside = false;
                var facetDepth = half - absProj;
                if (facetDepth < minFacetDepth)
                {
                    minFacetDepth = facetDepth;
                    var sign = proj < 0f ? -1f : 1f;
                    bestFacetLocal = axis * sign;
                    bestFacet = new float3(axis.x * sign, 0f, axis.y * sign);
                }
            }

            if (generatorCount == 0)
            {
                var dist = math.length(c);
                depth = radius - dist;
                if (dist > 1e-9f)
                {
                    normal = new float3(c.x / dist, 0f, c.y / dist);
                    closestXz = box.ShadowCenter;
                }
                return depth > 0f;
            }

            if (inside)
            {
                depth = minFacetDepth + radius;
                normal = bestFacet;
                closestXz = box.ShadowCenter + c + bestFacetLocal * minFacetDepth;
                return true;
            }

            var closest = float2.zero;
            var dist2 = ShadowEdgeDistance(box, c, out closest);
            depth = radius - dist2;
            closestXz = box.ShadowCenter + closest;
            if (dist2 > 1e-9f)
            {
                var dir = (c - closest) / dist2;
                normal = new float3(dir.x, 0f, dir.y);
            }
            else
            {
                normal = bestFacet;
            }
            return depth > 0f;
        }

        static float ShadowEdgeDistance(in Leaf box, float2 p, out float2 closest)
        {
            closest = p;
            var g0 = box.E0;
            var g1 = box.E1;
            var g2 = box.E2;
            SortByAngle(ref g0, ref g1);
            SortByAngle(ref g1, ref g2);
            SortByAngle(ref g0, ref g1);

            var vertex = -(g0 + g1 + g2);
            var minDistSq = float.MaxValue;
            var a = vertex;
            for (int i = 0; i < 6; i++)
            {
                var e = i < 3
                    ? 2f * (i == 0 ? g0 : i == 1 ? g1 : g2)
                    : -2f * (i == 3 ? g0 : i == 4 ? g1 : g2);
                var b = a + e;
                var abSq = math.lengthsq(e);
                if (abSq > 1e-12f)
                {
                    var t = math.clamp(math.dot(p - a, e) / abSq, 0f, 1f);
                    var q = a + e * t;
                    var distSq = math.lengthsq(p - q);
                    if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        closest = q;
                    }
                }
                a = b;
            }
            return math.sqrt(minDistSq);
        }

        static float ShadowAngle(float2 v)
        {
            var a = math.atan2(v.y, v.x);
            return a < 0f ? a + math.PI : a;
        }

        static void SortByAngle(ref float2 a, ref float2 b)
        {
            if (ShadowAngle(a) > ShadowAngle(b))
            {
                var t = a;
                a = b;
                b = t;
            }
        }

        static bool GeneratorAxisAligned(float2 e)
        {
            return math.abs(e.x) < 1e-6f || math.abs(e.y) < 1e-6f;
        }

        static bool ShadowAxisAligned(in Leaf box)
        {
            return GeneratorAxisAligned(box.E0) && GeneratorAxisAligned(box.E1) && GeneratorAxisAligned(box.E2);
        }

        static bool BoxBox(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            if (ShadowAxisAligned(a) && ShadowAxisAligned(b))
            {
                return AxisAlignedBoxes(a, b, out normal, out depth);
            }
            return ShadowSeparatingAxis(a, b, out normal, out depth);
        }

        static float2 ShadowHalfXz(in Leaf box)
        {
            return new float2(
                math.abs(box.E0.x) + math.abs(box.E1.x) + math.abs(box.E2.x),
                math.abs(box.E0.y) + math.abs(box.E1.y) + math.abs(box.E2.y));
        }

        static bool AxisAlignedBoxes(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var ha = ShadowHalfXz(a);
            var hb = ShadowHalfXz(b);
            var d = a.ShadowCenter - b.ShadowCenter;
            var overlap = ha + hb - math.abs(d);
            if (math.any(overlap < 0f)) return false;
            int axis = 0;
            var minO = overlap.x;
            if (overlap.y < minO) { minO = overlap.y; axis = 1; }
            normal = new float3(
                axis == 0 ? math.sign(d.x) : 0f,
                0f,
                axis == 1 ? math.sign(d.y) : 0f);
            if (math.all(normal == float3.zero)) normal = new float3(0f, 0f, 1f);
            depth = minO;
            return true;
        }

        static bool ShadowSeparatingAxis(in Leaf a, in Leaf b, out float3 normal, out float depth)
        {
            normal = float3.zero;
            depth = 0f;
            var la = a;
            var lb = b;
            var d = lb.ShadowCenter - la.ShadowCenter;
            var minOverlap = float.MaxValue;
            var bestAxis = float2.zero;
            var testedAny = false;

            bool TestAxis(float2 e)
            {
                var length = math.length(e);
                if (length < 1e-6f) return true;
                testedAny = true;
                var axis = e / length;
                var dist = math.abs(math.dot(d, axis));
                var overlap = ShadowHalf(la, axis) + ShadowHalf(lb, axis) - dist;
                if (overlap <= 0f) return false;
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    bestAxis = axis;
                }
                return true;
            }

            if (!TestAxis(new float2(-a.E0.y, a.E0.x))) return false;
            if (!TestAxis(new float2(-a.E1.y, a.E1.x))) return false;
            if (!TestAxis(new float2(-a.E2.y, a.E2.x))) return false;
            if (!TestAxis(new float2(-b.E0.y, b.E0.x))) return false;
            if (!TestAxis(new float2(-b.E1.y, b.E1.x))) return false;
            if (!TestAxis(new float2(-b.E2.y, b.E2.x))) return false;

            if (!testedAny)
            {
                normal = new float3(0f, 0f, 1f);
                depth = 0f;
                return true;
            }
            var sign = math.dot(bestAxis, d) > 0f ? -1f : 1f;
            normal = new float3(bestAxis.x * sign, 0f, bestAxis.y * sign);
            depth = minOverlap;
            return true;
        }

        internal static Aabb LeafAabb(in Leaf leaf)
        {
            if (leaf.Type == ColliderType.Sphere)
            {
                return new Aabb
                {
                    Min = new float3(leaf.ShadowCenter.x - leaf.Radius, 0f, leaf.ShadowCenter.y - leaf.Radius),
                    Max = new float3(leaf.ShadowCenter.x + leaf.Radius, 0f, leaf.ShadowCenter.y + leaf.Radius)
                };
            }
            var half = ShadowHalfXz(leaf);
            return new Aabb
            {
                Min = new float3(leaf.ShadowCenter.x - half.x, 0f, leaf.ShadowCenter.y - half.y),
                Max = new float3(leaf.ShadowCenter.x + half.x, 0f, leaf.ShadowCenter.y + half.y)
            };
        }

        internal static float PointLeafDistance(in Leaf leaf, float3 point, out float3 surfaceNormal, out float3 closestPoint)
        {
            surfaceNormal = new float3(0f, 0f, 1f);
            closestPoint = point;
            if (leaf.Type == ColliderType.Sphere)
            {
                var delta = point.xz - leaf.ShadowCenter;
                var dist = math.length(delta);
                if (dist > 1e-9f)
                {
                    surfaceNormal = new float3(delta.x / dist, 0f, delta.y / dist);
                    closestPoint = new float3(
                        leaf.ShadowCenter.x + surfaceNormal.x * leaf.Radius,
                        0f,
                        leaf.ShadowCenter.y + surfaceNormal.z * leaf.Radius);
                }
                return dist - leaf.Radius;
            }
            var c = point.xz - leaf.ShadowCenter;
            CircleShadow(leaf, c, 0f, out surfaceNormal, out var depth, out _, out var closestXz);
            closestPoint = new float3(closestXz.x, 0f, closestXz.y);
            return -depth;
        }

        internal static float LeafPairDistance(in Leaf a, in Leaf b, out float3 normalAToB)
        {
            normalAToB = new float3(0f, 0f, 1f);
            var delta = b.ShadowCenter - a.ShadowCenter;
            if (a.Type == ColliderType.Sphere && b.Type == ColliderType.Sphere)
            {
                var dist = math.length(delta);
                if (dist > 1e-9f) normalAToB = new float3(delta.x / dist, 0f, delta.y / dist);
                return dist - a.Radius - b.Radius;
            }
            if (a.Type == ColliderType.Sphere || b.Type == ColliderType.Sphere)
            {
                var sphere = a.Type == ColliderType.Sphere ? a : b;
                var box = a.Type == ColliderType.Sphere ? b : a;
                var spherePoint = new float3(sphere.ShadowCenter.x, 0f, sphere.ShadowCenter.y);
                var dist = PointLeafDistance(box, spherePoint, out var boxNormal, out _);
                normalAToB = a.Type == ColliderType.Sphere ? -boxNormal : boxNormal;
                return dist - sphere.Radius;
            }

            var bestGap = float.MinValue;
            var bestAxis = float2.zero;
            var minOverlap = float.MaxValue;
            var testedAny = false;
            var separated = false;
            var la = a;
            var lb = b;

            void TestAxis(float2 e)
            {
                var length = math.length(e);
                if (length < 1e-6f) return;
                testedAny = true;
                var axis = e / length;
                var gap = math.abs(math.dot(delta, axis)) - ShadowHalf(la, axis) - ShadowHalf(lb, axis);
                if (gap > 0f)
                {
                    separated = true;
                    if (gap > bestGap)
                    {
                        bestGap = gap;
                        bestAxis = axis;
                    }
                }
                else
                {
                    var overlap = -gap;
                    if (overlap < minOverlap) minOverlap = overlap;
                }
            }

            TestAxis(new float2(-a.E0.y, a.E0.x));
            TestAxis(new float2(-a.E1.y, a.E1.x));
            TestAxis(new float2(-a.E2.y, a.E2.x));
            TestAxis(new float2(-b.E0.y, b.E0.x));
            TestAxis(new float2(-b.E1.y, b.E1.x));
            TestAxis(new float2(-b.E2.y, b.E2.x));
            if (math.lengthsq(delta) > 1e-12f)
            {
                TestAxis(delta);
            }

            if (!testedAny)
            {
                var dist = math.length(delta);
                if (dist > 1e-9f) normalAToB = new float3(delta.x / dist, 0f, delta.y / dist);
                return dist;
            }
            if (separated)
            {
                var sign = math.dot(bestAxis, delta) > 0f ? 1f : -1f;
                normalAToB = new float3(bestAxis.x * sign, 0f, bestAxis.y * sign);
                return bestGap;
            }
            return -minOverlap;
        }
    }
}
