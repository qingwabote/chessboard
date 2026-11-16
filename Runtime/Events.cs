using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics
{
    public readonly struct TriggerEvent
    {
        public readonly Entity EntityA;
        public readonly Entity EntityB;
        public readonly int BodyIndexA;
        public readonly int BodyIndexB;

        public TriggerEvent(Entity entityA, Entity entityB, int bodyIndexA, int bodyIndexB)
        {
            EntityA = entityA;
            EntityB = entityB;
            BodyIndexA = bodyIndexA;
            BodyIndexB = bodyIndexB;
        }
    }

    public readonly struct CollisionEvent
    {
        public readonly Entity EntityA;
        public readonly Entity EntityB;
        public readonly int BodyIndexA;
        public readonly int BodyIndexB;
        public readonly float3 Normal;

        public CollisionEvent(Entity entityA, Entity entityB, int bodyIndexA, int bodyIndexB, float3 normal)
        {
            EntityA = entityA;
            EntityB = entityB;
            BodyIndexA = bodyIndexA;
            BodyIndexB = bodyIndexB;
            Normal = normal;
        }
    }

    public struct TriggerEvents
    {
#pragma warning disable 649
        NativeList<TriggerEvent> m_Events;
#pragma warning restore 649

        public NativeArray<TriggerEvent>.Enumerator GetEnumerator() => m_Events.GetEnumerator();
    }

    public struct CollisionEvents
    {
#pragma warning disable 649
        NativeList<CollisionEvent> m_Events;
#pragma warning restore 649

        public NativeArray<CollisionEvent>.Enumerator GetEnumerator() => m_Events.GetEnumerator();
    }

    public struct Simulation
    {
#pragma warning disable 649
        NativeList<TriggerEvent> m_TriggerEvents;
        NativeList<CollisionEvent> m_CollisionEvents;
#pragma warning restore 649

        public TriggerEvents TriggerEvents => UnsafeUtility.As<NativeList<TriggerEvent>, TriggerEvents>(ref m_TriggerEvents);
        public CollisionEvents CollisionEvents => UnsafeUtility.As<NativeList<CollisionEvent>, CollisionEvents>(ref m_CollisionEvents);
    }

    public interface ITriggerEventsJobBase
    {
        void Execute(TriggerEvent triggerEvent);
    }

    public interface ITriggerEventsJob : ITriggerEventsJobBase
    {
    }

    public interface ICollisionEventsJobBase
    {
        void Execute(CollisionEvent collisionEvent);
    }

    public interface ICollisionEventsJob : ICollisionEventsJobBase
    {
    }

    public static class ITriggerEventJobExtensions
    {
        public static JobHandle Schedule<T>(
            this T jobData, SimulationSingleton simulationSingleton, JobHandle inputDeps)
            where T : struct, ITriggerEventsJobBase
        {
            inputDeps.Complete();
            var events = simulationSingleton.TriggerEvents;
            for (int i = 0; i < events.Length; i++)
            {
                jobData.Execute(events[i]);
            }
            return default;
        }
    }

    public static class ICollisionEventJobExtensions
    {
        public static JobHandle Schedule<T>(
            this T jobData, SimulationSingleton simulationSingleton, JobHandle inputDeps)
            where T : struct, ICollisionEventsJobBase
        {
            inputDeps.Complete();
            var events = simulationSingleton.CollisionEvents;
            for (int i = 0; i < events.Length; i++)
            {
                jobData.Execute(events[i]);
            }
            return default;
        }
    }
}
