using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics
{
    internal struct TriggerEventData
    {
        public Entity EntityA;
        public Entity EntityB;
        public int BodyIndexA;
        public int BodyIndexB;
    }

    public struct TriggerEvent
    {
        internal TriggerEventData Data;

        public Entity EntityA => Data.EntityA;
        public Entity EntityB => Data.EntityB;
        public int BodyIndexA => Data.BodyIndexA;
        public int BodyIndexB => Data.BodyIndexB;
    }

    internal struct CollisionEventData
    {
        public Entity EntityA;
        public Entity EntityB;
        public int BodyIndexA;
        public int BodyIndexB;
        public float3 Normal;
    }

    public struct CollisionEvent
    {
        internal CollisionEventData Data;

        public Entity EntityA => Data.EntityA;
        public Entity EntityB => Data.EntityB;
        public int BodyIndexA => Data.BodyIndexA;
        public int BodyIndexB => Data.BodyIndexB;
        public float3 Normal => Data.Normal;
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
                jobData.Execute(new TriggerEvent { Data = events[i] });
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
                jobData.Execute(new CollisionEvent { Data = events[i] });
            }
            return default;
        }
    }
}
