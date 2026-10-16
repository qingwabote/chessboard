using Unity.Entities;

namespace Unity.Physics.Systems
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public partial class PhysicsSystemGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateBefore(typeof(PhysicsInitializeGroup))]
    public partial class BeforePhysicsSystemGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateBefore(typeof(PhysicsSimulationGroup))]
    public partial class PhysicsInitializeGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsInitializeGroup))]
    [UpdateBefore(typeof(ExportPhysicsWorld))]
    public partial class PhysicsSimulationGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateBefore(typeof(PhysicsCreateContactsGroup))]
    public partial class PhysicsCreateBodyPairsGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateBodyPairsGroup))]
    [UpdateBefore(typeof(PhysicsCreateJacobiansGroup))]
    public partial class PhysicsCreateContactsGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateContactsGroup))]
    [UpdateBefore(typeof(PhysicsSolveAndIntegrateGroup))]
    public partial class PhysicsCreateJacobiansGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateJacobiansGroup))]
    public partial class PhysicsSolveAndIntegrateGroup : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(ExportPhysicsWorld))]
    public partial class AfterPhysicsSystemGroup : ComponentSystemGroup
    {
    }
}
