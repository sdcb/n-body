namespace Sdcb.NBody.Scenarios;

public enum ScenarioCategory { PeriodicThreeBody, Choreography, SolarSystem, Galaxy, Cluster, Chaos, Sandbox }

public sealed record ScenarioInfo(string Id, ScenarioCategory Category, string NameZh, string NameEn, string DescriptionZh, string DescriptionEn);

/// <summary>Analytic Plummer dark-matter halo that follows body <paramref name="BodyIndex"/> (GPU scenes).</summary>
public sealed record Halo(int BodyIndex, double Mass, double Scale)
{
    public double MassWithin(double r) => Mass * r * r * r / Math.Pow(r * r + Scale * Scale, 1.5);
}

public sealed record ScenarioSetup
{
    /// <summary>Static-shape dark-matter halos attached to bodies (GPU scenes only).</summary>
    public IReadOnlyList<Halo> Halos { get; init; } = [];

    /// <summary>Dynamical-friction coefficient applied to halo hosts moving through other halos (lets galaxies merge).</summary>
    public double HaloFriction { get; init; }

    public required BodySet Bodies { get; init; }

    /// <summary>Chaos demos: a second, independently simulated copy.</summary>
    public BodySet? Twin { get; init; }

    public double G { get; init; } = 1.0;
    public double Softening { get; init; }
    public IntegratorKind Integrator { get; init; } = IntegratorKind.Yoshida4;
    public double Dt { get; init; } = 1e-3;

    /// <summary>Suggested simulation-time units per real second.</summary>
    public double TimeScale { get; init; } = 1.0;

    /// <summary>World-space radius that frames the scene.</summary>
    public double ViewRadius { get; init; } = 2.0;

    /// <summary>Simulation-time length of trails (0 = none).</summary>
    public double TrailDuration { get; init; } = 10.0;

    public double? Period { get; init; }

    /// <summary>Massive particle scenes: physics runs on the GPU (leapfrog, softening, G).</summary>
    public bool GpuSimulated { get; init; }

    public bool MergeOnCollision { get; init; }

    public float CameraPitchDegrees { get; init; } = 35f;
}

public abstract class Scenario
{
    public abstract ScenarioInfo Info { get; }

    /// <param name="particleBudget">Max particle count for GPU scenes; ignored by CPU scenes.</param>
    public abstract ScenarioSetup Create(int particleBudget = 65536, int seed = 42);

    public override string ToString() => Info.Id;
}

internal sealed class DelegateScenario(ScenarioInfo info, Func<int, int, ScenarioSetup> factory) : Scenario
{
    public override ScenarioInfo Info => info;
    public override ScenarioSetup Create(int particleBudget = 65536, int seed = 42) => factory(particleBudget, seed);
}

public static class ScenarioRegistry
{
    private static readonly Lazy<IReadOnlyList<Scenario>> s_all = new(Build);

    public static IReadOnlyList<Scenario> All => s_all.Value;

    public static Scenario Get(string id) =>
        All.FirstOrDefault(s => s.Info.Id == id) ?? throw new KeyNotFoundException($"Unknown scenario '{id}'.");

    public static int IndexOf(string id)
    {
        for (int i = 0; i < All.Count; ++i) if (All[i].Info.Id == id) return i;
        return -1;
    }

    private static readonly ScenarioCategory[] s_order =
    [
        ScenarioCategory.Galaxy, ScenarioCategory.PeriodicThreeBody, ScenarioCategory.Choreography, ScenarioCategory.SolarSystem,
        ScenarioCategory.Cluster, ScenarioCategory.Chaos, ScenarioCategory.Sandbox,
    ];

    private static IReadOnlyList<Scenario> Build()
    {
        Scenario[] all =
        [
            .. GalaxyScenarios.All(),
            .. PeriodicThreeBodyScenarios.All(),
            .. ChoreographyScenarios.All(),
            .. SolarSystemScenarios.All(),
            .. ClusterScenarios.All(),
            .. ChaosScenarios.All(),
            .. SandboxScenarios.All(),
        ];
        return [.. all.OrderBy(s => Array.IndexOf(s_order, s.Info.Category))];
    }
}
