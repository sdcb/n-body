using Sdcb.NBody.Scenarios;

namespace Sdcb.NBody.Tests;

public class PhysicsTests
{
    private static BodySet TwoBodyCircular()
    {
        BodySet b = new();
        b.Add(0, 0, 0, 0, 0, 0, 1.0);
        b.Add(1, 0, 0, 0, 1, 0, 1e-6);
        b.ShiftToCenterOfMassFrame();
        return b;
    }

    [Theory]
    [InlineData(IntegratorKind.Leapfrog, 1e-6)]
    [InlineData(IntegratorKind.Yoshida4, 1e-9)]
    [InlineData(IntegratorKind.RK4, 1e-7)]
    [InlineData(IntegratorKind.DormandPrince45, 1e-8)]
    public void CircularOrbit_ConservesEnergy(IntegratorKind kind, double tolerance)
    {
        BodySet b = TwoBodyCircular();
        ForceModel f = new();
        double e0 = Diagnostics.Measure(b, f).Total;
        IIntegrator integrator = Integrators.Create(kind);
        double dt = 2 * Math.PI / 1000;
        for (int i = 0; i < 10_000; ++i) integrator.Step(b, f, dt);   // 10 orbits
        double e1 = Diagnostics.Measure(b, f).Total;
        Assert.True(Math.Abs((e1 - e0) / e0) < tolerance, $"{kind}: relative energy error {Math.Abs((e1 - e0) / e0):e2}");
    }

    [Fact]
    public void RandomCluster_ConservesMomentum()
    {
        Random rng = new(1);
        BodySet b = new();
        for (int i = 0; i < 50; ++i)
        {
            b.Add(rng.NextDouble() * 2 - 1, rng.NextDouble() * 2 - 1, rng.NextDouble() * 2 - 1,
                rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, 0.5 + rng.NextDouble());
        }
        ForceModel f = new(1, 0.05);
        Conserved c0 = Diagnostics.Measure(b, f);
        IIntegrator integrator = Integrators.Create(IntegratorKind.Leapfrog);
        for (int i = 0; i < 500; ++i) integrator.Step(b, f, 1e-3);
        Conserved c1 = Diagnostics.Measure(b, f);
        Assert.InRange(c1.Px - c0.Px, -1e-10, 1e-10);
        Assert.InRange(c1.Py - c0.Py, -1e-10, 1e-10);
        Assert.InRange(c1.Pz - c0.Pz, -1e-10, 1e-10);
    }

    [Theory]
    [InlineData(IntegratorKind.DormandPrince45)]
    [InlineData(IntegratorKind.Yoshida4)]
    public void FigureEight_ReturnsAfterOnePeriod(IntegratorKind kind)
    {
        ScenarioSetup s = ScenarioRegistry.Get("figure-8").Create();
        BodySet b = s.Bodies.Clone();
        BodySet start = b.Clone();
        ForceModel f = new();
        IIntegrator integrator = Integrators.Create(kind);
        double period = s.Period!.Value;
        int steps = 20_000;
        for (int i = 0; i < steps; ++i) integrator.Step(b, f, period / steps);
        for (int i = 0; i < 3; ++i)
        {
            double d = Math.Sqrt(Math.Pow(b.Px[i] - start.Px[i], 2) + Math.Pow(b.Py[i] - start.Py[i], 2));
            Assert.True(d < 1e-3, $"body {i} off by {d}");
        }
    }

    [Fact]
    public void SimdForce_MatchesNaiveReference()
    {
        Random rng = new(2);
        BodySet b = new();
        for (int i = 0; i < 300; ++i)
        {
            b.Add(rng.NextGaussian(), rng.NextGaussian(), rng.NextGaussian(), 0, 0, 0, rng.NextDouble());
        }
        ForceModel f = new(1.3, 0.01);
        double[] ax = new double[300], ay = new double[300], az = new double[300];
        f.ComputeAccelerations(b, ax, ay, az);
        for (int i = 0; i < 300; ++i)
        {
            double rx = 0, ry = 0, rz = 0;
            for (int j = 0; j < 300; ++j)
            {
                if (i == j) continue;
                double dx = b.Px[j] - b.Px[i], dy = b.Py[j] - b.Py[i], dz = b.Pz[j] - b.Pz[i];
                double r2 = dx * dx + dy * dy + dz * dz + 1e-4;
                double s = 1.3 * b.Mass[j] / (r2 * Math.Sqrt(r2));
                rx += dx * s; ry += dy * s; rz += dz * s;
            }
            Assert.Equal(rx, ax[i], 1e-10);
            Assert.Equal(ry, ay[i], 1e-10);
            Assert.Equal(rz, az[i], 1e-10);
        }
    }

    [Fact]
    public void SolarSystem_EarthReturnsAfterOneYear()
    {
        ScenarioSetup s = ScenarioRegistry.Get("inner-solar-system").Create();
        Simulation sim = new(s);
        int earth = Array.IndexOf(sim.Bodies.Name, "Earth");
        int sun = Array.IndexOf(sim.Bodies.Name, "Sun");
        double x0 = sim.Bodies.Px[earth] - sim.Bodies.Px[sun], y0 = sim.Bodies.Py[earth] - sim.Bodies.Py[sun];
        sim.Advance(1.0);
        double x1 = sim.Bodies.Px[earth] - sim.Bodies.Px[sun], y1 = sim.Bodies.Py[earth] - sim.Bodies.Py[sun];
        double d = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        Assert.True(d < 0.02, $"Earth is {d:F4} AU from its starting point");
    }

    [Fact]
    public void DormandPrince_AdvancesExactlyDt()
    {
        BodySet b = TwoBodyCircular();
        IIntegrator dp = Integrators.Create(IntegratorKind.DormandPrince45);
        Assert.Equal(0.37, dp.Step(b, new ForceModel(), 0.37), 15);
        Simulation sim = new(new ScenarioSetup { Bodies = TwoBodyCircular(), Integrator = IntegratorKind.DormandPrince45, Dt = 0.01 });
        sim.Advance(1.234);
        Assert.Equal(1.234, sim.Time, 12);
    }

    [Fact]
    public void Merge_ConservesMassAndMomentum()
    {
        BodySet b = new();
        b.Add(-0.01, 0, 0, 1, 0, 0, 1, radius: 0.1f);
        b.Add(0.01, 0, 0, -0.5, 0.2, 0, 2, radius: 0.1f);
        Simulation sim = new(new ScenarioSetup { Bodies = b, MergeOnCollision = true, Dt = 1e-4 });
        int removed = -1;
        sim.BodyRemoved += i => removed = i;
        sim.StepOnce();
        Assert.Equal(1, sim.Bodies.Count);
        Assert.Equal(0, removed);
        Assert.Equal(3, sim.Bodies.Mass[0], 12);
        Assert.Equal(0.0, sim.Bodies.Mass[0] * sim.Bodies.Vx[0], 9);
        Assert.Equal(0.4, sim.Bodies.Mass[0] * sim.Bodies.Vy[0], 9);
    }

    [Fact]
    public void GalaxyDisk_StaysBoundForOneRotation()
    {
        BodySet b = new();
        Random rng = new(3);
        GalaxyScenarios.DiskParams p = new(0.15, 1.0, 0.5, 2.6, 0, new Rgb(1, 1, 1), new Rgb(1, 1, 1));
        GalaxyScenarios.AddDiskGalaxy(b, 1500, p, 0.035, rng, (0, 0, 0), (0, 0, 0), 0, 0, 1);
        double r0 = MedianRadius(b);
        ForceModel f = new(1, 0.035);
        IIntegrator lf = Integrators.Create(IntegratorKind.Leapfrog);
        for (int i = 0; i < 400; ++i) lf.Step(b, f, 0.01);
        double r1 = MedianRadius(b);
        Assert.InRange(r1 / r0, 0.7, 1.4);
    }

    private static double MedianRadius(BodySet b)
    {
        double[] r = new double[b.Count];
        for (int i = 0; i < b.Count; ++i) r[i] = Math.Sqrt(b.Px[i] * b.Px[i] + b.Py[i] * b.Py[i] + b.Pz[i] * b.Pz[i]);
        Array.Sort(r);
        return r[r.Length / 2];
    }
}

public class ScenarioTests
{
    public static TheoryData<string> Ids()
    {
        TheoryData<string> d = [];
        foreach (Scenario s in ScenarioRegistry.All) d.Add(s.Info.Id);
        return d;
    }

    [Fact]
    public void Ids_AreUnique() =>
        Assert.Equal(ScenarioRegistry.All.Count, ScenarioRegistry.All.Select(s => s.Info.Id).Distinct().Count());

    [Theory]
    [MemberData(nameof(Ids))]
    public void Scenario_ConstructsWithFiniteState(string id)
    {
        Scenario sc = ScenarioRegistry.Get(id);
        ScenarioSetup s = sc.Create(4096);
        Assert.True(s.Bodies.Count > 0);
        Assert.True(s.ViewRadius > 0);
        Assert.True(s.Dt > 0);
        Assert.False(string.IsNullOrWhiteSpace(sc.Info.NameZh));
        Assert.False(string.IsNullOrWhiteSpace(sc.Info.NameEn));
        BodySet b = s.Bodies;
        for (int i = 0; i < b.Count; ++i)
        {
            Assert.True(double.IsFinite(b.Px[i]) && double.IsFinite(b.Py[i]) && double.IsFinite(b.Pz[i]));
            Assert.True(double.IsFinite(b.Vx[i]) && double.IsFinite(b.Vy[i]) && double.IsFinite(b.Vz[i]));
            Assert.True(b.Mass[i] > 0);
        }
        if (s.GpuSimulated) Assert.True(b.Count <= 4096);
    }
}
