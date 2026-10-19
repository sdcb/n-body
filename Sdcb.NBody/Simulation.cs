using Sdcb.NBody.Scenarios;

namespace Sdcb.NBody;

public readonly record struct Conserved(double Kinetic, double Potential, double Px, double Py, double Pz, double Lx, double Ly, double Lz)
{
    public double Total => Kinetic + Potential;
}

public static class Diagnostics
{
    public static Conserved Measure(BodySet b, ForceModel f)
    {
        double ke = 0, px = 0, py = 0, pz = 0, lx = 0, ly = 0, lz = 0;
        for (int i = 0; i < b.Count; ++i)
        {
            double m = b.Mass[i];
            double vx = b.Vx[i], vy = b.Vy[i], vz = b.Vz[i];
            double x = b.Px[i], y = b.Py[i], z = b.Pz[i];
            ke += 0.5 * m * (vx * vx + vy * vy + vz * vz);
            px += m * vx; py += m * vy; pz += m * vz;
            lx += m * (y * vz - z * vy);
            ly += m * (z * vx - x * vz);
            lz += m * (x * vy - y * vx);
        }
        return new Conserved(ke, f.PotentialEnergy(b), px, py, pz, lx, ly, lz);
    }
}

/// <summary>A CPU simulation of one <see cref="BodySet"/>.</summary>
public sealed class Simulation
{
    private readonly BodySet _initial;
    private IIntegrator _integrator;

    public Simulation(ScenarioSetup setup)
    {
        Setup = setup;
        _initial = setup.Bodies.Clone();
        Bodies = setup.Bodies.Clone();
        Force = new ForceModel(setup.G, setup.Softening);
        _integrator = Integrators.Create(setup.Integrator);
        Dt = setup.Dt;
        MergeOnCollision = setup.MergeOnCollision;
        Initial = Current = Diagnostics.Measure(Bodies, Force);
        _diagCount = Bodies.Count;
    }

    public ScenarioSetup Setup { get; }
    public BodySet Bodies { get; }
    public ForceModel Force { get; }
    public double Time { get; private set; }
    public double Dt { get; set; }
    public bool MergeOnCollision { get; set; }
    public Conserved Initial { get; private set; }
    public Conserved Current { get; private set; }
    public double RelativeEnergyError { get; private set; }
    public event Action<int>? BodyRemoved;

    public IIntegrator Integrator
    {
        get => _integrator;
        set
        {
            ArgumentNullException.ThrowIfNull(value);
            value.Reset();
            _integrator = value;
        }
    }

    private int _diagCount;

    public void RefreshDiagnostics()
    {
        Current = Diagnostics.Measure(Bodies, Force);
        if (_diagCount != Bodies.Count)
        {
            Initial = Current;
            _diagCount = Bodies.Count;
        }
        double e0 = Initial.Total;
        RelativeEnergyError = Math.Abs(e0) > 1e-300 ? Math.Abs((Current.Total - e0) / e0) : Math.Abs(Current.Total - e0);
    }

    public void StepOnce()
    {
        Time += _integrator.Step(Bodies, Force, Dt);
        if (MergeOnCollision) Merge();
    }

    public void Advance(double simTime)
    {
        double target = Time + simTime;
        while (Time < target - 1e-15 * Math.Max(1, Math.Abs(target)))
        {
            double h = Math.Min(Dt, target - Time);
            double done = _integrator.Step(Bodies, Force, h);
            if (done <= 0) break;
            Time += done;
            if (MergeOnCollision) Merge();
        }
    }

    public int AddBody(double px, double py, double pz, double vx, double vy, double vz, double mass,
        BodyKind kind = BodyKind.Star, float radius = 0.05f, Rgb? color = null)
    {
        int i = Bodies.Add(px, py, pz, vx, vy, vz, mass, kind, radius, color);
        _integrator.Reset();
        _diagCount = -2; // force a new energy baseline
        Initial = Current = Diagnostics.Measure(Bodies, Force);
        _diagCount = Bodies.Count;
        RelativeEnergyError = 0;
        return i;
    }

    public void Reset()
    {
        Bodies.CopyFrom(_initial);
        Time = 0;
        Dt = Setup.Dt;
        _integrator.Reset();
        Initial = Current = Diagnostics.Measure(Bodies, Force);
        _diagCount = Bodies.Count;
        RelativeEnergyError = 0;
    }

    /// <summary>Perfectly inelastic merge of overlapping bodies (conserves mass and momentum).</summary>
    private void Merge()
    {
        BodySet b = Bodies;
        bool merged = false;
        for (int i = 0; i < b.Count; ++i)
        {
            for (int j = i + 1; j < b.Count; ++j)
            {
                double dx = b.Px[j] - b.Px[i], dy = b.Py[j] - b.Py[i], dz = b.Pz[j] - b.Pz[i];
                double lim = (b.Radius[i] + b.Radius[j]) * 0.5;
                if (dx * dx + dy * dy + dz * dz >= lim * lim) continue;

                int keep = b.Mass[i] >= b.Mass[j] ? i : j, gone = keep == i ? j : i;
                double m1 = b.Mass[keep], m2 = b.Mass[gone], m = m1 + m2;
                b.Px[keep] = (b.Px[keep] * m1 + b.Px[gone] * m2) / m;
                b.Py[keep] = (b.Py[keep] * m1 + b.Py[gone] * m2) / m;
                b.Pz[keep] = (b.Pz[keep] * m1 + b.Pz[gone] * m2) / m;
                b.Vx[keep] = (b.Vx[keep] * m1 + b.Vx[gone] * m2) / m;
                b.Vy[keep] = (b.Vy[keep] * m1 + b.Vy[gone] * m2) / m;
                b.Vz[keep] = (b.Vz[keep] * m1 + b.Vz[gone] * m2) / m;
                b.Mass[keep] = m;
                b.Radius[keep] = (float)Math.Cbrt(Math.Pow(b.Radius[keep], 3) + Math.Pow(b.Radius[gone], 3));
                b.RemoveAt(gone);
                BodyRemoved?.Invoke(gone);
                merged = true;
                if (gone == i) { --i; break; }
                --j;
            }
        }
        if (merged)
        {
            _integrator.Reset();
            _diagCount = -2;
            RefreshDiagnostics();
        }
    }
}
