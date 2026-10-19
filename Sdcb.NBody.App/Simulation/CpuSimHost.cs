using System.Diagnostics;
using System.Numerics;
using Sdcb.NBody.Scenarios;
using Sim = Sdcb.NBody.Simulation;

namespace Sdcb.NBody.App.Simulation;

/// <summary>
/// Runs a CPU simulation (plus optional chaos twin) on a worker thread, pipelined one frame
/// ahead of rendering. The main thread only touches state between <see cref="Wait"/> and <see cref="Kick"/>.
/// </summary>
internal sealed class CpuSimHost : IDisposable
{
    public Sim Main { get; }
    public Sim? Twin { get; }
    public List<TrailRing> Trails { get; } = [];
    public List<TrailRing> TwinTrails { get; } = [];
    public ScenarioSetup Setup { get; }
    public bool Throttled { get; private set; }
    public double LastStepMs { get; private set; }
    public float[] EnergyHistory { get; } = new float[240];
    public int EnergyHistoryCount { get; private set; }

    private readonly int _trailCapacity;
    private Task? _job;
    private int _diagCounter;
    private const double BudgetMs = 10.0;

    public CpuSimHost(ScenarioSetup setup)
    {
        Setup = setup;
        Main = new Sim(setup);
        if (setup.Twin != null)
        {
            Twin = new Sim(setup with { Bodies = setup.Twin, Twin = null });
        }
        Main.BodyRemoved += idx => { if (idx < Trails.Count) Trails.RemoveAt(idx); };
        if (Twin != null) Twin.BodyRemoved += idx => { if (idx < TwinTrails.Count) TwinTrails.RemoveAt(idx); };

        int n = Main.Bodies.Count + (Twin?.Bodies.Count ?? 0);
        _trailCapacity = Math.Clamp(120_000 / Math.Max(1, n), 96, 2000);
        SyncTrails();
        Main.RefreshDiagnostics();
        Twin?.RefreshDiagnostics();
    }

    /// <summary>Make sure there is one trail per body and trail durations follow the local orbital period.</summary>
    public void SyncTrails()
    {
        Sync(Main, Trails);
        if (Twin != null) Sync(Twin, TwinTrails);
    }

    private void Sync(Sim sim, List<TrailRing> trails)
    {
        BodySet b = sim.Bodies;
        while (trails.Count < b.Count) trails.Add(new TrailRing(_trailCapacity));
        while (trails.Count > b.Count) trails.RemoveAt(trails.Count - 1);
        for (int i = 0; i < b.Count; ++i)
        {
            trails[i].Duration = EstimateTrailDuration(sim, i);
            if (trails[i].Count == 0) trails[i].Add(Pos(b, i), sim.Time);
        }
    }

    private double EstimateTrailDuration(Sim sim, int i)
    {
        double global = Setup.TrailDuration;
        BodySet b = sim.Bodies;
        int best = -1;
        double bestPull = 0;
        for (int j = 0; j < b.Count; ++j)
        {
            if (j == i) continue;
            double dx = b.Px[j] - b.Px[i], dy = b.Py[j] - b.Py[i], dz = b.Pz[j] - b.Pz[i];
            double r2 = dx * dx + dy * dy + dz * dz + 1e-12;
            double pull = b.Mass[j] / r2;
            if (pull > bestPull) { bestPull = pull; best = j; }
        }
        if (best < 0 || b.Mass[best] < 10 * b.Mass[i]) return global;
        double rx = b.Px[best] - b.Px[i], ry = b.Py[best] - b.Py[i], rz = b.Pz[best] - b.Pz[i];
        double r = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        double gm = sim.Force.G * (b.Mass[best] + b.Mass[i]);
        double period = 2 * Math.PI * Math.Sqrt(r * r * r / gm);
        return Math.Min(global, period * 0.92);
    }

    private static Vector3 Pos(BodySet b, int i) => new((float)b.Px[i], (float)b.Py[i], (float)b.Pz[i]);

    public void Kick(double simDt)
    {
        if (simDt <= 0) return;
        _job = Task.Run(() => Run(simDt));
    }

    public void Wait()
    {
        Task? j = _job;
        _job = null;
        j?.GetAwaiter().GetResult();
    }

    private void Run(double simDt)
    {
        Stopwatch sw = Stopwatch.StartNew();
        double target = Main.Time + simDt;
        bool throttled = false;
        while (Main.Time < target - 1e-12)
        {
            if (sw.Elapsed.TotalMilliseconds > BudgetMs) { throttled = true; break; }
            double rem = target - Main.Time;
            if (rem < Main.Dt)
            {
                Main.Advance(rem);
                Twin?.Advance(rem);
            }
            else
            {
                Main.StepOnce();
                Twin?.StepOnce();
            }
            SampleTrails(Main, Trails);
            if (Twin != null) SampleTrails(Twin, TwinTrails);
        }
        Throttled = throttled;

        if (++_diagCounter % 4 == 0)
        {
            Main.RefreshDiagnostics();
            Twin?.RefreshDiagnostics();
            double err = Main.RelativeEnergyError;
            float v = (float)Math.Log10(Math.Max(err, 1e-16));
            if (EnergyHistoryCount < EnergyHistory.Length)
            {
                EnergyHistory[EnergyHistoryCount++] = v;
            }
            else
            {
                Array.Copy(EnergyHistory, 1, EnergyHistory, 0, EnergyHistory.Length - 1);
                EnergyHistory[^1] = v;
            }
        }
        LastStepMs = sw.Elapsed.TotalMilliseconds;
    }

    private static void SampleTrails(Sim sim, List<TrailRing> trails)
    {
        BodySet b = sim.Bodies;
        int n = Math.Min(b.Count, trails.Count);
        double t = sim.Time;
        for (int i = 0; i < n; ++i)
        {
            trails[i].MaybeAdd(Pos(b, i), t);
        }
    }

    public void ResetSimulation()
    {
        Wait();
        Main.Reset();
        Twin?.Reset();
        foreach (TrailRing t in Trails) t.Clear();
        foreach (TrailRing t in TwinTrails) t.Clear();
        EnergyHistoryCount = 0;
        SyncTrails();
        Main.RefreshDiagnostics();
    }

    public void Dispose() => Wait();
}
