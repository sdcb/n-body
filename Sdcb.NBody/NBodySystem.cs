using Sdcb.NBody.Solvers;
using System.Collections.Concurrent;

namespace Sdcb.NBody;

public class NBodySystem : IDisposable
{
    public readonly double _dt;
    public readonly Body[] _bodies;
    private readonly Solver _solver;

    public NBodySystem(SystemDef def)
    {
        _dt = def.Dt;
        _bodies = def.ToBodies();
        _solver = new Ode45(_bodies, def.Dt, maxDt: def.Dt);
    }

    public void Dispose()
    {
        for (int i = 0; i < _bodies.Length; ++i)
        {
            _bodies[i].Dispose();
        }
    }

    public double Elapsed;
    public bool Crashed => _bodies.Any(x => x.State.Crashed);

    public void Step()
    {
        BodyState[] oldStates = new BodyState[_bodies.Length];
        for (int i = 0; i < _bodies.Length; ++i)
        {
            oldStates[i] = _bodies[i].State;
        }

        double dt = _solver.Step(oldStates);
        Elapsed += dt;
    }

    public IEnumerable<SystemSnapshot> AutoStep(int boundedCapacity = 512, CancellationToken cancellationToken = default)
    {
        BlockingCollection<SystemSnapshot> q = new(boundedCapacity: boundedCapacity);
        Task _ = Task.Factory.StartNew(() =>
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                Step();
                try
                {
                    q.Add(GetSnapshot(), cancellationToken);
                }
                catch (TaskCanceledException)
                {
                    break;
                }
            }
            q.CompleteAdding();
        }, TaskCreationOptions.LongRunning);
        return q.GetConsumingEnumerable(cancellationToken);
    }

    public SystemSnapshot GetSnapshot() => new((float)Elapsed, [.. _bodies.Select(x => x.GetSnapshot())]);
}
