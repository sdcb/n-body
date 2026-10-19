using System.Numerics;

namespace Sdcb.NBody.App.Simulation;

/// <summary>Fixed-capacity ring of trail samples for one body.</summary>
internal sealed class TrailRing(int capacity)
{
    private readonly Vector3[] _pos = new Vector3[capacity];
    private readonly double[] _time = new double[capacity];
    private int _start, _count;

    public double Duration { get; set; } = 10;
    public double LastSampleTime { get; private set; } = double.NegativeInfinity;
    public int Count => _count;
    public int Capacity => _pos.Length;
    public double Interval => Duration / Capacity;

    public void Clear()
    {
        _start = 0;
        _count = 0;
        LastSampleTime = double.NegativeInfinity;
    }

    public void Add(Vector3 p, double t)
    {
        int idx = (_start + _count) % _pos.Length;
        if (_count == _pos.Length)
        {
            _start = (_start + 1) % _pos.Length;
        }
        else
        {
            ++_count;
        }
        _pos[idx] = p;
        _time[idx] = t;
        LastSampleTime = t;
    }

    public void MaybeAdd(Vector3 p, double t)
    {
        if (t - LastSampleTime >= Interval || t < LastSampleTime) Add(p, t);
    }

    public (Vector3 Pos, double Time) this[int i]
    {
        get
        {
            int idx = (_start + i) % _pos.Length;
            return (_pos[idx], _time[idx]);
        }
    }
}
