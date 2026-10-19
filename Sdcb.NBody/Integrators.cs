namespace Sdcb.NBody;

public enum IntegratorKind { Leapfrog, Yoshida4, RK4, DormandPrince45 }

public interface IIntegrator
{
    IntegratorKind Kind { get; }

    /// <summary>Advances <paramref name="bodies"/> by <paramref name="dt"/>; returns the time actually advanced.</summary>
    double Step(BodySet bodies, ForceModel force, double dt);

    /// <summary>Clears cached state (accelerations, adaptive step size).</summary>
    void Reset();
}

public static class Integrators
{
    public static IIntegrator Create(IntegratorKind kind) => kind switch
    {
        IntegratorKind.Leapfrog => new LeapfrogIntegrator(),
        IntegratorKind.Yoshida4 => new Yoshida4Integrator(),
        IntegratorKind.RK4 => new Rk4Integrator(),
        IntegratorKind.DormandPrince45 => new DormandPrince45Integrator(),
        _ => throw new ArgumentOutOfRangeException(nameof(kind)),
    };
}

/// <summary>Shared acceleration scratch with a cache of a(x) for the current positions.</summary>
public abstract class SymplecticIntegratorBase : IIntegrator
{
    protected double[] Ax = [], Ay = [], Az = [];
    private int _cachedVersion = -1;
    private BodySet? _cachedSet;
    private double _cachedX0, _cachedY0, _cachedZ0;
    private int _cachedCount = -1;

    public abstract IntegratorKind Kind { get; }
    public abstract double Step(BodySet bodies, ForceModel force, double dt);

    public virtual void Reset() => _cachedVersion = -1;

    protected void Ensure(int n)
    {
        ForceModel.EnsureLength(ref Ax, n);
        ForceModel.EnsureLength(ref Ay, n);
        ForceModel.EnsureLength(ref Az, n);
    }

    protected void Accel(BodySet b, ForceModel f)
    {
        Ensure(b.Count);
        f.ComputeAccelerations(b, Ax, Ay, Az);
        Remember(b);
    }

    protected void Remember(BodySet b)
    {
        _cachedSet = b;
        _cachedVersion = b.Version;
        _cachedCount = b.Count;
        if (b.Count > 0) { _cachedX0 = b.Px[0]; _cachedY0 = b.Py[0]; _cachedZ0 = b.Pz[0]; }
    }

    protected bool CacheValid(BodySet b) =>
        ReferenceEquals(_cachedSet, b) && _cachedVersion == b.Version && _cachedCount == b.Count &&
        (b.Count == 0 || (_cachedX0 == b.Px[0] && _cachedY0 == b.Py[0] && _cachedZ0 == b.Pz[0])) && Ax.Length >= b.Count;

    protected static void Drift(BodySet b, double h)
    {
        int n = b.Count;
        double[] px = b.Px, py = b.Py, pz = b.Pz, vx = b.Vx, vy = b.Vy, vz = b.Vz;
        for (int i = 0; i < n; ++i) { px[i] += vx[i] * h; py[i] += vy[i] * h; pz[i] += vz[i] * h; }
    }

    protected void Kick(BodySet b, double h)
    {
        int n = b.Count;
        double[] vx = b.Vx, vy = b.Vy, vz = b.Vz;
        for (int i = 0; i < n; ++i) { vx[i] += Ax[i] * h; vy[i] += Ay[i] * h; vz[i] += Az[i] * h; }
    }
}

/// <summary>Kick-drift-kick velocity Verlet (2nd order, symplectic).</summary>
public sealed class LeapfrogIntegrator : SymplecticIntegratorBase
{
    public override IntegratorKind Kind => IntegratorKind.Leapfrog;

    public override double Step(BodySet b, ForceModel f, double dt)
    {
        if (!CacheValid(b)) Accel(b, f);
        Kick(b, dt * 0.5);
        Drift(b, dt);
        Accel(b, f);
        Kick(b, dt * 0.5);
        Remember(b);
        return dt;
    }
}

/// <summary>Yoshida (1990) 4th-order symplectic composition, drift-kick form.</summary>
public sealed class Yoshida4Integrator : SymplecticIntegratorBase
{
    private static readonly double W1 = 1.0 / (2.0 - Math.Cbrt(2.0));
    private static readonly double W0 = -Math.Cbrt(2.0) / (2.0 - Math.Cbrt(2.0));
    private static readonly double C1 = W1 / 2, C2 = (W0 + W1) / 2;

    public override IntegratorKind Kind => IntegratorKind.Yoshida4;

    public override double Step(BodySet b, ForceModel f, double dt)
    {
        Drift(b, C1 * dt);
        Accel(b, f); Kick(b, W1 * dt);
        Drift(b, C2 * dt);
        Accel(b, f); Kick(b, W0 * dt);
        Drift(b, C2 * dt);
        Accel(b, f); Kick(b, W1 * dt);
        Drift(b, C1 * dt);
        return dt;
    }
}

/// <summary>Base for explicit Runge–Kutta methods working on a (pos, vel) state of 6 component arrays.</summary>
public abstract class RungeKuttaBase : IIntegrator
{
    protected double[][] Y0 = NewState(0), Tmp = NewState(0);
    protected int N;

    public abstract IntegratorKind Kind { get; }
    public abstract double Step(BodySet bodies, ForceModel force, double dt);
    public virtual void Reset() { }

    protected static double[][] NewState(int n) => [new double[n], new double[n], new double[n], new double[n], new double[n], new double[n]];

    protected static void EnsureState(ref double[][] s, int n)
    {
        if (s[0].Length < n) s = NewState(Math.Max(n, s[0].Length * 2));
    }

    protected virtual void EnsureScratch(int n)
    {
        N = n;
        EnsureState(ref Y0, n);
        EnsureState(ref Tmp, n);
    }

    protected void Load(BodySet b, double[][] y)
    {
        Array.Copy(b.Px, y[0], N); Array.Copy(b.Py, y[1], N); Array.Copy(b.Pz, y[2], N);
        Array.Copy(b.Vx, y[3], N); Array.Copy(b.Vy, y[4], N); Array.Copy(b.Vz, y[5], N);
    }

    protected void Store(double[][] y, BodySet b)
    {
        Array.Copy(y[0], b.Px, N); Array.Copy(y[1], b.Py, N); Array.Copy(y[2], b.Pz, N);
        Array.Copy(y[3], b.Vx, N); Array.Copy(y[4], b.Vy, N); Array.Copy(y[5], b.Vz, N);
    }

    /// <summary>k = f(y): dpos/dt = vel, dvel/dt = a(pos).</summary>
    protected void Derivative(BodySet b, ForceModel f, double[][] y, double[][] k)
    {
        Array.Copy(y[3], k[0], N); Array.Copy(y[4], k[1], N); Array.Copy(y[5], k[2], N);
        f.ComputeAccelerations(b, y[0], y[1], y[2], k[3], k[4], k[5]);
    }

    /// <summary>dst = y0 + h * Σ c_s k_s</summary>
    protected void Combine(double[][] dst, double[][] y0, double h, ReadOnlySpan<double> c, params ReadOnlySpan<double[][]> ks)
    {
        for (int comp = 0; comp < 6; ++comp)
        {
            double[] d = dst[comp], y = y0[comp];
            for (int i = 0; i < N; ++i)
            {
                double s = 0;
                for (int q = 0; q < ks.Length; ++q)
                {
                    if (c[q] != 0) s += c[q] * ks[q][comp][i];
                }
                d[i] = y[i] + h * s;
            }
        }
    }
}

public sealed class Rk4Integrator : RungeKuttaBase
{
    private double[][] _k1 = NewState(0), _k2 = NewState(0), _k3 = NewState(0), _k4 = NewState(0);

    public override IntegratorKind Kind => IntegratorKind.RK4;

    protected override void EnsureScratch(int n)
    {
        base.EnsureScratch(n);
        EnsureState(ref _k1, n); EnsureState(ref _k2, n); EnsureState(ref _k3, n); EnsureState(ref _k4, n);
    }

    public override double Step(BodySet b, ForceModel f, double dt)
    {
        EnsureScratch(b.Count);
        Load(b, Y0);
        Derivative(b, f, Y0, _k1);
        Combine(Tmp, Y0, dt * 0.5, [1], _k1);
        Derivative(b, f, Tmp, _k2);
        Combine(Tmp, Y0, dt * 0.5, [1], _k2);
        Derivative(b, f, Tmp, _k3);
        Combine(Tmp, Y0, dt, [1], _k3);
        Derivative(b, f, Tmp, _k4);
        Combine(Tmp, Y0, dt / 6.0, [1, 2, 2, 1], _k1, _k2, _k3, _k4);
        Store(Tmp, b);
        return dt;
    }
}

/// <summary>
/// Dormand–Prince 5(4) with adaptive sub-stepping. <see cref="Step"/> always advances exactly dt,
/// taking as many internal steps as the error control demands.
/// </summary>
public sealed class DormandPrince45Integrator : RungeKuttaBase
{
    private const double A21 = 1.0 / 5.0;
    private const double A31 = 3.0 / 40.0, A32 = 9.0 / 40.0;
    private const double A41 = 44.0 / 45.0, A42 = -56.0 / 15.0, A43 = 32.0 / 9.0;
    private const double A51 = 19372.0 / 6561.0, A52 = -25360.0 / 2187.0, A53 = 64448.0 / 6561.0, A54 = -212.0 / 729.0;
    private const double A61 = 9017.0 / 3168.0, A62 = -355.0 / 33.0, A63 = 46732.0 / 5247.0, A64 = 49.0 / 176.0, A65 = -5103.0 / 18656.0;
    private const double B1 = 35.0 / 384.0, B3 = 500.0 / 1113.0, B4 = 125.0 / 192.0, B5 = -2187.0 / 6784.0, B6 = 11.0 / 84.0;
    private const double E1 = 71.0 / 57600.0, E3 = -71.0 / 16695.0, E4 = 71.0 / 1920.0, E5 = -17253.0 / 339200.0, E6 = 22.0 / 525.0, E7 = -1.0 / 40.0;

    private double[][] _k1 = NewState(0), _k2 = NewState(0), _k3 = NewState(0), _k4 = NewState(0), _k5 = NewState(0), _k6 = NewState(0), _k7 = NewState(0), _y1 = NewState(0);
    private double _h;
    private bool _fsalValid;

    public double AbsTolerance { get; set; } = 1e-10;
    public double RelTolerance { get; set; } = 1e-10;
    public int StepsTaken { get; private set; }

    public override IntegratorKind Kind => IntegratorKind.DormandPrince45;

    public override void Reset()
    {
        _h = 0;
        _fsalValid = false;
    }

    protected override void EnsureScratch(int n)
    {
        if (n != N) _fsalValid = false;
        base.EnsureScratch(n);
        EnsureState(ref _k1, n); EnsureState(ref _k2, n); EnsureState(ref _k3, n); EnsureState(ref _k4, n);
        EnsureState(ref _k5, n); EnsureState(ref _k6, n); EnsureState(ref _k7, n); EnsureState(ref _y1, n);
    }

    public override double Step(BodySet b, ForceModel f, double dt)
    {
        if (dt <= 0) return 0;
        EnsureScratch(b.Count);
        if (N == 0) return dt;
        Load(b, Y0);
        _fsalValid = false; // bodies may have been modified externally between calls
        if (_h <= 0 || _h > dt) _h = dt;
        double minH = dt * 1e-9;
        double t = 0;
        int guard = 0;
        while (t < dt && guard++ < 1_000_000)
        {
            double h = Math.Min(_h, dt - t);
            bool last = h >= dt - t;
            if (!_fsalValid) Derivative(b, f, Y0, _k1);

            Combine(Tmp, Y0, h, [A21], _k1);
            Derivative(b, f, Tmp, _k2);
            Combine(Tmp, Y0, h, [A31, A32], _k1, _k2);
            Derivative(b, f, Tmp, _k3);
            Combine(Tmp, Y0, h, [A41, A42, A43], _k1, _k2, _k3);
            Derivative(b, f, Tmp, _k4);
            Combine(Tmp, Y0, h, [A51, A52, A53, A54], _k1, _k2, _k3, _k4);
            Derivative(b, f, Tmp, _k5);
            Combine(Tmp, Y0, h, [A61, A62, A63, A64, A65], _k1, _k2, _k3, _k4, _k5);
            Derivative(b, f, Tmp, _k6);
            Combine(_y1, Y0, h, [B1, 0, B3, B4, B5, B6], _k1, _k2, _k3, _k4, _k5, _k6);
            Derivative(b, f, _y1, _k7);

            double err = 0;
            for (int c = 0; c < 6; ++c)
            {
                double[] y0 = Y0[c], y1 = _y1[c], k1 = _k1[c], k3 = _k3[c], k4 = _k4[c], k5 = _k5[c], k6 = _k6[c], k7 = _k7[c];
                for (int i = 0; i < N; ++i)
                {
                    double e = h * (E1 * k1[i] + E3 * k3[i] + E4 * k4[i] + E5 * k5[i] + E6 * k6[i] + E7 * k7[i]);
                    double sc = AbsTolerance + RelTolerance * Math.Max(Math.Abs(y0[i]), Math.Abs(y1[i]));
                    double r = Math.Abs(e) / sc;
                    if (r > err) err = r;
                }
            }

            if (err <= 1.0 || h <= minH || double.IsNaN(err))
            {
                t = last ? dt : t + h;
                (Y0, _y1) = (_y1, Y0);
                (_k1, _k7) = (_k7, _k1);
                _fsalValid = true;
                StepsTaken++;
                double grow = err == 0 ? 5.0 : Math.Min(5.0, Math.Max(0.2, 0.9 * Math.Pow(err, -0.2)));
                // don't let a short final step shrink the persistent step size
                if (!last || h >= _h) _h = h * grow;
            }
            else
            {
                _h = Math.Max(minH, h * Math.Max(0.2, 0.9 * Math.Pow(err, -0.25)));
            }
        }
        Store(Y0, b);
        return dt;
    }
}
