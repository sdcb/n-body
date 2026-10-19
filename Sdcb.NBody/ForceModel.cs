using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Sdcb.NBody;

/// <summary>Newtonian gravity by direct summation with optional Plummer softening.</summary>
public sealed class ForceModel(double g = 1.0, double softening = 0.0)
{
    private const int ParallelThreshold = 256;

    public double G { get; set; } = g;

    /// <summary>Plummer softening length ε: the force uses (r² + ε²)^(-3/2).</summary>
    public double Softening { get; set; } = softening;

    public void ComputeAccelerations(BodySet b, double[] ax, double[] ay, double[] az) =>
        ComputeAccelerations(b, b.Px, b.Py, b.Pz, ax, ay, az);

    public void ComputeAccelerations(BodySet b, double[] px, double[] py, double[] pz, double[] ax, double[] ay, double[] az)
    {
        int n = b.Count;
        if (n >= ParallelThreshold)
        {
            double[] mass = b.Mass;
            double eps2 = Softening * Softening, g = G;
            Parallel.For(0, n, i => Row(i, n, px, py, pz, mass, eps2, g, ax, ay, az));
        }
        else
        {
            Pairwise(b, px, py, pz, ax, ay, az);
        }
    }

    private void Pairwise(BodySet b, double[] px, double[] py, double[] pz, double[] ax, double[] ay, double[] az)
    {
        int n = b.Count;
        double eps2 = Softening * Softening;
        double[] m = b.Mass;
        Array.Clear(ax, 0, n); Array.Clear(ay, 0, n); Array.Clear(az, 0, n);
        for (int i = 0; i < n; ++i)
        {
            double xi = px[i], yi = py[i], zi = pz[i];
            double axi = 0, ayi = 0, azi = 0;
            for (int j = i + 1; j < n; ++j)
            {
                double dx = px[j] - xi, dy = py[j] - yi, dz = pz[j] - zi;
                double r2 = dx * dx + dy * dy + dz * dz + eps2;
                if (r2 < 1e-300) continue;
                double inv = 1.0 / Math.Sqrt(r2);
                double inv3 = inv * inv * inv;
                double sj = m[j] * inv3, si = m[i] * inv3;
                axi += dx * sj; ayi += dy * sj; azi += dz * sj;
                ax[j] -= dx * si; ay[j] -= dy * si; az[j] -= dz * si;
            }
            ax[i] += axi; ay[i] += ayi; az[i] += azi;
        }
        double g = G;
        for (int i = 0; i < n; ++i) { ax[i] *= g; ay[i] *= g; az[i] *= g; }
    }

    /// <summary>Acceleration on body i from all others; SIMD over j.</summary>
    private static void Row(int i, int n, double[] px, double[] py, double[] pz, double[] mass, double eps2, double g,
        double[] ax, double[] ay, double[] az)
    {
        double xi = px[i], yi = py[i], zi = pz[i];
        int w = Vector<double>.Count;
        Vector<double> vxi = new(xi), vyi = new(yi), vzi = new(zi), veps = new(eps2), one = Vector<double>.One;
        Vector<double> sx = Vector<double>.Zero, sy = Vector<double>.Zero, sz = Vector<double>.Zero;
        ReadOnlySpan<Vector<double>> PX = MemoryMarshal.Cast<double, Vector<double>>(px.AsSpan(0, n));
        ReadOnlySpan<Vector<double>> PY = MemoryMarshal.Cast<double, Vector<double>>(py.AsSpan(0, n));
        ReadOnlySpan<Vector<double>> PZ = MemoryMarshal.Cast<double, Vector<double>>(pz.AsSpan(0, n));
        ReadOnlySpan<Vector<double>> M = MemoryMarshal.Cast<double, Vector<double>>(mass.AsSpan(0, n));
        int blocks = PX.Length;
        for (int k = 0; k < blocks; ++k)
        {
            Vector<double> dx = PX[k] - vxi, dy = PY[k] - vyi, dz = PZ[k] - vzi;
            Vector<double> r2 = dx * dx + dy * dy + dz * dz + veps;
            // self-interaction (r2 == 0 without softening) contributes nothing
            Vector<double> valid = Vector.GreaterThan(r2, new Vector<double>(1e-300)).As<long, double>();
            Vector<double> safe = Vector.ConditionalSelect(valid.As<double, long>(), r2, one);
            Vector<double> inv = one / Vector.SquareRoot(safe);
            Vector<double> s = Vector.ConditionalSelect(valid.As<double, long>(), M[k] * inv * inv * inv, Vector<double>.Zero);
            sx += dx * s; sy += dy * s; sz += dz * s;
        }
        double rx = Vector.Sum(sx), ry = Vector.Sum(sy), rz = Vector.Sum(sz);
        for (int j = blocks * w; j < n; ++j)
        {
            double dx = px[j] - xi, dy = py[j] - yi, dz = pz[j] - zi;
            double r2 = dx * dx + dy * dy + dz * dz + eps2;
            if (r2 < 1e-300) continue;
            double inv = 1.0 / Math.Sqrt(r2);
            double s = mass[j] * inv * inv * inv;
            rx += dx * s; ry += dy * s; rz += dz * s;
        }
        ax[i] = rx * g; ay[i] = ry * g; az[i] = rz * g;
    }

    public double PotentialEnergy(BodySet b)
    {
        int n = b.Count;
        double eps2 = Softening * Softening, u = 0;
        for (int i = 0; i < n; ++i)
        {
            double s = 0;
            for (int j = i + 1; j < n; ++j)
            {
                double dx = b.Px[j] - b.Px[i], dy = b.Py[j] - b.Py[i], dz = b.Pz[j] - b.Pz[i];
                double r = Math.Sqrt(dx * dx + dy * dy + dz * dz + eps2);
                if (r > 0) s += b.Mass[j] / r;
            }
            u -= b.Mass[i] * s;
        }
        return u * G;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void EnsureLength(ref double[] a, int n)
    {
        if (a.Length < n) a = new double[Math.Max(n, a.Length * 2)];
    }
}
