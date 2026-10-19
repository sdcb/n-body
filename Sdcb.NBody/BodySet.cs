namespace Sdcb.NBody;

/// <summary>
/// Growable Structure-of-Arrays container for bodies. Arrays have length <see cref="Capacity"/> (≥ <see cref="Count"/>);
/// only indices in [0, Count) are valid. Arrays are replaced when the set grows, so do not cache them across <see cref="Add"/> calls.
/// </summary>
public sealed class BodySet
{
    public double[] Px, Py, Pz, Vx, Vy, Vz, Mass;
    public float[] Radius;
    public Rgb[] Color;
    public BodyKind[] Kind;
    public string?[] Name;

    public BodySet(int capacity = 16)
    {
        capacity = Math.Max(capacity, 1);
        Px = new double[capacity]; Py = new double[capacity]; Pz = new double[capacity];
        Vx = new double[capacity]; Vy = new double[capacity]; Vz = new double[capacity];
        Mass = new double[capacity];
        Radius = new float[capacity];
        Color = new Rgb[capacity];
        Kind = new BodyKind[capacity];
        Name = new string?[capacity];
    }

    public int Count { get; private set; }

    public int Capacity => Px.Length;

    /// <summary>Incremented on every structural change (add/remove/clear/copy).</summary>
    public int Version { get; private set; }

    public double TotalMass
    {
        get
        {
            double m = 0;
            for (int i = 0; i < Count; ++i) m += Mass[i];
            return m;
        }
    }

    public void EnsureCapacity(int capacity)
    {
        if (capacity <= Capacity) return;
        int newCap = Math.Max(capacity, Capacity * 2);
        Array.Resize(ref Px, newCap); Array.Resize(ref Py, newCap); Array.Resize(ref Pz, newCap);
        Array.Resize(ref Vx, newCap); Array.Resize(ref Vy, newCap); Array.Resize(ref Vz, newCap);
        Array.Resize(ref Mass, newCap);
        Array.Resize(ref Radius, newCap);
        Array.Resize(ref Color, newCap);
        Array.Resize(ref Kind, newCap);
        Array.Resize(ref Name, newCap);
    }

    public int Add(double px, double py, double pz, double vx, double vy, double vz, double mass,
        BodyKind kind = BodyKind.Star, float radius = 0.05f, Rgb? color = null, string? name = null)
    {
        EnsureCapacity(Count + 1);
        int i = Count++;
        Px[i] = px; Py[i] = py; Pz[i] = pz;
        Vx[i] = vx; Vy[i] = vy; Vz[i] = vz;
        Mass[i] = mass;
        Kind[i] = kind;
        Radius[i] = radius;
        Color[i] = color ?? Rgb.DefaultFor(kind);
        Name[i] = name;
        Version++;
        return i;
    }

    /// <summary>Removes the body at <paramref name="index"/>, shifting later bodies down (order is preserved).</summary>
    public void RemoveAt(int index)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(index);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, Count);
        int tail = Count - index - 1;
        if (tail > 0)
        {
            Array.Copy(Px, index + 1, Px, index, tail); Array.Copy(Py, index + 1, Py, index, tail); Array.Copy(Pz, index + 1, Pz, index, tail);
            Array.Copy(Vx, index + 1, Vx, index, tail); Array.Copy(Vy, index + 1, Vy, index, tail); Array.Copy(Vz, index + 1, Vz, index, tail);
            Array.Copy(Mass, index + 1, Mass, index, tail);
            Array.Copy(Radius, index + 1, Radius, index, tail);
            Array.Copy(Color, index + 1, Color, index, tail);
            Array.Copy(Kind, index + 1, Kind, index, tail);
            Array.Copy(Name, index + 1, Name, index, tail);
        }
        Count--;
        Name[Count] = null;
        Version++;
    }

    public void Clear()
    {
        Array.Clear(Name, 0, Count);
        Count = 0;
        Version++;
    }

    public BodySet Clone()
    {
        BodySet r = new(Math.Max(Count, 1));
        r.CopyFrom(this);
        return r;
    }

    /// <summary>Copies all bodies from <paramref name="other"/>, growing as needed.</summary>
    public void CopyFrom(BodySet other)
    {
        ArgumentNullException.ThrowIfNull(other);
        if (ReferenceEquals(other, this)) return;
        int n = other.Count;
        EnsureCapacity(n);
        if (n < Count) Array.Clear(Name, n, Count - n);
        Array.Copy(other.Px, Px, n); Array.Copy(other.Py, Py, n); Array.Copy(other.Pz, Pz, n);
        Array.Copy(other.Vx, Vx, n); Array.Copy(other.Vy, Vy, n); Array.Copy(other.Vz, Vz, n);
        Array.Copy(other.Mass, Mass, n);
        Array.Copy(other.Radius, Radius, n);
        Array.Copy(other.Color, Color, n);
        Array.Copy(other.Kind, Kind, n);
        Array.Copy(other.Name, Name, n);
        Count = n;
        Version++;
    }

    public (double X, double Y, double Z) CenterOfMass()
    {
        double m = 0, x = 0, y = 0, z = 0;
        for (int i = 0; i < Count; ++i)
        {
            double mi = Mass[i];
            m += mi; x += mi * Px[i]; y += mi * Py[i]; z += mi * Pz[i];
        }
        return m == 0 ? default : (x / m, y / m, z / m);
    }

    public (double X, double Y, double Z) CenterOfMassVelocity()
    {
        double m = 0, x = 0, y = 0, z = 0;
        for (int i = 0; i < Count; ++i)
        {
            double mi = Mass[i];
            m += mi; x += mi * Vx[i]; y += mi * Vy[i]; z += mi * Vz[i];
        }
        return m == 0 ? default : (x / m, y / m, z / m);
    }

    /// <summary>Subtracts the center-of-mass position and velocity from every body.</summary>
    public void ShiftToCenterOfMassFrame()
    {
        var (cx, cy, cz) = CenterOfMass();
        var (cvx, cvy, cvz) = CenterOfMassVelocity();
        for (int i = 0; i < Count; ++i)
        {
            Px[i] -= cx; Py[i] -= cy; Pz[i] -= cz;
            Vx[i] -= cvx; Vy[i] -= cvy; Vz[i] -= cvz;
        }
    }
}
