namespace Sdcb.NBody;

public enum BodyKind
{
    Star,
    Planet,
    Moon,
    BlackHole,
    Particle,
}

/// <summary>Linear color hint, components nominally in 0..1 (may exceed 1 for HDR).</summary>
public readonly record struct Rgb(float R, float G, float B)
{
    public static Rgb operator *(Rgb c, float s) => new(c.R * s, c.G * s, c.B * s);

    public static Rgb Lerp(Rgb a, Rgb b, float t) =>
        new(a.R + (b.R - a.R) * t, a.G + (b.G - a.G) * t, a.B + (b.B - a.B) * t);

    /// <summary>Default color hint for a body kind.</summary>
    public static Rgb DefaultFor(BodyKind kind) => kind switch
    {
        BodyKind.Star => new(1.6f, 1.15f, 0.6f),
        BodyKind.Planet => new(0.45f, 0.75f, 1.2f),
        BodyKind.Moon => new(0.8f, 0.8f, 0.85f),
        BodyKind.BlackHole => new(1.2f, 0.7f, 1.6f),
        _ => new(0.85f, 0.9f, 1.0f),
    };
}
