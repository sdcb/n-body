namespace Sdcb.NBody.Scenarios;

internal static class Palette
{
    public static readonly Rgb[] Trio =
    [
        new(1.35f, 0.62f, 0.22f),   // amber
        new(0.30f, 0.80f, 1.40f),   // cyan
        new(1.30f, 0.32f, 0.78f),   // magenta
        new(0.55f, 1.25f, 0.45f),   // lime
        new(1.30f, 1.10f, 0.40f),   // gold
        new(0.62f, 0.50f, 1.40f),   // violet
    ];

    public static Rgb At(int i) => Trio[i % Trio.Length];

    /// <summary>Smooth rainbow for rings (hue in [0,1)).</summary>
    public static Rgb Hue(double h, float intensity = 1.3f)
    {
        double r = Math.Clamp(Math.Abs(h * 6 - 3) - 1, 0, 1);
        double g = Math.Clamp(2 - Math.Abs(h * 6 - 2), 0, 1);
        double b = Math.Clamp(2 - Math.Abs(h * 6 - 4), 0, 1);
        return new Rgb((float)(0.25 + 0.75 * r) * intensity, (float)(0.25 + 0.75 * g) * intensity, (float)(0.25 + 0.75 * b) * intensity);
    }

    /// <summary>Stellar colour by temperature t in [0,1] (red dwarf → blue giant).</summary>
    public static Rgb Star(double t, float intensity = 1f)
    {
        Rgb red = new(1.0f, 0.42f, 0.18f), yellow = new(1.0f, 0.85f, 0.6f), white = new(0.92f, 0.95f, 1.0f), blue = new(0.55f, 0.72f, 1.0f);
        Rgb c = t < 0.33 ? Rgb.Lerp(red, yellow, (float)(t / 0.33))
              : t < 0.66 ? Rgb.Lerp(yellow, white, (float)((t - 0.33) / 0.33))
              : Rgb.Lerp(white, blue, (float)((t - 0.66) / 0.34));
        return c * intensity;
    }
}

internal static class RandomExtensions
{
    public static double NextGaussian(this Random r)
    {
        double u1 = 1.0 - r.NextDouble(), u2 = r.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
    }
}
