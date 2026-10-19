namespace Sdcb.NBody.App.Graphics;

internal sealed record QualityTier(string Name, int ParticleBudget, int MaxGpuStepsPerFrame, int BloomLevels)
{
    public static QualityTier Detect(GraphicsDevice g)
    {
        ulong vramMb = g.DedicatedVideoMemory / (1024 * 1024);
        bool discrete = g.VendorId is 0x10DE or 0x1002 && vramMb >= 2048; // NVIDIA / AMD
        string? overrideTier = Environment.GetEnvironmentVariable("NBODY_QUALITY");
        return (overrideTier?.ToLowerInvariant()) switch
        {
            "ultra" => Ultra,
            "high" => High,
            "medium" => Medium,
            "low" => Low,
            _ when discrete && vramMb >= 10_000 => Ultra,
            _ when discrete && vramMb >= 6_000 => High,
            _ when vramMb >= 2_000 || discrete => Medium,
            _ => Low,
        };
    }

    public static readonly QualityTier Ultra = new("Ultra", 65536, 1, 7);
    public static readonly QualityTier High = new("High", 36864, 1, 7);
    public static readonly QualityTier Medium = new("Medium", 16384, 1, 6);
    public static readonly QualityTier Low = new("Low", 8192, 1, 5);
}
