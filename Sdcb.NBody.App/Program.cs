using Sdcb.NBody.App.Platform;

namespace Sdcb.NBody.App;

internal sealed record AppOptions(bool Windowed, string? Scenario, bool NoAttract, double CaptureAt, string? CapturePath, int Width, int Height)
{
    public static AppOptions Parse(string[] args)
    {
        string? Value(string name)
        {
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
        string? capture = Value("--capture");
        return new AppOptions(
            args.Contains("--windowed") || args.Contains("-w") || capture != null,
            Value("--scenario"),
            args.Contains("--no-attract") || Value("--scenario") != null,
            double.TryParse(Value("--capture-at"), System.Globalization.CultureInfo.InvariantCulture, out double t) ? t : 6,
            capture,
            int.TryParse(Value("--width"), out int w) ? w : 1600,
            int.TryParse(Value("--height"), out int h) ? h : 900);
    }
}

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        AppOptions options = AppOptions.Parse(args);
        s_silent = options.CapturePath != null || args.Contains("--check-shaders");
        if (args.Contains("--check-shaders")) return CheckShaders();
        try
        {
            using NBodyApp app = new(options);
            app.Run();
            return 0;
        }
        catch (Exception ex)
        {
            ReportFatal(ex);
            return 1;
        }
    }

    private static bool s_silent;

    /// <summary>Compiles every shader entry point without creating a window; prints errors and returns non-zero on failure.</summary>
    private static int CheckShaders()
    {
        (string File, string Entry, string Profile)[] all =
        [
            ("starfield.hlsl", "VSMain", "vs_5_0"), ("starfield.hlsl", "PSMain", "ps_5_0"),
            ("trail.hlsl", "VSMain", "vs_5_0"), ("trail.hlsl", "PSMain", "ps_5_0"),
            ("body.hlsl", "VSMain", "vs_5_0"), ("body.hlsl", "PSMain", "ps_5_0"),
            ("particles.hlsl", "VSMain", "vs_5_0"), ("particles.hlsl", "PSMain", "ps_5_0"),
            ("nbody.hlsl", "CSKickDrift", "cs_5_0"), ("nbody.hlsl", "CSAccel", "cs_5_0"), ("nbody.hlsl", "CSKick", "cs_5_0"), ("nbody.hlsl", "CSGatherBodies", "cs_5_0"),
            ("post.hlsl", "VSFullscreen", "vs_5_0"), ("post.hlsl", "PSDownsampleFirst", "ps_5_0"), ("post.hlsl", "PSDownsample", "ps_5_0"),
            ("post.hlsl", "PSUpsample", "ps_5_0"), ("post.hlsl", "PSComposite", "ps_5_0"),
        ];
        int failures = 0;
        foreach (var s in all)
        {
            try
            {
                Graphics.ShaderLibrary.Compile(s.File, s.Entry, s.Profile);
                Console.WriteLine($"OK   {s.File}:{s.Entry}");
            }
            catch (Exception ex)
            {
                failures++;
                Console.WriteLine($"FAIL {ex.Message}");
            }
        }
        return failures;
    }

    public static void ReportFatal(Exception ex)
    {
        string text = ex.ToString();
        try
        {
            File.WriteAllText(Path.Combine(AppContext.BaseDirectory, "nbody-crash.log"), text);
        }
        catch
        {
        }
        if (!s_silent) Win32.MessageBox(0, text, "N-Body", 0x10);
    }
}
