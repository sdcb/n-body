namespace Sdcb.NBody.Scenarios;

internal static class SandboxScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return new DelegateScenario(new ScenarioInfo("sandbox", ScenarioCategory.Sandbox, "引力沙盒", "Gravity Sandbox",
            "按住 Shift 拖拽鼠标，向恒星系发射新的天体。相撞的天体会合并为一体。",
            "Hold Shift and drag to launch new bodies into the system. Colliding bodies merge."),
            (_, _) => Create());
        yield return new DelegateScenario(new ScenarioInfo("binary-star-planets", ScenarioCategory.SolarSystem, "双星与环双星行星", "Circumbinary Planets",
            "类似开普勒-16 的双星系统，三颗行星绕着两颗恒星公转——“塔图因”的天空有两个太阳。",
            "A Kepler-16-like binary with three circumbinary planets — Tatooine's sky has two suns."),
            (_, _) => Binary());
    }

    private static ScenarioSetup Create()
    {
        BodySet b = new(32);
        b.Add(0, 0, 0, 0, 0, 0, 1.0, BodyKind.Star, 0.12f, new Rgb(1.7f, 1.15f, 0.55f), "Sun");
        AddCircular(b, 1.0, 1e-3, 0.045f, new Rgb(0.4f, 0.7f, 1.2f), 0.3, "I");
        AddCircular(b, 1.7, 3e-4, 0.035f, new Rgb(1.1f, 0.55f, 0.3f), 2.4, "II");
        AddCircular(b, 2.6, 2e-3, 0.06f, new Rgb(1.0f, 0.82f, 0.55f), 4.1, "III");
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Integrator = IntegratorKind.Yoshida4,
            Dt = 0.002,
            TimeScale = 1.2,
            ViewRadius = 3.2,
            TrailDuration = 12,
            MergeOnCollision = true,
            CameraPitchDegrees = 55,
        };
    }

    private static void AddCircular(BodySet b, double r, double m, float radius, Rgb color, double phase, string name, double centralMass = 1.0)
    {
        double v = Math.Sqrt(centralMass / r);
        b.Add(r * Math.Cos(phase), r * Math.Sin(phase), 0, -v * Math.Sin(phase), v * Math.Cos(phase), 0, m, BodyKind.Planet, radius, color, name);
    }

    private static ScenarioSetup Binary()
    {
        BodySet b = new(8);
        double m1 = 1.0, m2 = 0.45, a = 1.0, mt = m1 + m2;
        double v = Math.Sqrt(mt / a);
        b.Add(-a * m2 / mt, 0, 0, 0, -v * m2 / mt, 0, m1, BodyKind.Star, 0.11f, new Rgb(1.6f, 1.0f, 0.5f), "A");
        b.Add(a * m1 / mt, 0, 0, 0, v * m1 / mt, 0, m2, BodyKind.Star, 0.08f, new Rgb(1.4f, 0.5f, 0.3f), "B");
        AddCircular(b, 3.4, 2e-4, 0.05f, new Rgb(0.95f, 0.8f, 0.55f), 0.0, "b", mt);
        AddCircular(b, 5.0, 1e-4, 0.045f, new Rgb(0.4f, 0.7f, 1.2f), 2.0, "c", mt);
        AddCircular(b, 7.2, 3e-4, 0.06f, new Rgb(0.6f, 0.9f, 1.0f), 4.2, "d", mt);
        for (int i = 2; i < b.Count; ++i) b.Pz[i] = 0.05 * (i - 3);
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Integrator = IntegratorKind.Yoshida4,
            Dt = 0.002,
            TimeScale = 3.5,
            ViewRadius = 8,
            TrailDuration = 160,
            CameraPitchDegrees = 40,
        };
    }
}
