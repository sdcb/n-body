namespace Sdcb.NBody.Scenarios;

internal static class ChaosScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return new DelegateScenario(new ScenarioInfo("chaos-twins", ScenarioCategory.Chaos, "蝴蝶效应", "The Butterfly Effect",
            "两份毕达哥拉斯三体系统同时演化，初始位置仅相差十亿分之一。起初完全重合，随后彻底分道扬镳。",
            "Two copies of the Pythagorean three-body problem whose starting positions differ by one part in a billion. Identical at first — then utterly different."),
            (_, _) => Pythagorean(twin: true));
        yield return new DelegateScenario(new ScenarioInfo("pythagorean", ScenarioCategory.Chaos, "毕达哥拉斯三体问题", "Pythagorean Three-Body",
            "质量 3:4:5 的三颗星静止于直角三角形顶点（Burrau, 1913）。经过一连串近距离交会，最终一对双星与第三颗星分别被抛射。",
            "Masses 3, 4, 5 released from rest at the corners of a 3-4-5 triangle (Burrau, 1913). After a frenzy of close encounters, a binary and a single star are flung apart."),
            (_, _) => Pythagorean(twin: false));
    }

    private static BodySet PythagoreanBodies(double perturb, float tint)
    {
        BodySet b = new(3);
        Rgb[] cols = tint == 0
            ? [Palette.At(0), Palette.At(1), Palette.At(2)]
            : [new(0.35f, 0.85f, 1.4f), new(0.45f, 0.75f, 1.4f), new(0.3f, 1.0f, 1.2f)];
        b.Add(1 + perturb, 3, 0, 0, 0, 0, 3, BodyKind.Star, 0.09f, cols[0]);
        b.Add(-2, -1, 0, 0, 0, 0, 4, BodyKind.Star, 0.10f, cols[1]);
        b.Add(1, -1, 0, 0, 0, 0, 5, BodyKind.Star, 0.11f, cols[2]);
        b.ShiftToCenterOfMassFrame();
        return b;
    }

    private static ScenarioSetup Pythagorean(bool twin)
    {
        BodySet main = PythagoreanBodies(0, 0);
        if (twin)
        {
            // warm palette for the original, cool for the twin
            main.Color[0] = new(1.4f, 0.55f, 0.2f); main.Color[1] = new(1.35f, 0.75f, 0.25f); main.Color[2] = new(1.3f, 0.4f, 0.35f);
        }
        return new ScenarioSetup
        {
            Bodies = main,
            Twin = twin ? PythagoreanBodies(1e-9, 1) : null,
            Integrator = IntegratorKind.DormandPrince45,
            Dt = 0.004,
            TimeScale = 2.2,
            ViewRadius = 4.5,
            TrailDuration = 6,
            CameraPitchDegrees = 70,
        };
    }
}
