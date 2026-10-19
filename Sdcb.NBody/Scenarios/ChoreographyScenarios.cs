namespace Sdcb.NBody.Scenarios;

internal static class ChoreographyScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return Ring("ring-4", 4, "四星方阵", "Square of Four",
            "四颗等质量恒星组成的旋转正方形，精确平衡却注定失稳。",
            "Four equal stars in a rotating square — perfectly balanced, ultimately unstable.");
        yield return Ring("ring-6", 6, "六星环", "Ring of Six",
            "六颗星的刚性旋转环：对称性带来的平衡，以及失衡后壮观的崩塌。",
            "A rigidly rotating ring of six: symmetry-born balance, and a spectacular collapse once it breaks.");
        yield return Ring("ring-12", 12, "十二星冠", "Crown of Twelve",
            "十二颗星的皇冠。数值误差会被放大，看看它能维持多久。",
            "A crown of twelve stars. Numerical noise grows — how long will it hold?");
        yield return new DelegateScenario(new ScenarioInfo("saturn-ring", ScenarioCategory.Choreography, "中心星与卫星环", "Moonlet Ring",
            "一颗中心恒星与 36 颗小卫星组成的稳定环，微弱的相互引力让环缓慢起伏。",
            "A star encircled by 36 moonlets; their faint mutual pull makes the ring ripple."), (_, _) => SaturnRing());
    }

    private static Scenario Ring(string id, int n, string zh, string en, string dzh, string den) =>
        new DelegateScenario(new ScenarioInfo(id, ScenarioCategory.Choreography, zh, en, dzh, den), (_, _) =>
        {
            BodySet b = CreateStableRing(n);
            double v = Math.Sqrt(b.Vx[0] * b.Vx[0] + b.Vy[0] * b.Vy[0]);
            double period = 2 * Math.PI / v;
            for (int i = 0; i < n; ++i) b.Radius[i] = 0.045f;
            return new ScenarioSetup
            {
                Bodies = b,
                Integrator = IntegratorKind.DormandPrince45,
                Dt = period / 1500,
                TimeScale = period / 8,
                ViewRadius = 1.6,
                TrailDuration = period * 0.9,
                Period = period,
                CameraPitchDegrees = 55,
            };
        });

    /// <summary>N equal masses on a circle with the exact speed for rigid rotation.</summary>
    public static BodySet CreateStableRing(int n, double g = 1.0, double m = 1.0, double r = 1.0)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(n, 2);
        double sum = 0;
        for (int k = 1; k < n; ++k) sum += 1.0 / Math.Sin(Math.PI * k / n);
        double v = Math.Sqrt(g * m / (4.0 * r) * sum);
        BodySet b = new(n);
        for (int i = 0; i < n; ++i)
        {
            double a = 2 * Math.PI * i / n;
            b.Add(r * Math.Cos(a), r * Math.Sin(a), 0, -v * Math.Sin(a), v * Math.Cos(a), 0, m, BodyKind.Star, 0.05f, Palette.Hue((double)i / n));
        }
        return b;
    }

    private static ScenarioSetup SaturnRing()
    {
        BodySet b = new(40);
        b.Add(0, 0, 0, 0, 0, 0, 1.0, BodyKind.Star, 0.14f, new Rgb(1.5f, 1.05f, 0.55f), "Sun");
        const int n = 36;
        const double m = 2e-5;
        Random rng = new(5);
        for (int i = 0; i < n; ++i)
        {
            double a = 2 * Math.PI * i / n;
            double r = 1.0 + 0.004 * rng.NextGaussian();
            double v = Math.Sqrt((1.0 + n * m * 0.5) / r);
            b.Add(r * Math.Cos(a), r * Math.Sin(a), 0.01 * rng.NextGaussian(), -v * Math.Sin(a), v * Math.Cos(a), 0, m,
                BodyKind.Moon, 0.018f, Palette.Hue((double)i / n, 1.1f));
        }
        b.ShiftToCenterOfMassFrame();
        double period = 2 * Math.PI;
        return new ScenarioSetup
        {
            Bodies = b,
            Integrator = IntegratorKind.Yoshida4,
            Dt = period / 800,
            TimeScale = period / 6,
            ViewRadius = 1.4,
            TrailDuration = period * 0.25,
            CameraPitchDegrees = 30,
        };
    }
}
