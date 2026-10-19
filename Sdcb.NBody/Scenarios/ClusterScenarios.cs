namespace Sdcb.NBody.Scenarios;

internal static class ClusterScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return new DelegateScenario(new ScenarioInfo("protoplanetary-disk", ScenarioCategory.Cluster, "原行星盘", "Protoplanetary Disk",
            "年轻恒星周围的尘埃盘中嵌着三颗巨行星。它们会清扫出环形空隙，并激起螺旋密度波——ALMA 望远镜看到的正是这种结构。",
            "Three giant planets embedded in a young star's dusty disk carve out gaps and launch spiral wakes — the structures ALMA actually sees."),
            (budget, seed) => Protoplanetary(budget, seed));
        yield return new DelegateScenario(new ScenarioInfo("plummer-cluster", ScenarioCategory.Cluster, "球状星团", "Globular Cluster",
            "依照 Plummer 模型生成的球状星团，初始略微“偏冷”，于是先坍缩、再反弹，最终趋于维里平衡。",
            "A Plummer-model globular cluster started slightly 'cold' — it collapses, rebounds and settles into virial equilibrium."),
            (budget, seed) => Plummer(budget, seed));
        yield return new DelegateScenario(new ScenarioInfo("cold-collapse", ScenarioCategory.Cluster, "冷坍缩", "Cold Collapse",
            "静止的均匀球体在自引力下自由坍缩，经历剧烈弛豫后形成核-晕结构。颜色标记了初始半径，可以看清物质如何混合。",
            "A uniform sphere at rest free-falls under its own gravity and violently relaxes into a core–halo system. Colours mark the initial radius, revealing how matter mixes."),
            (budget, seed) => ColdCollapse(budget, seed));
    }

    private static ScenarioSetup Plummer(int budget, int seed)
    {
        Random rng = new(seed);
        int n = Math.Max(256, budget / 2);
        const double a = 0.45, m = 1.0, cold = 0.85;
        BodySet b = new(n);
        for (int i = 0; i < n; ++i)
        {
            double r;
            do { r = a / Math.Sqrt(Math.Pow(Math.Max(rng.NextDouble(), 1e-10), -2.0 / 3.0) - 1); } while (r > 6 * a);
            var (x, y, z) = Isotropic(rng, r);
            double qv, g;
            do { qv = rng.NextDouble(); g = rng.NextDouble() * 0.1; } while (g > qv * qv * Math.Pow(1 - qv * qv, 3.5));
            double vesc = Math.Sqrt(2 * m / a) * Math.Pow(1 + r * r / (a * a), -0.25);
            var (vx, vy, vz) = Isotropic(rng, qv * vesc * cold);
            double t = rng.NextDouble();
            Rgb c = t < 0.04 ? Palette.Star(0.95, 1.4f) : Palette.Star(Math.Pow(rng.NextDouble(), 1.6) * 0.62, (float)(0.7 + 0.6 * rng.NextDouble()));
            b.Add(x, y, z, vx, vy, vz, m / n, BodyKind.Particle, 0.01f, c);
        }
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Softening = 0.025,
            Integrator = IntegratorKind.Leapfrog,
            Dt = 0.008,
            TimeScale = 0.45,
            ViewRadius = 2.0,
            TrailDuration = 0,
            GpuSimulated = true,
            CameraPitchDegrees = 20,
        };
    }

    private static ScenarioSetup ColdCollapse(int budget, int seed)
    {
        Random rng = new(seed);
        int n = Math.Max(256, budget / 2);
        const double radius = 1.5, omega = 0.18;
        BodySet b = new(n);
        for (int i = 0; i < n; ++i)
        {
            double r = radius * Math.Cbrt(rng.NextDouble());
            var (x, y, z) = Isotropic(rng, r);
            double t = r / radius;
            Rgb c = Rgb.Lerp(new Rgb(0.45f, 0.75f, 1.5f), new Rgb(1.5f, 0.6f, 0.25f), (float)t) * (float)(0.8 + 0.4 * rng.NextDouble());
            b.Add(x, y, z, -omega * y, omega * x, 0, 1.0 / n, BodyKind.Particle, 0.01f, c);
        }
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Softening = 0.03,
            Integrator = IntegratorKind.Leapfrog,
            Dt = 0.006,
            TimeScale = 0.3,
            ViewRadius = 2.2,
            TrailDuration = 0,
            GpuSimulated = true,
            CameraPitchDegrees = 25,
        };
    }

    private static ScenarioSetup Protoplanetary(int budget, int seed)
    {
        Random rng = new(seed);
        int n = Math.Max(256, budget / 3);
        BodySet b = new(n + 4);
        b.Add(0, 0, 0, 0, 0, 0, 1.0, BodyKind.Star, 0.09f, new Rgb(1.7f, 1.2f, 0.6f), "Star");
        (double r, double m, float rad, Rgb c)[] planets =
        [
            (1.3, 1.5e-3, 0.05f, new Rgb(1.0f, 0.75f, 0.5f)),
            (2.2, 1.0e-3, 0.045f, new Rgb(0.5f, 0.8f, 1.2f)),
            (3.3, 6.0e-4, 0.04f, new Rgb(0.7f, 0.95f, 1.1f)),
        ];
        double phase = 0.7;
        foreach (var p in planets)
        {
            double v = Math.Sqrt((1 + p.m) / p.r);
            b.Add(p.r * Math.Cos(phase), p.r * Math.Sin(phase), 0, -v * Math.Sin(phase), v * Math.Cos(phase), 0, p.m, BodyKind.Planet, p.rad, p.c);
            phase += 2.3;
        }
        int dust = n - b.Count;
        for (int i = 0; i < dust; ++i)
        {
            double r = 0.6 + 3.8 * rng.NextDouble();
            double th = rng.NextDouble() * 2 * Math.PI;
            double v = Math.Sqrt(1.0 / r) * (1 + 0.006 * rng.NextGaussian());
            double z = 0.012 * r * rng.NextGaussian();
            double t = (r - 0.6) / 3.8;
            Rgb c = Rgb.Lerp(new Rgb(1.4f, 0.75f, 0.4f), new Rgb(0.55f, 0.5f, 0.9f), (float)t) * (float)(0.7 + 0.5 * rng.NextDouble());
            b.Add(r * Math.Cos(th), r * Math.Sin(th), z, -v * Math.Sin(th), v * Math.Cos(th), 0.01 * v * rng.NextGaussian(), 1e-9, BodyKind.Particle, 0.01f, c);
        }
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Softening = 0.015,
            Integrator = IntegratorKind.Leapfrog,
            Dt = 0.004,
            TimeScale = 2.5,
            ViewRadius = 4.2,
            TrailDuration = 0,
            GpuSimulated = true,
            CameraPitchDegrees = 52,
        };
    }

    private static (double, double, double) Isotropic(Random rng, double r)
    {
        double cz = rng.NextDouble() * 2 - 1, phi = rng.NextDouble() * 2 * Math.PI, s = Math.Sqrt(1 - cz * cz);
        return (r * s * Math.Cos(phi), r * s * Math.Sin(phi), r * cz);
    }
}
