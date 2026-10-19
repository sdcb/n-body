namespace Sdcb.NBody.Scenarios;

internal static class PeriodicThreeBodyScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return Make("figure-8", "8 字形轨道", "Figure-Eight",
            "Chenciner 与 Montgomery 于 2000 年证明存在的三体编舞：三颗等质量星沿同一条 8 字形曲线彼此追逐。",
            "The choreography proven by Chenciner & Montgomery (2000): three equal masses chase each other along a single figure-eight.",
            () => Bodies((-0.97000436, 0.24308753, 0.466203685, 0.43236573), (0.97000436, -0.24308753, 0.466203685, 0.43236573), (0, 0, -0.93240737, -0.86473146)),
            period: 6.32591398);

        foreach (var s in Suvakov()) yield return s;

        yield return Make("broucke-a15", "Broucke A15", "Broucke A15",
            "Broucke 在 1975 年数值搜索到的共线初值周期解之一，三体在复杂的回旋中不断交换位置。",
            "One of Broucke's 1975 periodic solutions from collinear initial conditions — an intricate dance of swaps and loops.",
            () => Collinear((-1.1889693067, 0.8042120498), (3.8201881837, 0.0212794833), (-2.631218877, -0.8254915331)), period: null, fallbackTrail: 9);
        yield return Make("henon-3", "Hénon 3", "Hénon 3",
            "Hénon 族周期解：一颗星在两颗星之间穿梭，画出层叠的花瓣。",
            "A Hénon-family orbit: one star shuttles between the other two, tracing layered petals.",
            () => Collinear((-0.9738300580, 4.3072892019), (1.9988948637, 0.1333821680), (-1.0250648057, -4.4406713699)), period: null, fallbackTrail: 5);
        yield return Make("henon-5", "Hénon 5", "Hénon 5",
            "原项目默认展示的 Hénon 解，现在有了 3D 视角和 HDR 光效。",
            "The original project's default Hénon orbit — now in 3D with HDR light.",
            () => Collinear((-0.9353825545, 3.3166932522), (1.9545571553, 0.1654488998), (-1.0191746008, -3.4821421520)), period: null, fallbackTrail: 5);
        yield return Make("henon-42", "Hénon 42", "Hénon 42",
            "三颗星以不同的节奏交替领舞，轨迹宛如缠绕的丝带。",
            "Three stars take turns leading, weaving ribbons through space.",
            () => Collinear((1.1593879407, 1.1787714143), (1.7740754142, -0.6271771385), (-2.9334633549, -0.5515942758)), period: null, fallbackTrail: 8);
        yield return Make("lagrange-triangle", "拉格朗日三角", "Lagrange Triangle",
            "等边三角形刚性旋转的精确解。等质量情形不稳定——微小误差终将撕裂这个完美构型。",
            "Lagrange's rigidly rotating equilateral solution. With equal masses it is unstable — tiny errors eventually tear it apart.",
            Lagrange, period: 2 * Math.PI * Math.Sqrt(Math.Sqrt(3)), fallbackTrail: 6, integrator: IntegratorKind.Yoshida4);
    }

    private static IEnumerable<Scenario> Suvakov()
    {
        // Šuvakov & Dmitrašinović (2013): x1 = -1, x2 = 1, x3 = 0; v1 = v2 = (p1, p2), v3 = -2 (p1, p2)
        (string id, string zh, string en, double p1, double p2, double T, string dzh, string den)[] list =
        [
            ("butterfly-1", "蝴蝶 I", "Butterfly I", 0.306893, 0.125507, 6.2356, "2013 年 Šuvakov 与 Dmitrašinović 发现的 13 族新周期解之一，轨迹如蝴蝶展翅。", "One of 13 new families found by Šuvakov & Dmitrašinović in 2013 — the orbit spreads like butterfly wings."),
            ("butterfly-2", "蝴蝶 II", "Butterfly II", 0.392955, 0.097579, 7.0039, "蝴蝶族的另一个成员，翅膀更宽，节奏更快。", "Another butterfly — wider wings, faster rhythm."),
            ("butterfly-3", "蝴蝶 III", "Butterfly III", 0.405916, 0.230163, 13.8658, "多层翅膀的蝴蝶，三颗星在两翼间反复穿梭。", "A many-layered butterfly; the stars shuttle between the wings."),
            ("moth-1", "飞蛾 I", "Moth I", 0.464445, 0.396060, 14.8939, "飞蛾族：轨迹在中心交织，再向两侧舒展。", "Moth family: paths braid at the centre, then unfold to both sides."),
            ("moth-2", "飞蛾 II", "Moth II", 0.439166, 0.452968, 28.6703, "更长周期的飞蛾，图案层层嵌套。", "A longer-period moth with nested patterns."),
            ("moth-3", "飞蛾 III", "Moth III", 0.383444, 0.377364, 25.8406, "飞蛾 III：密集的回环构成蛾翼的纹理。", "Moth III: dense loops form the texture of its wings."),
            ("goggles", "护目镜", "Goggles", 0.083300, 0.127889, 10.4668, "两只“镜片”之间来回弹跳，形如一副护目镜。", "Bouncing between two 'lenses' — a pair of goggles."),
            ("dragonfly", "蜻蜓", "Dragonfly", 0.080584, 0.588836, 21.2710, "细长的身体与透明的双翼，三体版的蜻蜓。", "A slender body and glassy wings — a three-body dragonfly."),
            ("yin-yang-1a", "阴阳 Ia", "Yin-Yang Ia", 0.513938, 0.304736, 17.3284, "阴阳族：两半互补的图案，暗合太极。", "Yin-Yang family: two complementary halves, like the taijitu."),
            ("yin-yang-1b", "阴阳 Ib", "Yin-Yang Ib", 0.282699, 0.327209, 10.9626, "阴阳族的另一解，更紧凑的双涡结构。", "A more compact yin-yang with twin vortices."),
            ("bumblebee", "大黄蜂", "Bumblebee", 0.184279, 0.587188, 63.5345, "周期长达 63.5 的复杂编舞，嗡嗡盘旋的大黄蜂。", "A 63.5-unit epic of loops — the buzzing bumblebee."),
            ("yarn", "毛线团", "Yarn", 0.559064, 0.349192, 55.5018, "无数回环缠成一团毛线，是最复杂的周期解之一。", "Countless loops wound into a ball of yarn — one of the most intricate solutions."),
        ];
        foreach (var o in list)
        {
            double p1 = o.p1, p2 = o.p2;
            yield return Make(o.id, o.zh, o.en, o.dzh, o.den,
                () => Bodies((-1, 0, p1, p2), (1, 0, p1, p2), (0, 0, -2 * p1, -2 * p2)), period: o.T);
        }
    }

    private static BodySet Bodies(params (double x, double y, double vx, double vy)[] b)
    {
        BodySet set = new(3);
        for (int i = 0; i < b.Length; ++i)
        {
            set.Add(b[i].x, b[i].y, 0, b[i].vx, b[i].vy, 0, 1.0, BodyKind.Star, 0.05f, Palette.At(i));
        }
        set.ShiftToCenterOfMassFrame();
        return set;
    }

    /// <summary>Collinear start on the x axis with velocities along y.</summary>
    private static BodySet Collinear(params (double x, double vy)[] b) =>
        Bodies([.. b.Select(t => (t.x, 0.0, 0.0, t.vy))]);

    private static BodySet Lagrange()
    {
        BodySet set = new(3);
        double r = 1, v = Math.Sqrt(1.0 / (Math.Sqrt(3) * r));
        for (int i = 0; i < 3; ++i)
        {
            double a = 2 * Math.PI * i / 3 + Math.PI / 2;
            set.Add(r * Math.Cos(a), r * Math.Sin(a), 0, -v * Math.Sin(a), v * Math.Cos(a), 0, 1, BodyKind.Star, 0.05f, Palette.At(i));
        }
        return set;
    }

    private static Scenario Make(string id, string zh, string en, string dzh, string den, Func<BodySet> bodies, double? period,
        double fallbackTrail = 8, IntegratorKind integrator = IntegratorKind.DormandPrince45)
    {
        ScenarioInfo info = new(id, ScenarioCategory.PeriodicThreeBody, zh, en, dzh, den);
        return new DelegateScenario(info, (_, _) =>
        {
            BodySet b = bodies();
            double span = period ?? fallbackTrail;
            double radius = Framing.MaxRadius(b, 1.0, 0, Math.Min(span, 30), samples: 600);
            float bodyRadius = (float)(radius * 0.035);
            for (int i = 0; i < b.Count; ++i) b.Radius[i] = bodyRadius;
            double seconds = Math.Clamp(8 + span * 0.3, 9, 30);
            return new ScenarioSetup
            {
                Bodies = b,
                G = 1,
                Integrator = integrator,
                Dt = span / 2400,
                TimeScale = span / seconds,
                ViewRadius = radius * 1.08,
                TrailDuration = span,
                Period = period,
                CameraPitchDegrees = 62,
            };
        });
    }
}

internal static class Framing
{
    /// <summary>Simulates a copy of the bodies and returns the maximum distance from the origin reached.</summary>
    public static double MaxRadius(BodySet bodies, double g, double softening, double duration, int samples)
    {
        BodySet b = bodies.Clone();
        ForceModel f = new(g, softening);
        IIntegrator integrator = Integrators.Create(IntegratorKind.DormandPrince45);
        if (integrator is DormandPrince45Integrator dp) { dp.AbsTolerance = 1e-8; dp.RelTolerance = 1e-8; }
        double max = Radius(b), h = duration / samples;
        for (int s = 0; s < samples; ++s)
        {
            integrator.Step(b, f, h);
            max = Math.Max(max, Radius(b));
        }
        return Math.Max(max, 1e-3);
    }

    private static double Radius(BodySet b)
    {
        double m = 0;
        for (int i = 0; i < b.Count; ++i)
        {
            m = Math.Max(m, Math.Sqrt(b.Px[i] * b.Px[i] + b.Py[i] * b.Py[i] + b.Pz[i] * b.Pz[i]));
        }
        return m;
    }
}
